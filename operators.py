import bpy
import bmesh
import mathutils
import math
import numpy as np
import csv
import os
import json
import time
from mathutils import Matrix
from bpy.types import Operator

class COLLISION_OT_cancel(Operator):
    """Cancel the current collision calculation"""
    bl_idname = "collision.cancel"
    bl_label = "Cancel Calculation"
    
    def execute(self, context):
        props = context.scene.collision_props
        
        if props.is_calculating:
            # Set a flag that the calculation operator will check
            props.is_calculating = False
            self.report({'INFO'}, "Cancelling calculation...")
        else:
            self.report({'WARNING'}, "No calculation is in progress")
            
        return {'FINISHED'}

class COLLISION_OT_calculate(Operator):
    """Calculate object collisions based on rotations and translations"""
    bl_idname = "collision.calculate"
    bl_label = "Calculate Collisions"
    bl_options = {'REGISTER', 'UNDO'}
    
    # Modal properties
    _timer = None
    _is_initialized = False
    _is_finished = False
    
    # Calculation state tracking
    _rot_x_range = []
    _rot_y_range = []
    _rot_z_range = []
    _trans_x_range = []
    _trans_y_range = []
    _trans_z_range = []
    _csv_data = []
    _collision_data = []
    _total_iterations = 0
    _completed_iterations = 0
    _prox_bvh = None
    _orig_rot_loc = None
    _orig_rot_rotation = None
    _cur_x_idx = 0
    _cur_y_idx = 0
    _cur_z_idx = 0
    _cur_tx_idx = 0
    _cur_ty_idx = 0
    _cur_tz_idx = 0
    
    # Additional optimization variables
    _prox_mesh = None
    _dist_meshes = {}  # Cache for distal object transformed meshes
    _last_transform_key = None  # Track the last transformation to avoid redundant BVH creation
    
    def check_collision(self, prox_obj, dist_obj, prox_bvh=None, transform_matrix=None):
        """Use BVH trees to check for collision between two objects
        
        Args:
            prox_obj: The proximal (fixed) object
            dist_obj: The distal (moving) object
            prox_bvh: Optional pre-calculated BVH tree for the proximal object
            transform_matrix: The transformation matrix for the distal object
        """
        props = bpy.context.scene.collision_props
        
        # Create BVH tree for proximal object if not provided
        if prox_bvh is None:
            prox_bvh = self.create_bvh_tree_from_object(prox_obj)
        
        # Create a transform key for caching
        transform_key = None
        if transform_matrix is not None:
            # Create a hashable key from the matrix values
            transform_key = tuple(tuple(row) for row in transform_matrix)
        
        # Try to use cached BVH tree for distal object if available
        if transform_key and transform_key in self._dist_meshes:
            dist_bvh = self._dist_meshes[transform_key]
        else:
            # Create a new BVH tree for distal object
            if transform_matrix:
                dist_bvh = self.create_bvh_tree_with_transform(dist_obj, transform_matrix)
            else:
                dist_bvh = self.create_bvh_tree_from_object(dist_obj)
            
            # Cache this BVH tree
            if transform_key:
                # Limit cache size by removing oldest entries if too many
                if len(self._dist_meshes) > 50:  # Only keep the 50 most recent transformations
                    oldest_key = next(iter(self._dist_meshes))
                    del self._dist_meshes[oldest_key]
                self._dist_meshes[transform_key] = dist_bvh
        
        # Check for overlap between the two BVH trees
        # The overlap method returns a list of overlapping pairs
        overlap_pairs = prox_bvh.overlap(dist_bvh)
        
        # If any overlap is found, return True
        return len(overlap_pairs) > 0
    
    def create_bvh_tree_from_object(self, obj):
        """Create a BVH tree from an object's mesh data"""
        bm = bmesh.new()
        mesh = obj.to_mesh()
        bm.from_mesh(mesh)
        bm.transform(obj.matrix_world)
        bvh = mathutils.bvhtree.BVHTree.FromBMesh(bm)
        
        # Clean up
        bm.free()
        obj.to_mesh_clear()
        
        return bvh
    
    def create_bvh_tree_with_transform(self, obj, transform_matrix):
        """Create a BVH tree with a specific transformation matrix"""
        bm = bmesh.new()
        mesh = obj.to_mesh()
        bm.from_mesh(mesh)
        
        # Apply the transform directly to the BMesh vertices
        # This avoids the need for a scene update
        bm.transform(transform_matrix)
        
        # Create BVH tree
        bvh = mathutils.bvhtree.BVHTree.FromBMesh(bm)
        
        # Clean up
        bm.free()
        obj.to_mesh_clear()
        
        return bvh
    
    def initialize_calculation(self, context):
        """Set up calculation parameters and state"""
        props = context.scene.collision_props
        
        # Validate input
        if not props.proximal_object or not props.distal_object:
            self.report({'ERROR'}, "Both proximal and distal objects must be selected")
            return False
        
        # Get the objects
        prox_obj = props.proximal_object
        dist_obj = props.distal_object
        rot_obj = props.rotational_object if props.rotational_object else dist_obj
        
        # Store original transformations - this is our reference point
        self._orig_rot_loc = rot_obj.location.copy()
        self._orig_rot_rotation = rot_obj.rotation_euler.copy()
        
        self.report({'INFO'}, f"Starting from rotation: {[math.degrees(r) for r in self._orig_rot_rotation]}")
        self.report({'INFO'}, f"Starting from location: {self._orig_rot_loc}")
        
        # Create rotation range lists - these are RELATIVE to the current rotation
        self._rot_x_range = np.arange(props.rot_x_min, props.rot_x_max + props.rot_x_inc, props.rot_x_inc).tolist()
        self._rot_y_range = np.arange(props.rot_y_min, props.rot_y_max + props.rot_y_inc, props.rot_y_inc).tolist()
        self._rot_z_range = np.arange(props.rot_z_min, props.rot_z_max + props.rot_z_inc, props.rot_z_inc).tolist()
        
        # Create translation range lists - these are RELATIVE to the current location
        self._trans_x_range = np.arange(props.trans_x_min, props.trans_x_max + props.trans_x_inc, props.trans_x_inc).tolist()
        self._trans_y_range = np.arange(props.trans_y_min, props.trans_y_max + props.trans_y_inc, props.trans_y_inc).tolist()
        self._trans_z_range = np.arange(props.trans_z_min, props.trans_z_max + props.trans_z_inc, props.trans_z_inc).tolist()
        
        # Make sure we have at least one value in each range
        if len(self._rot_x_range) == 0: self._rot_x_range = [props.rot_x_min]
        if len(self._rot_y_range) == 0: self._rot_y_range = [props.rot_y_min]
        if len(self._rot_z_range) == 0: self._rot_z_range = [props.rot_z_min]
        if len(self._trans_x_range) == 0: self._trans_x_range = [props.trans_x_min]
        if len(self._trans_y_range) == 0: self._trans_y_range = [props.trans_y_min]
        if len(self._trans_z_range) == 0: self._trans_z_range = [props.trans_z_min]
        
        # Calculate total iterations for progress reporting
        self._total_iterations = len(self._rot_x_range) * len(self._rot_y_range) * len(self._rot_z_range) * \
                           len(self._trans_x_range) * len(self._trans_y_range) * len(self._trans_z_range)
        
        # Prepare CSV data
        self._csv_data = [["rot_x", "rot_y", "rot_z", "trans_x", "trans_y", "trans_z", "collision"]]
        
        # Prepare data for object attributes if enabled
        self._collision_data = []
        
        # Initialize progress counter
        self._completed_iterations = 0
        
        # Pre-calculate the BVH tree for the proximal object (which doesn't move)
        # This is a major optimization as we only need to calculate it once
        self.report({'INFO'}, "Pre-calculating BVH tree for proximal object...")
        self._prox_bvh = self.create_bvh_tree_from_object(props.proximal_object)
        
        # Reset the distal mesh cache
        self._dist_meshes = {}
        
        # Initialize index trackers for iteration
        self._cur_x_idx = 0
        self._cur_y_idx = 0
        self._cur_z_idx = 0
        self._cur_tx_idx = 0
        self._cur_ty_idx = 0
        self._cur_tz_idx = 0
        
        # Set initialization flag
        self._is_initialized = True
        self._is_finished = False
        
        # Add a counter for non-collision keyframes
        self._non_collision_frame = 1
        
        # Update UI to show progress
        props.calculation_progress = 0.0
        props.is_calculating = True
        
        return True
    
    def process_batch(self, context):
        """Process a batch of calculations"""
        props = context.scene.collision_props
        
        if not self._is_initialized or self._is_finished:
            return False
        
        # Use the configurable batch size
        batch_size = props.batch_size
        
        # Get the objects
        prox_obj = props.proximal_object
        dist_obj = props.distal_object
        rot_obj = props.rotational_object if props.rotational_object else dist_obj
        
        # Store original transformations for restoration at end if using scene updates
        orig_loc = rot_obj.location.copy()
        orig_rot = rot_obj.rotation_euler.copy()
        
        # Process a batch of iterations
        batch_counter = 0
        last_view_update = 0
        
        while batch_counter < batch_size:
            # Check if we need to stop
            if not props.is_calculating:
                self.report({'INFO'}, "Calculation cancelled by user")
                self.finalize_calculation(context, cancelled=True)
                return False
            
            # Get current indices and values
            if self._cur_x_idx >= len(self._rot_x_range):
                self._is_finished = True
                break
                
            rot_x = self._rot_x_range[self._cur_x_idx]
            rot_y = self._rot_y_range[self._cur_y_idx]
            rot_z = self._rot_z_range[self._cur_z_idx]
            trans_x = self._trans_x_range[self._cur_tx_idx]
            trans_y = self._trans_y_range[self._cur_ty_idx]
            trans_z = self._trans_z_range[self._cur_tz_idx]
            
            # Always use method 1: update the scene
            # Reset rotation object to original position
            rot_obj.location = self._orig_rot_loc.copy()
            rot_obj.rotation_euler = self._orig_rot_rotation.copy()
            
            # Apply rotation (converting degrees to radians)
            # The additional rotation is applied to the current rotation
            rot_euler = mathutils.Euler(
                (math.radians(rot_x), math.radians(rot_y), math.radians(rot_z)),
                props.rot_order  # use selected rotation order
            )
            
            # Apply rotation to the rotation object
            rot_obj.rotation_euler.rotate(rot_euler)
            
            # Apply translation to the rotation object
            rot_obj.location.x += trans_x
            rot_obj.location.y += trans_y
            rot_obj.location.z += trans_z
            
            # Update the scene (expensive operation)
            context.view_layer.update()
            
            # Check for collision using the pre-calculated proximal BVH tree
            collision = self.check_collision(prox_obj, dist_obj, self._prox_bvh)
            
            # Record data
            # Calculate absolute rotations for clarity
            absolute_rot_x = self._orig_rot_rotation.x + math.radians(rot_x)
            absolute_rot_y = self._orig_rot_rotation.y + math.radians(rot_y)
            absolute_rot_z = self._orig_rot_rotation.z + math.radians(rot_z)
            
            # Convert back to degrees for storage
            absolute_rot_x = math.degrees(absolute_rot_x)
            absolute_rot_y = math.degrees(absolute_rot_y)
            absolute_rot_z = math.degrees(absolute_rot_z)
            
            self._csv_data.append([rot_x, rot_y, rot_z, trans_x, trans_y, trans_z, 1 if collision else 0])
            
            # If pose is collision-free, insert keyframes immediately
            if not collision:
                if props.visualize_collisions:
                    rot_obj.keyframe_insert(data_path="location", frame=self._non_collision_frame)
                    rot_obj.keyframe_insert(data_path="rotation_euler", frame=self._non_collision_frame)
                self._non_collision_frame += 1
            
            # Increment indices using more efficient approach
            self._cur_tz_idx += 1
            if self._cur_tz_idx >= len(self._trans_z_range):
                self._cur_tz_idx = 0
                self._cur_ty_idx += 1
                if self._cur_ty_idx >= len(self._trans_y_range):
                    self._cur_ty_idx = 0
                    self._cur_tx_idx += 1
                    if self._cur_tx_idx >= len(self._trans_x_range):
                        self._cur_tx_idx = 0
                        self._cur_z_idx += 1
                        if self._cur_z_idx >= len(self._rot_z_range):
                            self._cur_z_idx = 0
                            self._cur_y_idx += 1
                            if self._cur_y_idx >= len(self._rot_y_range):
                                self._cur_y_idx = 0
                                self._cur_x_idx += 1
                                if self._cur_x_idx >= len(self._rot_x_range):
                                    self._is_finished = True
                                    break
            
            # Update progress counter
            self._completed_iterations += 1
            batch_counter += 1
            
            # Update progress in the UI 
            if self._total_iterations > 0:
                progress_pct = (self._completed_iterations / self._total_iterations) * 100
                props.calculation_progress = progress_pct
                
                # Periodically update view for better user feedback
                if (batch_counter % 25 == 0):
                    for area in context.screen.areas:
                        if area.type == 'VIEW_3D':
                            area.tag_redraw()
        
        # Always restore original position and update
        rot_obj.location = orig_loc
        rot_obj.rotation_euler = orig_rot
        context.view_layer.update()
        
        # Check if we're done
        if self._is_finished:
            self.finalize_calculation(context)
            return False
            
        return True
    
    def finalize_calculation(self, context, cancelled=False):
        """Complete the calculation and process results"""
        props = context.scene.collision_props
        
        # Reset rotation object to original position
        rot_obj = props.rotational_object if props.rotational_object else props.distal_object
        rot_obj.location = self._orig_rot_loc
        rot_obj.rotation_euler = self._orig_rot_rotation
        context.view_layer.update()
        
        # Clear the mesh caches to free memory
        self._dist_meshes = {}
        
        if not cancelled:
            # Export CSV
            if props.export_to_csv and props.export_path:
                filepath = bpy.path.abspath(props.export_path)
                dirpath = os.path.dirname(filepath)
                
                # Only create directories if there's actually a directory path
                if dirpath:
                    os.makedirs(dirpath, exist_ok=True)
                
                with open(filepath, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerows(self._csv_data)
                
                self.report({'INFO'}, f"Collision data exported to {filepath}")
            
            # Only store the collision data as an attribute if enabled in the UI
            if props.store_as_attributes and props.attribute_name_prefix:
                collision_data = self._csv_data[1:]
                self.store_collision_data_as_attributes(context, rot_obj, collision_data, props.attribute_name_prefix)
            

            # Removed separate keyframe operator call.
            self.report({'INFO'}, f"Inserted {self._non_collision_frame-1} keyframes on collision-free poses")
        
        # Reset calculation state
        props.is_calculating = False
        props.calculation_progress = 0.0
        self._is_initialized = False
        self._is_finished = True
        
        # Clear references to temporary objects
        self._prox_bvh = None
        self._orig_rot_loc = None
        self._orig_rot_rotation = None
        
        return
    
    def modal(self, context, event):
        """Modal function called during calculation"""
        props = context.scene.collision_props
        
        # Check if calculation is still active
        if not props.is_calculating:
            self.cancel(context)
            return {'CANCELLED'}
            
        # Check for timer event to process next batch
        if event.type == 'TIMER':
            # Process a batch of calculations
            if not self.process_batch(context):
                # If process_batch returns False, we're done or cancelled
                self.cancel(context)
                return {'FINISHED'}
            
            # Force UI redraw to update progress bar
            for area in context.screen.areas:
                if area.type == 'VIEW_3D':
                    area.tag_redraw()
        
        return {'PASS_THROUGH'}
    
    def execute(self, context):
        """Start the calculation process"""
        props = context.scene.collision_props
        
        # Check if a calculation is already in progress
        if props.is_calculating:
            self.report({'WARNING'}, "A calculation is already in progress")
            return {'CANCELLED'}
            
        # Initialize the calculation
        if not self.initialize_calculation(context):
            return {'CANCELLED'}
            
        # Set up the modal timer
        wm = context.window_manager
        self._timer = wm.event_timer_add(0.1, window=context.window)
        wm.modal_handler_add(self)
        
        return {'RUNNING_MODAL'}
    
    def cancel(self, context):
        """Clean up the modal operator"""
        wm = context.window_manager
        if self._timer:
            wm.event_timer_remove(self._timer)
            self._timer = None
            
        # If we were cancelled in the middle of a calculation,
        # call finalize to clean up
        if self._is_initialized and not self._is_finished:
            self.finalize_calculation(context, cancelled=True)
    
    def store_collision_data_as_attributes(self, context, obj, collision_data, prefix):
        """Store collision data as compact JSON in a single attribute"""
        # First, clear any existing collision attributes
        for attr_name in list(obj.keys()):
            if attr_name.startswith(prefix):
                del obj[attr_name]
        
        # Store the collision count
        obj[f"{prefix}count"] = len(collision_data)
        
        # Prepare a compact representation of collision data.
        # Each row is expected to be: [rot_x, rot_y, rot_z, trans_x, trans_y, trans_z, collision]
        # Here we store only the pose data (first six values). Adjust if needed.
        compact_data = [ row[:6] for row in collision_data ]
        
        
        # Store as JSON string in a single attribute
        obj[f"{prefix}data"] = json.dumps(compact_data)
        
        self.report({'INFO'}, f"Stored collision data in compact format")
    
    # Commented out unused function: get_collision_data_from_attributes
    '''
    def get_collision_data_from_attributes(self, obj, prefix):
        """Retrieve collision data from compact JSON attribute"""
        if f"{prefix}data" not in obj:
            return []
        
        # Parse the JSON data
        compact_data = json.loads(obj[f"{prefix}data"])
        
        # Convert back to dictionary format
        collision_data = []
        for item in compact_data:
            collision_data.append({
                'rot_x': item[0],
                'rot_y': item[1],
                'rot_z': item[2],
                'trans_x': item[3],
                'trans_y': item[4],
                'trans_z': item[5]
            })
        
        return collision_data
    '''

    # Commented out unused function: visualize_collision_data
    '''
    def visualize_collision_data(self, context, obj, collision_data, prefix):
        """Create an NLA strip for positions without collisions"""
        self.report({'INFO'}, f"Creating animation layer for non-collision poses")
        
        # Get the rotational object
        rot_obj = context.scene.collision_props.rotational_object if context.scene.collision_props.rotational_object else obj
        
        # Store original transformations
        orig_rot_loc = rot_obj.location.copy()
        orig_rot_rotation = rot_obj.rotation_euler.copy()
        
        # Make sure we have animation data
        if not rot_obj.animation_data:
            rot_obj.animation_data_create()
        
        # Store original action if there is one
        original_action = rot_obj.animation_data.action
        
        # Create a new action with a consistent name
        action_name = "ROM_Safe_Poses"
        
        # Check if the action already exists and use it if it does
        if action_name in bpy.data.actions:
            new_action = bpy.data.actions[action_name]
            # Clear existing keyframes
            for fc in new_action.fcurves:
                new_action.fcurves.remove(fc)
        else:
            new_action = bpy.data.actions.new(name=action_name)
        
        # Set this as the active action while we create keyframes
        rot_obj.animation_data.action = new_action
        
        # First, keyframe the original pose at frame 0
        rot_obj.keyframe_insert(data_path="location", frame=0)
        rot_obj.keyframe_insert(data_path="rotation_euler", frame=0)
        
        # Get all poses from our calculation
        props = context.scene.collision_props
        
        # Create rotation range lists
        rot_x_range = np.arange(props.rot_x_min, props.rot_x_max + props.rot_x_inc, props.rot_x_inc)
        rot_y_range = np.arange(props.rot_y_min, props.rot_y_max + props.rot_y_inc, props.rot_y_inc)
        rot_z_range = np.arange(props.rot_z_min, props.rot_z_max + props.rot_z_inc, props.rot_z_inc)
        
        # Create translation range lists
        trans_x_range = np.arange(props.trans_x_min, props.trans_x_max + props.trans_x_inc, props.trans_x_inc)
        trans_y_range = np.arange(props.trans_y_min, props.trans_y_max + props.trans_y_inc, props.trans_y_inc)
        trans_z_range = np.arange(props.trans_z_min, props.trans_z_max + props.trans_z_inc, props.trans_z_inc)
        
        # Make sure we have at least one value in each range
        if len(rot_x_range) == 0: rot_x_range = [props.rot_x_min]
        if len(rot_y_range) == 0: rot_y_range = [props.rot_y_min]
        if len(rot_z_range) == 0: rot_z_range = [props.rot_z_min]
        if len(trans_x_range) == 0: trans_x_range = [props.trans_x_min]
        if len(trans_y_range) == 0: trans_y_range = [props.trans_y_min]
        if len(trans_z_range) == 0: trans_z_range = [props.trans_z_min]
        
        # Create a dictionary of collision poses for quick lookup
        collision_poses = {}
        for data in collision_data:
            key = (data['rot_x'], data['rot_y'], data['rot_z'], 
                   data['trans_x'], data['trans_y'], data['trans_z'])
            collision_poses[key] = True
        
        # Create keyframes for non-collision poses
        # Start from frame 1 since frame 0 is our original pose
        frame = 1
        frame_data = []
        
        # Start creating keyframes
        for rot_x in rot_x_range:
            for rot_y in rot_y_range:
                for rot_z in rot_z_range:
                    for trans_x in trans_x_range:
                        for trans_y in trans_y_range:
                            for trans_z in trans_z_range:
                                key = (rot_x, rot_y, rot_z, trans_x, trans_y, trans_z)
                                
                                # If this pose has no collision, add a keyframe
                                if key not in collision_poses:
                                    frame_data.append({
                                        'frame': frame,
                                        'rot_x': rot_x,
                                        'rot_y': rot_y,
                                        'rot_z': rot_z,
                                        'trans_x': trans_x,
                                        'trans_y': trans_y,
                                        'trans_z': trans_z
                                    })
                                    frame += 1
        
        # Now apply all the keyframes
        for i, data in enumerate(frame_data):
            if i % 100 == 0:
                self.report({'INFO'}, f"Creating keyframe {i}/{len(frame_data)}")
            
            # Reset object to original position
            rot_obj.location = orig_rot_loc.copy()
            rot_obj.rotation_euler = orig_rot_rotation.copy()
            
            # Apply rotation (converting degrees to radians)
            rot_euler = mathutils.Euler(
                (math.radians(data['rot_x']), math.radians(data['rot_y']), math.radians(data['rot_z'])), 
                'XYZ'
            )
            
            # Apply rotation to the rotation object
            rot_obj.rotation_euler.rotate(rot_euler)
            
            # Apply translation to the rotation object
            rot_obj.location.x += data['trans_x']
            rot_obj.location.y += data['trans_y']
            rot_obj.location.z += data['trans_z']
            
            # Insert keyframe
            rot_obj.keyframe_insert(data_path="location", frame=data['frame'])
            rot_obj.keyframe_insert(data_path="rotation_euler", frame=data['frame'])
        
        # Add the action as an NLA strip
        if new_action.users > 0:  # Only add if we created keyframes
            # Check if a "ROM Safe Poses" track already exists, if yes, remove it
            existing_track = rot_obj.animation_data.nla_tracks.get("ROM_Safe_Poses")
            if existing_track:
                rot_obj.animation_data.nla_tracks.remove(existing_track)
                
            # Create a new track with the same name as the action
            track = rot_obj.animation_data.nla_tracks.new()
            track.name = "ROM_Safe_Poses"
            
            # Create strip with the same name
            strip = track.strips.new(name="ROM_Safe_Poses", start=0, action=new_action)
            
            # Configure the strip
            strip.blend_type = 'REPLACE'
            strip.use_auto_blend = False
            strip.extrapolation = 'HOLD'  # Hold the last frame's pose
            
            # Restore or create the original animation track if there was one
            if original_action:
                # Check if an "Original Animation" track already exists
                orig_track = rot_obj.animation_data.nla_tracks.get("Original_Animation")
                if not orig_track:
                    # Create a new track for the original animation
                    orig_track = rot_obj.animation_data.nla_tracks.new()
                    orig_track.name = "Original_Animation"
                    # Add the original action as a strip
                    orig_strip = orig_track.strips.new(name="Original_Animation", start=0, action=original_action)
                    orig_strip.blend_type = 'REPLACE'
                    orig_strip.extrapolation = 'HOLD'
                
                # Adjust which track is active based on checkbox
                if props.visualize_collisions:
                    # Show ROM Safe Poses, hide Original Animation
                    track.mute = False
                    strip.influence = 1.0
                    orig_track.mute = True
                    
                    # Set as active action (visible in animation panel)
                    rot_obj.animation_data.action = new_action
                else:
                    # Hide ROM Safe Poses, show Original Animation
                    track.mute = True
                    strip.influence = 0.0
                    orig_track.mute = False
                    
                    # Set original as active action
                    rot_obj.animation_data.action = original_action
            else:
                # If there was no original animation
                if props.visualize_collisions:
                    # Show ROM animation
                    track.mute = False
                    strip.influence = 1.0
                    rot_obj.animation_data.action = new_action
                else:
                    # Hide ROM animation
                    track.mute = True
                    strip.influence = 0.0
                    rot_obj.animation_data.action = None
            
            # Make sure the effect of the active strip is applied
            if props.visualize_collisions:
                # Set the current frame to 0 to see the original pose initially
                context.scene.frame_set(0)
                # Enable NLA evaluation
                rot_obj.animation_data.use_nla = True
                # Force an update
                rot_obj.update_tag()
                context.view_layer.update()
            
            self.report({'INFO'}, f"Created NLA strip with {len(frame_data)} non-collision poses")
        else:
            # If we didn't create any keyframes (all poses have collisions)
            self.report({'WARNING'}, "No collision-free poses found to create animation")
            
            # Restore original action if there was one
            rot_obj.animation_data.action = original_action
        
        # Reset object to original position
        rot_obj.location = orig_rot_loc
        rot_obj.rotation_euler = orig_rot_rotation
    '''

    # Commented out unused function: make_collection_visible
    '''
    def make_collection_visible(self, layer_collection, target_name):
        """Recursively make a collection visible"""
        if layer_collection.name == target_name:
            layer_collection.hide_viewport = False
            return True
        
        for child in layer_collection.children:
            if self.make_collection_visible(child, target_name):
                layer_collection.hide_viewport = False
                return True
        
        return False
    '''
    
# ...existing code...