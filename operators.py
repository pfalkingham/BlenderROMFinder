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
    _orig_bone_matrix_local = None
    
    # Cached proximal hull object and its BVH tree
    _prox_hull_obj = None 
    _prox_hull_bvh = None
    
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
    _start_time = None
    
    def create_bvh_tree(self, obj, transform_matrix=None):
        """Create a BVH tree from an object's mesh data, with optional transformation."""
        bm = bmesh.new()
        mesh = obj.to_mesh()
        bm.from_mesh(mesh)
        if transform_matrix is not None:
            bm.transform(transform_matrix)
        else:
            bm.transform(obj.matrix_world)
        bvh = mathutils.bvhtree.BVHTree.FromBMesh(bm)
        bm.free()
        obj.to_mesh_clear()
        return bvh

    def create_convex_hull_object(self, obj):
        """Create a temporary convex hull mesh object from the given object."""
        # Duplicate the object and enter edit mode
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.duplicate()
        hull_obj = bpy.context.active_object
        # Enter edit mode and create convex hull
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.convex_hull()
        bpy.ops.object.mode_set(mode='OBJECT')
        return hull_obj

    def remove_temp_object(self, obj):
        bpy.data.objects.remove(obj, do_unlink=True)

    def restore_object_transform(self, obj, location, rotation):
        """Restore the object's location and rotation."""
        obj.location = location.copy()
        obj.rotation_euler = rotation.copy()

    def check_collision(self, prox_obj, dist_obj, prox_bvh_original_mesh=None, transform_matrix=None):
        """Use BVH trees to check for collision between two objects, with optional convex hull optimization."""
        props = bpy.context.scene.collision_props # Get access to properties

        if props.use_convex_hull_optimization:
            # 1. Proximal Hull (cached Blender object and its BVH)
            if self._prox_hull_obj is None:
                self._prox_hull_obj = self.create_convex_hull_object(prox_obj)
            
            if self._prox_hull_bvh is None:
                if self._prox_hull_obj:
                    self._prox_hull_bvh = self.create_bvh_tree(self._prox_hull_obj)
                else:
                    # Fall through to original mesh check if hull creation fails
                    pass 

            if not self._prox_hull_bvh:
                # Fall through to original mesh check
                pass 
            else:
                # 2. Distal Hull (temporary Blender object and its BVH for current pose)
                temp_dist_hull_obj = self.create_convex_hull_object(dist_obj)
                if not temp_dist_hull_obj:
                    # Fall through to original mesh check
                    pass 
                else:
                    current_dist_hull_bvh = self.create_bvh_tree(temp_dist_hull_obj)
                    
                    if temp_dist_hull_obj.name in bpy.data.objects:
                        self.remove_temp_object(temp_dist_hull_obj)
                    temp_dist_hull_obj = None

                    if not current_dist_hull_bvh:
                        # Fall through to original mesh check
                        pass 
                    else:
                        # 3. Hull Check
                        hull_overlap_pairs = self._prox_hull_bvh.overlap(current_dist_hull_bvh)
                        if not hull_overlap_pairs:
                            return False # No collision if hulls don't overlap (potential containment missed)
        
        # 4. Original Mesh Check (if hulls overlapped, hull check was skipped, or optimization is off)
        dist_bvh_original_mesh = None
        transform_key = None
        if transform_matrix is not None:
            transform_key = tuple(tuple(row) for row in transform_matrix)
        
        if transform_key and transform_key in self._dist_meshes:
            dist_bvh_original_mesh = self._dist_meshes[transform_key]
        else:
            dist_bvh_original_mesh = self.create_bvh_tree(dist_obj, transform_matrix)
            if transform_key and dist_bvh_original_mesh:
                if len(self._dist_meshes) > 50: # Cache limit
                    oldest_key = next(iter(self._dist_meshes))
                    del self._dist_meshes[oldest_key]
                self._dist_meshes[transform_key] = dist_bvh_original_mesh
        
        if not prox_bvh_original_mesh or not dist_bvh_original_mesh:
            self.report({'ERROR'}, "BVH for original mesh check not available.")
            return True # Fail safe

        original_mesh_overlap_pairs = prox_bvh_original_mesh.overlap(dist_bvh_original_mesh)
        
        return len(original_mesh_overlap_pairs) > 0

    def get_bone_world_matrix(self, armature_obj, bone_name):
        # Get the world matrix of the specified bone in the armature
        if armature_obj and armature_obj.type == 'ARMATURE' and bone_name:
            # Use pose bone for current transform
            pose_bone = armature_obj.pose.bones.get(bone_name)
            if pose_bone:
                return armature_obj.matrix_world @ pose_bone.matrix
        return None

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
        use_bone = False
        bone_matrix = None
        bone_name = getattr(props, 'rotational_bone', None)
        if rot_obj and rot_obj.type == 'ARMATURE' and bone_name:
            bone_matrix = self.get_bone_world_matrix(rot_obj, bone_name)
            use_bone = bone_matrix is not None
        
        # Store original transformations - this is our reference point
        if use_bone:
            self._orig_bone_matrix = bone_matrix.copy()
            self._orig_rot_loc = None
            self._orig_rot_rotation = None
            self.report({'INFO'}, f"Using bone '{bone_name}' as pivot.")
        else:
            self._orig_rot_loc = rot_obj.location.copy()
            self._orig_rot_rotation = rot_obj.rotation_euler.copy()
            self.report({'INFO'}, f"Starting from rotation: {[math.degrees(r) for r in self._orig_rot_rotation]}")
            self.report({'INFO'}, f"Starting from location: {self._orig_rot_loc}")
        
        # Keyframe the starting position at frame 0
        if use_bone:
            pose_bone = rot_obj.pose.bones.get(bone_name)
            if pose_bone:
                pose_bone.keyframe_insert(data_path="location", frame=0)
                pose_bone.keyframe_insert(data_path="rotation_euler", frame=0)
        else:
            rot_obj.keyframe_insert(data_path="location", frame=0)
            rot_obj.keyframe_insert(data_path="rotation_euler", frame=0)
        
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
        self._csv_data = [["rot_x", "rot_y", "rot_z", "trans_x", "trans_y", "trans_z", "Valid_pose"]]
        
        # Prepare data for object attributes if enabled
        self._collision_data = []
        
        # Initialize progress counter
        self._completed_iterations = 0
        
        # Pre-calculate the BVH tree for the proximal object (which doesn't move)
        # This is a major optimization as we only need to calculate it once
        self.report({'INFO'}, "Pre-calculating BVH tree for proximal object...")
        self._prox_bvh = self.create_bvh_tree(props.proximal_object)
        
        # Ensure cached hull attributes are reset for a new calculation run
        if hasattr(self, '_prox_hull_obj') and self._prox_hull_obj:
            if self._prox_hull_obj.name in bpy.data.objects: # Check if it still exists
                self.remove_temp_object(self._prox_hull_obj)
        self._prox_hull_obj = None
        self._prox_hull_bvh = None # BVH trees are just Python objects, no Blender data to remove directly
        
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
        
        self._start_time = time.time()
        
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
        bone_name = getattr(props, 'rotational_bone', None)
        use_bone = rot_obj and rot_obj.type == 'ARMATURE' and bone_name
        
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
            
            if use_bone:
                # Reset bone to original matrix (approximate, as Blender doesn't allow direct set)
                pose_bone = rot_obj.pose.bones.get(bone_name)
                if pose_bone:
                    # Calculate relative rotation
                    rot_euler = mathutils.Euler((math.radians(rot_x), math.radians(rot_y), math.radians(rot_z)), props.rot_order)
                    pose_bone.rotation_mode = props.rot_order
                    pose_bone.rotation_euler = rot_euler
                    # Optionally, apply translation to bone head (not common in Blender, but can be done via location)
                    pose_bone.location = mathutils.Vector((trans_x, trans_y, trans_z))
                    context.view_layer.update()
            else:
                # Always use method 1: update the scene
                # Reset rotation object to original position - Location is still relative
                rot_obj.location = self._orig_rot_loc + mathutils.Vector((trans_x, trans_y, trans_z))

                # Calculate the target absolute rotation
                # Start from the original rotation
                target_rotation = self._orig_rot_rotation.copy()
                # Create the delta rotation Euler
                delta_rot_euler = mathutils.Euler(
                    (math.radians(rot_x), math.radians(rot_y), math.radians(rot_z)),
                    props.rot_order  # use selected rotation order
                )
                # Apply the delta rotation to the original rotation
                target_rotation.rotate(delta_rot_euler)

                # Set the object's rotation directly to the target absolute rotation
                rot_obj.rotation_euler = target_rotation

                # Update the scene (expensive operation)
                context.view_layer.update()
            
            # Check for collision using the pre-calculated proximal BVH tree
            collision = self.check_collision(prox_obj, dist_obj, self._prox_bvh)
            
            # Record data
            # Calculate absolute rotations for clarity
            if self._orig_rot_rotation is not None:
                absolute_rot_x = self._orig_rot_rotation.x + math.radians(rot_x)
                absolute_rot_y = self._orig_rot_rotation.y + math.radians(rot_y)
                absolute_rot_z = self._orig_rot_rotation.z + math.radians(rot_z)
                # Convert back to degrees for storage
                absolute_rot_x = math.degrees(absolute_rot_x)
                absolute_rot_y = math.degrees(absolute_rot_y)
                absolute_rot_z = math.degrees(absolute_rot_z)
            else:
                absolute_rot_x = None
                absolute_rot_y = None
                absolute_rot_z = None
            
            self._csv_data.append([rot_x, rot_y, rot_z, trans_x, trans_y, trans_z, 0 if collision else 1]) #Changed so that 0 is collision and 1 is valid pose.
            
            # If pose is collision-free, insert keyframes immediately
            if not collision:
                if props.visualize_collisions:
                    if use_bone and pose_bone:
                        pose_bone.keyframe_insert(data_path="location", frame=self._non_collision_frame)
                        pose_bone.keyframe_insert(data_path="rotation_euler", frame=self._non_collision_frame)
                    else:
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
                # Time remaining estimate
                if self._completed_iterations > 0 and self._start_time:
                    elapsed = time.time() - self._start_time
                    rate = self._completed_iterations / elapsed
                    remaining = (self._total_iterations - self._completed_iterations) / rate if rate > 0 else 0
                    mins, secs = divmod(int(remaining), 60)
                    props.time_remaining = f"Time remaining: {mins:02d}:{secs:02d}"
                else:
                    props.time_remaining = "Calculating..."
                
                # Periodically update view for better user feedback
                if (batch_counter % 25 == 0):
                    for area in context.screen.areas:
                        if area.type == 'VIEW_3D':
                            area.tag_redraw()
        
        # Always restore original position and update
        if use_bone and pose_bone:
            # Reset bone transform
            pose_bone.location = mathutils.Vector((0,0,0))
            pose_bone.rotation_euler = mathutils.Euler((0,0,0), props.rot_order)
            context.view_layer.update()
        else:
            self.restore_object_transform(rot_obj, orig_loc, orig_rot)
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
        bone_name = getattr(props, 'rotational_bone', None)
        use_bone = rot_obj and rot_obj.type == 'ARMATURE' and bone_name
        if use_bone:
            pose_bone = rot_obj.pose.bones.get(bone_name)
            if pose_bone:
                pose_bone.location = mathutils.Vector((0,0,0))
                pose_bone.rotation_euler = mathutils.Euler((0,0,0), props.rot_order)
                context.view_layer.update()
        elif self._orig_rot_loc is not None and self._orig_rot_rotation is not None:
            self.restore_object_transform(rot_obj, self._orig_rot_loc, self._orig_rot_rotation)
            context.view_layer.update()
        
        # Clear the mesh caches to free memory
        self._dist_meshes = {}
        
        # Clean up the cached proximal hull object and its BVH
        if hasattr(self, '_prox_hull_obj') and self._prox_hull_obj:
            if self._prox_hull_obj.name in bpy.data.objects: # Check if it still exists
                self.remove_temp_object(self._prox_hull_obj)
            self._prox_hull_obj = None
        # BVH trees are Python objects; they are garbage collected. No specific Blender data to remove for _prox_hull_bvh itself.
        self._prox_hull_bvh = None 

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
        
        # Output total time taken
        if self._start_time:
            elapsed = time.time() - self._start_time
            mins, secs = divmod(int(elapsed), 60)
            self.report({'INFO'}, f"Total time taken: {mins:02d}:{secs:02d}")
        
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
            
        # Ensure we are in Object Mode before starting calculations
        if bpy.ops.object.mode_set.poll():
            bpy.ops.object.mode_set(mode='OBJECT')
            
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

