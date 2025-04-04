import bpy
import bmesh
import mathutils
import math
import numpy as np
import csv
import os
import json
from bpy.types import Operator

class COLLISION_OT_calculate(Operator):
    """Calculate object collisions based on rotations and translations"""
    bl_idname = "collision.calculate"
    bl_label = "Calculate Collisions"
    bl_options = {'REGISTER', 'UNDO'}
    
    def check_collision(self, prox_obj, dist_obj, prox_bvh=None):
        """Use BVH trees to check for collision between two objects
        
        Args:
            prox_obj: The proximal (fixed) object
            dist_obj: The distal (moving) object
            prox_bvh: Optional pre-calculated BVH tree for the proximal object
        """
        # Create BVH tree for proximal object if not provided
        if prox_bvh is None:
            prox_bvh = self.create_bvh_tree_from_object(prox_obj)
            
        # Create BVH tree for distal object
        dist_bvh = self.create_bvh_tree_from_object(dist_obj)
        
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
    
    def execute(self, context):
        props = context.scene.collision_props
        
        # Validate input
        if not props.proximal_object or not props.distal_object:
            self.report({'ERROR'}, "Both proximal and distal objects must be selected")
            return {'CANCELLED'}
        
        # Get the objects
        prox_obj = props.proximal_object
        dist_obj = props.distal_object
        rot_obj = props.rotational_object if props.rotational_object else dist_obj
        
        # Store original transformations - this is our reference point
        orig_rot_loc = rot_obj.location.copy()
        orig_rot_rotation = rot_obj.rotation_euler.copy()
        
        self.report({'INFO'}, f"Starting from rotation: {[math.degrees(r) for r in orig_rot_rotation]}")
        self.report({'INFO'}, f"Starting from location: {orig_rot_loc}")
        
        # Create rotation range lists - these are RELATIVE to the current rotation
        rot_x_range = np.arange(props.rot_x_min, props.rot_x_max + props.rot_x_inc, props.rot_x_inc)
        rot_y_range = np.arange(props.rot_y_min, props.rot_y_max + props.rot_y_inc, props.rot_y_inc)
        rot_z_range = np.arange(props.rot_z_min, props.rot_z_max + props.rot_z_inc, props.rot_z_inc)
        
        # Create translation range lists - these are RELATIVE to the current location
        trans_x_range = np.arange(props.trans_x_min, props.trans_x_max + props.trans_x_inc, props.trans_x_inc)
        trans_y_range = np.arange(props.trans_y_min, props.trans_y_max + props.trans_y_inc, props.trans_y_inc)
        trans_z_range = np.arange(props.trans_x_min, props.trans_x_max + props.trans_x_inc, props.trans_x_inc)
        
        # Make sure we have at least one value in each range
        if len(rot_x_range) == 0: rot_x_range = [props.rot_x_min]
        if len(rot_y_range) == 0: rot_y_range = [props.rot_y_min]
        if len(rot_z_range) == 0: rot_z_range = [props.rot_z_min]
        if len(trans_x_range) == 0: trans_x_range = [props.trans_x_min]
        if len(trans_y_range) == 0: trans_y_range = [props.trans_y_min]
        if len(trans_z_range) == 0: trans_z_range = [props.trans_z_min]
        
        # Calculate total iterations for progress reporting
        total_iterations = len(rot_x_range) * len(rot_y_range) * len(rot_z_range) * \
                           len(trans_x_range) * len(trans_y_range) * len(trans_z_range)
        
        # Prepare CSV data
        csv_data = [["rot_x", "rot_y", "rot_z", "trans_x", "trans_y", "trans_z", "collision"]]
        
        # Prepare data for object attributes if enabled
        collision_data = []
        
        # Initialize progress counter
        current_iter = 0
        
        # Use rot_obj's location as the pivot point
        pivot = rot_obj.location.copy()
        
        # Pre-calculate the BVH tree for the proximal object (which doesn't move)
        # This is a major optimization as we only need to calculate it once
        self.report({'INFO'}, "Pre-calculating BVH tree for proximal object...")
        prox_bvh = self.create_bvh_tree_from_object(prox_obj)
        
        # Start calculation
        for rot_x in rot_x_range:
            for rot_y in rot_y_range:
                for rot_z in rot_z_range:
                    for trans_x in trans_x_range:
                        for trans_y in trans_y_range:
                            for trans_z in trans_z_range:
                                # Update progress
                                current_iter += 1
                                if current_iter % 100 == 0:  # Update every 100 iterations
                                    self.report({'INFO'}, f"Processing {current_iter}/{total_iterations} iterations")
                                
                                # Reset rotation object to original position
                                rot_obj.location = orig_rot_loc.copy()
                                rot_obj.rotation_euler = orig_rot_rotation.copy()
                                
                                # Apply rotation (converting degrees to radians)
                                # The additional rotation is applied to the current rotation
                                rot_euler = mathutils.Euler(
                                    (math.radians(rot_x), math.radians(rot_y), math.radians(rot_z)), 
                                    'XYZ'
                                )
                                
                                # Apply rotation to the rotation object
                                rot_obj.rotation_euler.rotate(rot_euler)
                                
                                # Apply translation to the rotation object
                                rot_obj.location.x += trans_x
                                rot_obj.location.y += trans_y
                                rot_obj.location.z += trans_z
                                
                                # Update the scene
                                context.view_layer.update()
                                
                                # Check for collision using the pre-calculated proximal BVH tree
                                collision = self.check_collision(prox_obj, dist_obj, prox_bvh)
                                
                                # Record data - store the absolute world rotations for clarity
                                absolute_rot_x = math.degrees(rot_obj.rotation_euler.x)
                                absolute_rot_y = math.degrees(rot_obj.rotation_euler.y)
                                absolute_rot_z = math.degrees(rot_obj.rotation_euler.z)
                                
                                csv_data.append([rot_x, rot_y, rot_z, trans_x, trans_y, trans_z, 1 if collision else 0])
                                
                                # If storing as attributes, save data
                                if props.store_as_attributes and collision:
                                    collision_data.append({
                                        'rot_x': rot_x,
                                        'rot_y': rot_y,
                                        'rot_z': rot_z,
                                        'trans_x': trans_x,
                                        'trans_y': trans_y,
                                        'trans_z': trans_z,
                                        'absolute_rot_x': absolute_rot_x,
                                        'absolute_rot_y': absolute_rot_y,
                                        'absolute_rot_z': absolute_rot_z
                                    })
        
        # Reset rotation object to original position
        rot_obj.location = orig_rot_loc
        rot_obj.rotation_euler = orig_rot_rotation
        context.view_layer.update()
        
        # Export CSV
        if props.export_to_csv and props.export_path:
            filepath = bpy.path.abspath(props.export_path)
            dirpath = os.path.dirname(filepath)
            
            # Only create directories if there's actually a directory path
            if dirpath:
                os.makedirs(dirpath, exist_ok=True)
            
            with open(filepath, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerows(csv_data)
            
            self.report({'INFO'}, f"Collision data exported to {filepath}")
        
        # Store data as attributes if enabled
        if props.store_as_attributes and collision_data:
            self.store_collision_data_as_attributes(context, dist_obj, collision_data, props.attribute_name_prefix)
            self.report({'INFO'}, f"Stored {len(collision_data)} collision points as attributes")
            
            # Always create visualization, but control visibility with the checkbox
            self.visualize_collision_data(context, dist_obj, collision_data, props.attribute_name_prefix)
            show_status = "shown" if props.visualize_collisions else "created but hidden"
            self.report({'INFO'}, f"Animation layer of non-collision poses {show_status}")
        
        return {'FINISHED'}
    
    def store_collision_data_as_attributes(self, context, obj, collision_data, prefix):
        """Store collision data as compact JSON in a single attribute"""
        # First, clear any existing collision attributes
        for attr_name in list(obj.keys()):
            if attr_name.startswith(prefix):
                del obj[attr_name]
        
        # Store the collision count
        obj[f"{prefix}count"] = len(collision_data)
        
        # Prepare a compact representation of collision data
        compact_data = []
        for data in collision_data:
            compact_data.append([
                data['rot_x'], data['rot_y'], data['rot_z'],
                data['trans_x'], data['trans_y'], data['trans_z']
            ])
        
        # Store as JSON string in a single attribute
        obj[f"{prefix}data"] = json.dumps(compact_data)
        
        self.report({'INFO'}, f"Stored collision data in compact format")
    
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
                    
                    # Move ROM track to the top to ensure it's applied
                    while rot_obj.animation_data.nla_tracks.find(track.name) > 0:
                        bpy.ops.anim.nla_track_move_up({"object": rot_obj}, track_index=rot_obj.animation_data.nla_tracks.find(track.name))
                else:
                    # Hide ROM Safe Poses, show Original Animation
                    track.mute = True
                    strip.influence = 0.0
                    orig_track.mute = False
                    
                    # Set original as active action
                    rot_obj.animation_data.action = original_action
                    
                    # Move Original track to the top to ensure it's applied
                    while rot_obj.animation_data.nla_tracks.find(orig_track.name) > 0:
                        bpy.ops.anim.nla_track_move_up({"object": rot_obj}, track_index=rot_obj.animation_data.nla_tracks.find(orig_track.name))
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