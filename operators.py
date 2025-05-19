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
        # BVH for distal object is created directly as poses are unique (or caching is removed)
        dist_bvh_original_mesh = self.create_bvh_tree(dist_obj, transform_matrix)
        
        if not prox_bvh_original_mesh or not dist_bvh_original_mesh:
            self.report({'ERROR'}, "BVH for original mesh check not available. Proximal or Distal BVH failed.")
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

    def calculate_acs_rotation(self, context, ACSf_obj, ACSm_obj, rot_x, rot_y, rot_z):
        """Calculate proper anatomical rotations based on ACSf and ACSm coordinate systems
        
        Parameters:
        - ACSf_obj: The fixed anatomical coordinate system object
        - ACSm_obj: The mobile anatomical coordinate system object
        - rot_x: X rotation in degrees (long axis rotation)
        - rot_y: Y rotation in degrees (adduction/abduction)
        - rot_z: Z rotation in degrees (flexion/extension)
        
        Returns:
        - Rotation matrix to apply to ACSm object
        """
        # Get the Z axis from ACSf (for flexion/extension)
        # In Blender's local space, Z axis is (0,0,1) rotated by the object's rotation
        z_axis = ACSf_obj.matrix_world.to_quaternion() @ mathutils.Vector((0, 0, 1))
        z_axis.normalize()
        
        # Get the X axis from ACSm (for long axis rotation)
        x_axis = ACSm_obj.matrix_world.to_quaternion() @ mathutils.Vector((1, 0, 0))
        x_axis.normalize()
        
        # Calculate Y axis as the cross product of Z and X
        # This gives us the adduction/abduction axis
        y_axis = z_axis.cross(x_axis)
        y_axis.normalize()
        
        # Recalculate X to ensure orthogonality (X = Y × Z)
        x_axis = y_axis.cross(z_axis)
        x_axis.normalize()
        
        # Create rotation matrices for each axis
        rot_matrix_x = mathutils.Matrix.Rotation(math.radians(rot_x), 4, x_axis)
        rot_matrix_y = mathutils.Matrix.Rotation(math.radians(rot_y), 4, y_axis)
        rot_matrix_z = mathutils.Matrix.Rotation(math.radians(rot_z), 4, z_axis)
        
        # Combine rotations according to specified order
        # For anatomical movement, typically Z is applied first (flexion/extension)
        # Then Y (adduction/abduction), and finally X (long axis rotation)
        final_rotation = rot_matrix_x @ rot_matrix_y @ rot_matrix_z
        
        return final_rotation

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
        ACSf_obj = props.ACSf_object
        ACSm_obj = props.ACSm_object
        
        if not ACSf_obj or not ACSm_obj:
            self.report({'ERROR'}, "Both ACSf and ACSm objects must be selected")
            return False

        # Check for armature bone usage in ACSf
        use_ACSf_bone = False
        ACSf_bone_matrix = None
        ACSf_bone_name = getattr(props, 'ACSf_bone', None)
        if ACSf_obj and ACSf_obj.type == 'ARMATURE' and ACSf_bone_name:
            ACSf_bone_matrix = self.get_bone_world_matrix(ACSf_obj, ACSf_bone_name)
            use_ACSf_bone = ACSf_bone_matrix is not None
            
        # Check for armature bone usage in ACSm
        use_ACSm_bone = False
        ACSm_bone_matrix = None
        ACSm_bone_name = getattr(props, 'ACSm_bone', None)
        if ACSm_obj and ACSm_obj.type == 'ARMATURE' and ACSm_bone_name:
            ACSm_bone_matrix = self.get_bone_world_matrix(ACSm_obj, ACSm_bone_name)
            use_ACSm_bone = ACSm_bone_matrix is not None
        
        # Store original transformations for ACSf - this is our fixed reference point
        if use_ACSf_bone:
            self._orig_ACSf_bone_matrix = ACSf_bone_matrix.copy()
            self._orig_ACSf_loc = None
            self._orig_ACSf_rotation = None
            self.report({'INFO'}, f"Using bone '{ACSf_bone_name}' as fixed ACS.")
        else:
            self._orig_ACSf_loc = ACSf_obj.location.copy()
            self._orig_ACSf_rotation = ACSf_obj.rotation_euler.copy()
            self.report({'INFO'}, f"Fixed ACS starting from rotation: {[math.degrees(r) for r in self._orig_ACSf_rotation]}")
            self.report({'INFO'}, f"Fixed ACS starting from location: {self._orig_ACSf_loc}")
            
        # Store original transformations for ACSm - this will be moved
        if use_ACSm_bone:
            self._orig_ACSm_bone_matrix = ACSm_bone_matrix.copy()
            self._orig_ACSm_loc = None
            self._orig_ACSm_rotation = None
            self.report({'INFO'}, f"Using bone '{ACSm_bone_name}' as mobile ACS.")
        else:
            self._orig_ACSm_loc = ACSm_obj.location.copy()
            self._orig_ACSm_rotation = ACSm_obj.rotation_euler.copy()
            self.report({'INFO'}, f"Mobile ACS starting from rotation: {[math.degrees(r) for r in self._orig_ACSm_rotation]}")
            self.report({'INFO'}, f"Mobile ACS starting from location: {self._orig_ACSm_loc}")
        
        # Keyframe the starting position at frame 0
        if use_ACSm_bone:
            pose_bone = ACSm_obj.pose.bones.get(ACSm_bone_name)
            if pose_bone:
                pose_bone.keyframe_insert(data_path="location", frame=0)
                pose_bone.keyframe_insert(data_path="rotation_euler", frame=0)
        else:
            ACSm_obj.keyframe_insert(data_path="location", frame=0)
            ACSm_obj.keyframe_insert(data_path="rotation_euler", frame=0)
        
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
        if prox_obj:
            self._prox_bvh = self.create_bvh_tree(prox_obj) # Use the main prox_obj for its own BVH
            if not self._prox_bvh:
                self.report({'ERROR'}, "Failed to create BVH tree for proximal object. Aborting calculation.")
                props.is_calculating = False
                self._is_initialized = False 
                return # Exit initialize_calculation early
        else:
            self.report({'ERROR'}, "Proximal object not set. Aborting calculation.")
            props.is_calculating = False
            self._is_initialized = False
            return # Exit initialize_calculation early
        
        # Initialize proximal hull and its BVH if optimization is enabled
        if props.use_convex_hull_optimization:
            self.report({'INFO'}, "Pre-calculating convex hull and BVH for proximal object...")
            if self._prox_hull_obj is None: # Check if already created
                self._prox_hull_obj = self.create_convex_hull_object(prox_obj)
            
            if self._prox_hull_obj and self._prox_hull_bvh is None: # Check if BVH already created
                 self._prox_hull_bvh = self.create_bvh_tree(self._prox_hull_obj)

            if not self._prox_hull_obj or not self._prox_hull_bvh:
                self.report({'WARNING'}, "Failed to create convex hull or its BVH for proximal object. Convex hull optimization will be less effective or disabled for prox.")
                # Continue without hull if it fails, check_collision will handle it or fall back

        self._is_initialized = True
        self._start_time = time.time()
        
        # Ensure cached hull attributes are reset for a new calculation run
        if hasattr(self, '_prox_hull_obj') and self._prox_hull_obj:
            if self._prox_hull_obj.name in bpy.data.objects: # Check if it still exists
                self.remove_temp_object(self._prox_hull_obj)
        self._prox_hull_obj = None
        self._prox_hull_bvh = None # BVH trees are just Python objects, no Blender data to remove directly
        
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
        ACSf_obj = props.ACSf_object
        ACSm_obj = props.ACSm_object
        ACSf_bone_name = getattr(props, 'ACSf_bone', None)
        ACSm_bone_name = getattr(props, 'ACSm_bone', None)
        use_ACSf_bone = ACSf_obj and ACSf_obj.type == 'ARMATURE' and ACSf_bone_name
        use_ACSm_bone = ACSm_obj and ACSm_obj.type == 'ARMATURE' and ACSm_bone_name
        
        # Store original transformations for restoration at end if using scene updates
        orig_ACSm_loc = ACSm_obj.location.copy() if not use_ACSm_bone else None
        orig_ACSm_rot = ACSm_obj.rotation_euler.copy() if not use_ACSm_bone else None
        
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
                
            rot_x = self._rot_x_range[self._cur_x_idx]  # X-axis (long axis rotation) from ACSm
            rot_y = self._rot_y_range[self._cur_y_idx]  # Y-axis (adduction/abduction) calculated
            rot_z = self._rot_z_range[self._cur_z_idx]  # Z-axis (flexion/extension) from ACSf
            trans_x = self._trans_x_range[self._cur_tx_idx]
            trans_y = self._trans_y_range[self._cur_ty_idx]
            trans_z = self._trans_z_range[self._cur_tz_idx]
              # Calculate proper anatomical rotations based on ACS objects
            rotation_matrix = self.calculate_acs_rotation(context, ACSf_obj, ACSm_obj, rot_x, rot_y, rot_z)
            
            # Apply rotations and translations to ACSm (mobile object)
            if use_ACSm_bone:
                pose_bone = ACSm_obj.pose.bones.get(ACSm_bone_name)
                if pose_bone:
                    # Apply the calculated rotation to the pose bone
                    orig_matrix = ACSm_obj.matrix_world.copy() @ pose_bone.matrix.copy()
                    new_matrix = orig_matrix @ rotation_matrix
                    
                    # Extract rotation and set it
                    pose_bone.rotation_mode = props.rot_order
                    pose_bone.rotation_euler = new_matrix.to_euler(props.rot_order)
                    
                    # Apply translations to ACSm
                    pose_bone.location = mathutils.Vector((trans_x, trans_y, trans_z))
                    context.view_layer.update()
            else:
                # Apply the calculated rotation to the ACSm object
                orig_matrix = ACSm_obj.matrix_world.copy()
                new_matrix = orig_matrix @ rotation_matrix
                
                # Extract rotation and set it
                ACSm_obj.rotation_mode = props.rot_order
                ACSm_obj.rotation_euler = new_matrix.to_euler(props.rot_order)
                
                # Apply translations to ACSm
                ACSm_obj.location = self._orig_ACSm_loc + mathutils.Vector((trans_x, trans_y, trans_z))
                context.view_layer.update()
            
            # Check for collision using the pre-calculated proximal BVH tree
            collision = self.check_collision(prox_obj, dist_obj, self._prox_bvh)
            
            # Record data for CSV export
            self._csv_data.append([rot_x, rot_y, rot_z, trans_x, trans_y, trans_z, 0 if collision else 1]) #Changed so that 0 is collision and 1 is valid pose.
            
            # If pose is collision-free, insert keyframes immediately
            if not collision:
                if props.visualize_collisions:
                    if use_ACSm_bone:
                        pose_bone = ACSm_obj.pose.bones.get(ACSm_bone_name)
                        if pose_bone:
                            pose_bone.keyframe_insert(data_path="location", frame=self._non_collision_frame)
                            pose_bone.keyframe_insert(data_path="rotation_euler", frame=self._non_collision_frame)
                    else:
                        ACSm_obj.keyframe_insert(data_path="location", frame=self._non_collision_frame)
                        ACSm_obj.keyframe_insert(data_path="rotation_euler", frame=self._non_collision_frame)
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
        if use_ACSm_bone:
            pose_bone = ACSm_obj.pose.bones.get(ACSm_bone_name)
            if pose_bone:
                pose_bone.location = mathutils.Vector((0,0,0))
                pose_bone.rotation_euler = mathutils.Euler((0,0,0), props.rot_order)
                context.view_layer.update()
        elif orig_ACSm_loc is not None and orig_ACSm_rot is not None:
            self.restore_object_transform(ACSm_obj, orig_ACSm_loc, orig_ACSm_rot)
            context.view_layer.update()
        
        # Check if we're done
        if self._is_finished:
            self.finalize_calculation(context)
            return False
            
        return True
    
    def finalize_calculation(self, context, cancelled=False):
        """Complete the calculation and process results"""
        props = context.scene.collision_props
        
        # Reset ACSm object to original position
        ACSm_obj = props.ACSm_object
        ACSm_bone_name = getattr(props, 'ACSm_bone', None)
        use_ACSm_bone = ACSm_obj and ACSm_obj.type == 'ARMATURE' and ACSm_bone_name
        
        if use_ACSm_bone:
            pose_bone = ACSm_obj.pose.bones.get(ACSm_bone_name)
            if pose_bone:
                pose_bone.location = mathutils.Vector((0,0,0))
                pose_bone.rotation_euler = mathutils.Euler((0,0,0), props.rot_order)
                context.view_layer.update()
        elif hasattr(self, '_orig_ACSm_loc') and hasattr(self, '_orig_ACSm_rotation'):
            if self._orig_ACSm_loc is not None and self._orig_ACSm_rotation is not None:
                self.restore_object_transform(ACSm_obj, self._orig_ACSm_loc, self._orig_ACSm_rotation)
                context.view_layer.update()
        
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
        if hasattr(self, '_orig_ACSf_loc'):
            self._orig_ACSf_loc = None
        if hasattr(self, '_orig_ACSf_rotation'):
            self._orig_ACSf_rotation = None
        if hasattr(self, '_orig_ACSf_bone_matrix'):
            self._orig_ACSf_bone_matrix = None
        if hasattr(self, '_orig_ACSm_loc'):
            self._orig_ACSm_loc = None
        if hasattr(self, '_orig_ACSm_rotation'):
            self._orig_ACSm_rotation = None
        if hasattr(self, '_orig_ACSm_bone_matrix'):
            self._orig_ACSm_bone_matrix = None
        
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

