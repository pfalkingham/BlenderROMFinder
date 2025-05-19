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

    def calculate_acs_rotation(self, context, ACSf_obj, ACSm_obj, rot_x_degrees, rot_y_degrees, rot_z_degrees):
        props = context.scene.collision_props
        import mathutils
        import math
        from mathutils import Matrix, Vector

        current_transform_local_to_ACSf = Matrix.Identity(4)

        # 1. Flexion/Extension (FE) - around ACSf's local Z
        axis_Z_FE_in_ACSf_local = Vector((0, 0, 1))
        rot_mat_FE = Matrix.Rotation(math.radians(rot_z_degrees), 4, axis_Z_FE_in_ACSf_local)
        current_transform_local_to_ACSf = current_transform_local_to_ACSf @ rot_mat_FE

        # 2. Adduction/Abduction (AD/AB)
        if hasattr(props, 'rotation_mode_enum') and props.rotation_mode_enum == 'ISB_STANDARD':
            # ISB Standard Floating Y-axis
            axis_X_ACSm_in_ACSf_local_after_FE = current_transform_local_to_ACSf.col[0].to_3d().normalized()
            axis_Y_ADAB_in_ACSf_local = axis_Z_FE_in_ACSf_local.cross(axis_X_ACSm_in_ACSf_local_after_FE)
            if axis_Y_ADAB_in_ACSf_local.length < 1e-8:
                axis_Y_ADAB_in_ACSf_local = Vector((0, 1, 0))
            else:
                axis_Y_ADAB_in_ACSf_local.normalize()
        elif hasattr(props, 'rotation_mode_enum') and props.rotation_mode_enum == 'INTUITIVE':
            # Intuitive Mode: AD/AB always around ACSf's fixed local Y-axis
            axis_Y_ADAB_in_ACSf_local = Vector((0, 1, 0))
        else:
            axis_Y_ADAB_in_ACSf_local = Vector((0, 1, 0))

        rot_mat_ADAB = Matrix.Rotation(math.radians(rot_y_degrees), 4, axis_Y_ADAB_in_ACSf_local)
        current_transform_local_to_ACSf = current_transform_local_to_ACSf @ rot_mat_ADAB

        # 3. Long-Axis Rotation (LAR) - around ACSm's new local X-axis
        rot_mat_LAR_local_to_new_ACSm_frame = Matrix.Rotation(math.radians(rot_x_degrees), 4, Vector((1, 0, 0)))
        current_transform_local_to_ACSf = current_transform_local_to_ACSf @ rot_mat_LAR_local_to_new_ACSm_frame

        return current_transform_local_to_ACSf

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
            # For completeness, store initial local matrix of the pose bone
            pose_bone = ACSm_obj.pose.bones.get(ACSm_bone_name)
            if pose_bone:
                self._initial_ACSm_bone_matrix_local = pose_bone.matrix.copy()
        else:
            self._orig_ACSm_loc = ACSm_obj.location.copy()
            self._orig_ACSm_rotation = ACSm_obj.rotation_euler.copy()
            self.report({'INFO'}, f"Mobile ACS starting from rotation: {[math.degrees(r) for r in self._orig_ACSm_rotation]}")
            self.report({'INFO'}, f"Mobile ACS starting from location: {self._orig_ACSm_loc}")
            # Store initial local matrix of ACSm_obj
            self._initial_ACSm_matrix_local = ACSm_obj.matrix_local.copy()

        # Store initial relative transform of ACSm to ACSf (object or bone)
        mat_ACSf_world_initial = ACSf_bone_matrix.copy() if use_ACSf_bone else ACSf_obj.matrix_world.copy()
        mat_ACSm_world_initial = ACSm_bone_matrix.copy() if use_ACSm_bone else ACSm_obj.matrix_world.copy()
        self._initial_relative_ACSm_to_ACSf_matrix = mat_ACSf_world_initial.inverted() @ mat_ACSm_world_initial

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
        # Ensure Vector is available if not imported at the top of the file
        # from mathutils import Vector (should be at the top of your file already)
        # from mathutils import Matrix (should be at the top of your file already)

        if not self._is_initialized or self._is_finished:
            return False
        
        batch_size = props.batch_size
        
        prox_obj = props.proximal_object
        dist_obj = props.distal_object
        ACSf_obj = props.ACSf_object
        ACSm_obj = props.ACSm_object
        ACSf_bone_name = getattr(props, 'ACSf_bone', None) # Your existing getattr
        ACSm_bone_name = getattr(props, 'ACSm_bone', None) # Your existing getattr
        use_ACSf_bone = ACSf_obj and ACSf_obj.type == 'ARMATURE' and ACSf_bone_name # Your existing check
        use_ACSm_bone = ACSm_obj and ACSm_obj.type == 'ARMATURE' and ACSm_bone_name # Your existing check
        
        # Store original transformations for restoration at end if using scene updates
        # These are for the ACSm_obj itself if it's not a bone context for JCS application
        orig_ACSm_loc_if_object = ACSm_obj.location.copy() if not use_ACSm_bone else None
        orig_ACSm_rot_if_object = ACSm_obj.rotation_euler.copy() if not use_ACSm_bone else None
        
        batch_counter = 0
        # last_view_update = 0 # This variable wasn't used in your uploaded code, can be removed if not needed
        
        while batch_counter < batch_size:
            # Check if we need to stop
            if not props.is_calculating:
                self.report({'INFO'}, "Calculation cancelled by user")
                self.finalize_calculation(context, cancelled=True)
                return False # Stop processing this batch
            
            # Check if all iterations are complete
            if self._cur_x_idx >= len(self._rot_x_range):
                self._is_finished = True
                break # Exit while loop, will go to finalize_calculation
                
            # Get current JCS rotation angles and translation values from loop ranges
            rot_x = self._rot_x_range[self._cur_x_idx]
            rot_y = self._rot_y_range[self._cur_y_idx]
            rot_z = self._rot_z_range[self._cur_z_idx]
            trans_x = self._trans_x_range[self._cur_tx_idx]
            trans_y = self._trans_y_range[self._cur_ty_idx]
            trans_z = self._trans_z_range[self._cur_tz_idx]

            # 1. Calculate the JCS ROTATIONAL matrix (local to ACSf)
            jcs_orientation_matrix_local_to_ACSf = self.calculate_acs_rotation(
                context, ACSf_obj, ACSm_obj, rot_x, rot_y, rot_z
            )

            # 2. Combine this with ACSm's initial local orientation to get the target rotated pose
            if use_ACSm_bone:
                initial_local_orientation_matrix = self._initial_ACSm_bone_matrix_local
            else:
                initial_local_orientation_matrix = self._initial_ACSm_matrix_local
            
            target_orientation_matrix_local = initial_local_orientation_matrix @ jcs_orientation_matrix_local_to_ACSf
            
            # --- START OF INTEGRATED TRANSLATION LOGIC ---
            final_ACSm_pose_matrix_local = target_orientation_matrix_local.copy() # Start with the rotated pose

            translation_mode = getattr(props, 'translation_mode_enum', 'SIMPLE_ACSf') # Default if prop not added yet

            if translation_mode == 'SIMPLE_ACSf':
                translation_offset_in_ACSf_local_space = mathutils.Vector((trans_x, trans_y, trans_z))
                initial_local_translation = initial_local_orientation_matrix.to_translation() # Get translation from initial local
                final_ACSm_pose_matrix_local.translation = initial_local_translation + translation_offset_in_ACSf_local_space

            elif translation_mode == 'ACSM_LOCAL_POST_ROT':
                translation_vector_local_to_rotated_ACSm = mathutils.Vector((trans_x, trans_y, trans_z))
                local_translation_offset_matrix = mathutils.Matrix.Translation(translation_vector_local_to_rotated_ACSm)
                final_ACSm_pose_matrix_local = final_ACSm_pose_matrix_local @ local_translation_offset_matrix
            
            elif translation_mode == 'MG_PRISM_HINGE':
                # Define initial prism axes (local to ACSf). 
                # TODO: These should ideally be user-configurable or derived more robustly.
                initial_prism_axis_distraction_vec = mathutils.Vector((1, 0, 0)) # Example: ACSf local X
                initial_prism_axis_ap_glide_vec    = mathutils.Vector((0, 1, 0)) # Example: ACSf local Y
                initial_prism_axis_ml_shift_vec    = mathutils.Vector((0, 0, 1)) # Example: ACSf local Z (less common for ML)
                                                                            # A more common ML for a knee might be its FE axis.
                                                                            # Or derived as cross product of other two prism axes.

                # Primary hinge rotation is assumed to be rot_z (FE around ACSf's local Z)
                rot_mat_FE_only = mathutils.Matrix.Rotation(math.radians(rot_z), 4, mathutils.Vector((0,0,1)))

                current_prism_axis_distraction = (rot_mat_FE_only @ initial_prism_axis_distraction_vec.to_4d()).to_3d()
                current_prism_axis_ap_glide    = (rot_mat_FE_only @ initial_prism_axis_ap_glide_vec.to_4d()).to_3d()
                current_prism_axis_ml_shift    = (rot_mat_FE_only @ initial_prism_axis_ml_shift_vec.to_4d()).to_3d()
                # Ensure they remain unit vectors if they were initially (multiplication by rotation matrix preserves length)

                translation_offset_in_ACSf_local_space = \
                    (current_prism_axis_distraction * trans_x) + \
                    (current_prism_axis_ap_glide    * trans_y) + \
                    (current_prism_axis_ml_shift    * trans_z)
                
                initial_local_translation = initial_local_orientation_matrix.to_translation()
                final_ACSm_pose_matrix_local.translation = initial_local_translation + translation_offset_in_ACSf_local_space
            
            else: 
                self.report({'WARNING'}, f"Unknown translation mode: {translation_mode}. Defaulting to SIMPLE_ACSf.")
                translation_offset_in_ACSf_local_space = mathutils.Vector((trans_x, trans_y, trans_z))
                initial_local_translation = initial_local_orientation_matrix.to_translation()
                final_ACSm_pose_matrix_local.translation = initial_local_translation + translation_offset_in_ACSf_local_space
            # --- END OF INTEGRATED TRANSLATION LOGIC ---

            # Apply this final combined matrix to ACSm_obj (or its bone):
            if use_ACSm_bone:
                pose_bone = ACSm_obj.pose.bones.get(ACSm_bone_name)
                if pose_bone:
                    pose_bone.matrix = final_ACSm_pose_matrix_local 
            else:
                ACSm_obj.matrix_local = final_ACSm_pose_matrix_local
            
            context.view_layer.update()

            # Check for collision using the pre-calculated proximal BVH tree
            collision = self.check_collision(prox_obj, dist_obj, self._prox_bvh)
            
            # Record data for CSV export
            self._csv_data.append([rot_x, rot_y, rot_z, trans_x, trans_y, trans_z, 0 if collision else 1])
            
            # Keyframing logic
            if not collision:
                if props.visualize_collisions:
                    if use_ACSm_bone:
                        pose_bone = ACSm_obj.pose.bones.get(ACSm_bone_name)
                        if pose_bone:
                            # Keyframe the bone's local location and rotation
                            pose_bone.keyframe_insert(data_path="location", frame=self._non_collision_frame)
                            pose_bone.keyframe_insert(data_path="rotation_euler", frame=self._non_collision_frame)
                    else:
                        # Keyframe the object's local location and rotation
                        ACSm_obj.keyframe_insert(data_path="location", frame=self._non_collision_frame)
                        ACSm_obj.keyframe_insert(data_path="rotation_euler", frame=self._non_collision_frame)
                self._non_collision_frame += 1
            
            # Increment indices (your existing efficient logic)
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
                                    # No 'break' here, let the check at the start of the
                                    # while loop or the batch_counter handle loop exit
                                    # for the current batch. The _is_finished flag will
                                    # terminate the overall processing.
            
            self._completed_iterations += 1
            batch_counter += 1
            
            # Update progress in the UI 
            if self._total_iterations > 0:
                progress_pct = (self._completed_iterations / self._total_iterations) * 100
                props.calculation_progress = progress_pct
                if self._completed_iterations > 0 and self._start_time:
                    elapsed = time.time() - self._start_time
                    if elapsed > 0: # Avoid division by zero if first batch is instant
                        rate = self._completed_iterations / elapsed
                        remaining_iterations = self._total_iterations - self._completed_iterations
                        remaining_time_sec = remaining_iterations / rate if rate > 0 else 0
                    else:
                        remaining_time_sec = float('inf') if self._total_iterations > 0 else 0 # Show infinity if no time elapsed but work remains
                    
                    if remaining_time_sec == float('inf'):
                        props.time_remaining = "Time remaining: Calculating..."
                    else:
                        mins, secs = divmod(int(remaining_time_sec), 60)
                        props.time_remaining = f"Time remaining: {mins:02d}:{secs:02d}"
                else:
                    props.time_remaining = "Calculating..."
                
                if (batch_counter % 25 == 0): # Your existing UI refresh
                    for area in context.screen.areas:
                        if area.type == 'VIEW_3D':
                            area.tag_redraw()
        
        # Restore original position and update
        # This resets ACSm_obj to its state BEFORE this batch, not its absolute initial state,
        # which is correct for modal operation if it was being transformed for visualization.
        # However, since we are setting ACSm_obj.matrix_local directly from _initial_...
        # this restoration might not be strictly necessary here unless you have other
        # UI updates that temporarily move it. For safety, it's okay.
        if use_ACSm_bone:
            pose_bone = ACSm_obj.pose.bones.get(ACSm_bone_name)
            if pose_bone and hasattr(self, '_initial_ACSm_bone_matrix_local'): # Check if initial was stored
                pose_bone.matrix = self._initial_ACSm_bone_matrix_local
                # Resetting location/rotation_euler might be redundant if matrix is set
                # pose_bone.location = mathutils.Vector((0,0,0))
                # pose_bone.rotation_euler = mathutils.Euler((0,0,0), props.rot_order)
                context.view_layer.update()
        elif orig_ACSm_loc_if_object is not None and orig_ACSm_rot_if_object is not None:
             # If ACSm is an object, restore its local transform to what it was before this batch run
             # (or better, just set it to its _initial_ACSm_matrix_local state)
            if hasattr(self, '_initial_ACSm_matrix_local'):
                ACSm_obj.matrix_local = self._initial_ACSm_matrix_local
            else: # Fallback to what you had if _initial_ACSm_matrix_local isn't available for some reason
                 self.restore_object_transform(ACSm_obj, orig_ACSm_loc_if_object, orig_ACSm_rot_if_object)
            context.view_layer.update()
        
        if self._is_finished:
            self.finalize_calculation(context)
            return False # Done with all iterations
            
        return True # More batches to process
    
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

