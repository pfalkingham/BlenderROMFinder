import bpy
import bmesh
import mathutils
import math
import numpy as np
import csv
import os
import json
import time
from mathutils import Matrix, Vector # Explicitly import Vector
from bpy.types import Operator
from bpy.props import StringProperty
from .poseCalculations import (
    get_jcs_rotation_part,
    calculate_jcs_orientation_matrix_local_to_acsf,
    calculate_pose_for_isb_standard_mode,
    calculate_pose_for_intuitive_mode,
    calculate_pose_for_mg_hinge_mode
)

class COLLISION_OT_cancel(Operator):
    """Cancel the current collision calculation"""
    bl_idname = "collision.cancel"
    bl_label = "Cancel Calculation"
    
    def execute(self, context):
        props = context.scene.collision_props
        
        if props.is_calculating:
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
    
    _timer = None
    _is_initialized = False
    _is_finished = False
    
    _rot_x_range = []
    _rot_y_range = []
    _rot_z_range = []
    _trans_x_range = []
    _trans_y_range = []
    _trans_z_range = []
    _csv_data = []
    _total_iterations = 0
    _completed_iterations = 0
    _prox_bvh = None
    
    _initial_ACSm_matrix_local = None # For ACSm object
    _initial_ACSm_bone_matrix_local = None # For ACSm bone
    
    _prox_hull_obj = None 
    _prox_hull_bvh = None
    
    _cur_x_idx = 0
    _cur_y_idx = 0
    _cur_z_idx = 0
    _cur_tx_idx = 0
    _cur_ty_idx = 0
    _cur_tz_idx = 0
    
    _start_time = None
    
    # --- BVH and Collision Helper Methods (from your existing code, assumed correct) ---
    def create_bvh_tree(self, obj, transform_matrix=None):
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
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.duplicate()
        hull_obj = bpy.context.active_object
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.convex_hull()
        bpy.ops.object.mode_set(mode='OBJECT')
        return hull_obj

    def remove_temp_object(self, obj):
        bpy.data.objects.remove(obj, do_unlink=True)

    def restore_object_transform(self, obj, location, rotation):
        obj.location = location.copy()
        obj.rotation_euler = rotation.copy()

    def check_collision(self, prox_obj, dist_obj, prox_bvh_original_mesh=None, transform_matrix=None): # Added transform_matrix
        props = bpy.context.scene.collision_props
        if props.use_convex_hull_optimization:
            if self._prox_hull_obj is None: self._prox_hull_obj = self.create_convex_hull_object(prox_obj)
            if self._prox_hull_bvh is None and self._prox_hull_obj: self._prox_hull_bvh = self.create_bvh_tree(self._prox_hull_obj)
            
            if self._prox_hull_bvh:
                temp_dist_hull_obj = self.create_convex_hull_object(dist_obj)
                if temp_dist_hull_obj:
                    current_dist_hull_bvh = self.create_bvh_tree(temp_dist_hull_obj)
                    if temp_dist_hull_obj.name in bpy.data.objects: self.remove_temp_object(temp_dist_hull_obj)
                    if current_dist_hull_bvh and not self._prox_hull_bvh.overlap(current_dist_hull_bvh):
                        return False 
        
        # If not using hull opt, or if hulls overlapped, or hull creation failed:
        # transform_matrix for dist_obj is its current matrix_world for this check
        dist_bvh_original_mesh = self.create_bvh_tree(dist_obj, dist_obj.matrix_world if transform_matrix is None else transform_matrix)
        
        # prox_bvh_original_mesh is self._prox_bvh (BVH of original prox_obj mesh)
        if not self._prox_bvh or not dist_bvh_original_mesh:
            self.report({'ERROR'}, "BVH for original mesh check failed.")
            return True 

        return len(self._prox_bvh.overlap(dist_bvh_original_mesh)) > 0

    # --- Main Execution Flow ---
    def initialize_calculation(self, context):
        # Explicitly initialize potentially optional instance attributes to None
        self._initial_ACSm_matrix_local = None
        self._initial_ACSm_bone_matrix_local = None
        self._prox_obj = None # Add others from your attrs_to_clear that might be optional
        self._dist_obj = None
        self._transform_target = None
        self._saved_target_matrix_local = None
        
        props = context.scene.collision_props
        if not props.proximal_object or not props.distal_object: self.report({'ERROR'}, "Proximal/Distal objects missing"); return False
        if not props.ACSf_object or not props.ACSm_object: self.report({'ERROR'}, "ACSf/ACSm objects missing"); return False

        self._prox_obj = props.proximal_object # Renamed for clarity
        self._dist_obj = props.distal_object   # Renamed for clarity

        # Store initial local matrix of ACSm (object or bone)
        ACSm_obj = props.ACSm_object
        ACSm_bone_name = getattr(props, 'ACSm_bone', None)
        use_ACSm_bone = ACSm_obj and ACSm_obj.type == 'ARMATURE' and ACSm_bone_name
        if use_ACSm_bone:
            pose_bone = ACSm_obj.pose.bones.get(ACSm_bone_name)
            if pose_bone: self._initial_ACSm_bone_matrix_local = pose_bone.matrix.copy()
            else: self.report({'ERROR'}, f"ACSm bone '{ACSm_bone_name}' not found."); return False
        else:
            self._initial_ACSm_matrix_local = ACSm_obj.matrix_local.copy()
        
        # Save initial state for final reset (using actual object to be transformed)
        self._transform_target = ACSm_obj # Assume ACSm_obj is what gets transformed
        self._saved_target_matrix_local = self._transform_target.matrix_local.copy()


        # Ranges (avoid duplicates due to float steps)
        def gen_range(min_val, max_val, inc):
            """Generate float range inclusive of max_val, rounded to avoid FP errors, and remove duplicates."""
            if inc <= 0:
                return [round(min_val, 6)]
            vals = []
            current = min_val
            eps = inc * 1e-6
            while current <= max_val + eps:
                vals.append(round(current, 6))
                current += inc
            # Remove duplicates while preserving order
            seen = set()
            unique_vals = []
            for v in vals:
                if v not in seen:
                    seen.add(v)
                    unique_vals.append(v)
            return unique_vals

        self._rot_x_range = gen_range(props.rot_x_min, props.rot_x_max, props.rot_x_inc)
        self._rot_y_range = gen_range(props.rot_y_min, props.rot_y_max, props.rot_y_inc)
        self._rot_z_range = gen_range(props.rot_z_min, props.rot_z_max, props.rot_z_inc)
        self._trans_x_range = gen_range(props.trans_x_min, props.trans_x_max, props.trans_x_inc)
        self._trans_y_range = gen_range(props.trans_y_min, props.trans_y_max, props.trans_y_inc)
        self._trans_z_range = gen_range(props.trans_z_min, props.trans_z_max, props.trans_z_inc)

        if not self._rot_x_range: self._rot_x_range = [props.rot_x_min]
        # ... (ensure all ranges have at least one value) ...
        if not self._rot_y_range: self._rot_y_range = [props.rot_y_min]
        if not self._rot_z_range: self._rot_z_range = [props.rot_z_min]
        if not self._trans_x_range: self._trans_x_range = [props.trans_x_min]
        if not self._trans_y_range: self._trans_y_range = [props.trans_y_min]
        if not self._trans_z_range: self._trans_z_range = [props.trans_z_min]

        self._total_iterations = len(self._rot_x_range) * len(self._rot_y_range) * len(self._rot_z_range) * \
                                 len(self._trans_x_range) * len(self._trans_y_range) * len(self._trans_z_range)
        if self._total_iterations == 0: self.report({'WARNING'}, "Total iterations is zero."); return False

        self._csv_data = [["rot_x", "rot_y", "rot_z", "trans_x", "trans_y", "trans_z", "Valid_pose"]]
        self._completed_iterations = 0
        
        self.report({'INFO'}, "Pre-calculating BVH for proximal object...")
        if self._prox_obj: self._prox_bvh = self.create_bvh_tree(self._prox_obj)
        if not self._prox_bvh: self.report({'ERROR'}, "Proximal BVH failed."); return False
        
        if props.use_convex_hull_optimization: # Reset hull cache
            if hasattr(self, '_prox_hull_obj') and self._prox_hull_obj:
                if self._prox_hull_obj.name in bpy.data.objects: self.remove_temp_object(self._prox_hull_obj)
            self._prox_hull_obj = None
            self._prox_hull_bvh = None

        # Keyframe ACSm at frame 0 (its initial state)
        target_for_keyframe = ACSm_obj
        if use_ACSm_bone: target_for_keyframe = ACSm_obj.pose.bones.get(ACSm_bone_name)
        
        if target_for_keyframe:
            target_for_keyframe.keyframe_insert(data_path="location", frame=0)
            target_for_keyframe.keyframe_insert(data_path="rotation_euler", frame=0)

        self._cur_x_idx, self._cur_y_idx, self._cur_z_idx = 0,0,0
        self._cur_tx_idx, self._cur_ty_idx, self._cur_tz_idx = 0,0,0
        self._non_collision_frame = 1
        self._is_initialized = True
        self._start_time = time.time()
        props.calculation_progress = 0.0
        props.is_calculating = True # This is set by execute, but good to ensure
        self.report({'INFO'}, f"Initialization complete. Total iterations: {self._total_iterations}")
        return True

    def process_batch(self, context):
        props = context.scene.collision_props

        if not self._is_initialized or self._is_finished or not props.is_calculating:
            return False
        
        batch_size = props.batch_size
        
        # ACSf_obj = props.ACSf_object # Not directly needed here if passed to helpers
        ACSm_obj = props.ACSm_object 
        ACSm_bone_name = getattr(props, 'ACSm_bone', None)
        use_ACSm_bone = ACSm_obj and ACSm_obj.type == 'ARMATURE' and ACSm_bone_name
        
        if use_ACSm_bone:
            acsm_initial_local_matrix_for_calc = self._initial_ACSm_bone_matrix_local
        else:
            acsm_initial_local_matrix_for_calc = self._initial_ACSm_matrix_local

        batch_counter = 0
        while batch_counter < batch_size:
            # ... (Standard checks for props.is_calculating and self._is_finished) ...
            if not props.is_calculating: self.finalize_calculation(context, cancelled=True); return False
            if self._cur_x_idx >= len(self._rot_x_range): self._is_finished = True; break # Should be caught by process_batch return too
                
            rx = self._rot_x_range[self._cur_x_idx]
            ry = self._rot_y_range[self._cur_y_idx]
            rz = self._rot_z_range[self._cur_z_idx]
            tx = self._trans_x_range[self._cur_tx_idx]
            ty = self._trans_y_range[self._cur_ty_idx]
            tz = self._trans_z_range[self._cur_tz_idx]

            final_ACSm_pose_matrix_local = None
            # Use your single enum prop here:
            operational_mode = getattr(props, 'rotation_mode_enum', 'MG_HINGE') # Default to your current default

            if operational_mode == 'ISB_STANDARD':
                final_ACSm_pose_matrix_local = calculate_pose_for_isb_standard_mode(rx, ry, rz, tx, ty, tz, props, acsm_initial_local_matrix_for_calc, acsm_obj=ACSm_obj, acsf_obj=props.ACSf_object)
            elif operational_mode == 'INTUITIVE':
                final_ACSm_pose_matrix_local = calculate_pose_for_intuitive_mode(rx, ry, rz, tx, ty, tz, props, acsm_initial_local_matrix_for_calc, acsm_obj=ACSm_obj, acsf_obj=props.ACSf_object)
            elif operational_mode == 'MG_HINGE':
                final_ACSm_pose_matrix_local = calculate_pose_for_mg_hinge_mode(rx, ry, rz, tx, ty, tz, props, acsm_initial_local_matrix_for_calc)
            else:
                self.report({'WARNING'}, f"Unknown operational_mode: {operational_mode}. Skipping pose.")
            
            if final_ACSm_pose_matrix_local:
                if use_ACSm_bone:
                    pose_bone = ACSm_obj.pose.bones.get(ACSm_bone_name)
                    if pose_bone: pose_bone.matrix = final_ACSm_pose_matrix_local
                else:
                    ACSm_obj.matrix_local = final_ACSm_pose_matrix_local
                context.view_layer.update()

                # --- MODIFIED: Skip collision check if debug_mode and turn_off_collisions are both True ---
                if props.debug_mode and props.turn_off_collisions:
                    collision = False
                else:
                    collision = self.check_collision(self._prox_obj, self._dist_obj, self._prox_bvh)
                self._csv_data.append([rx, ry, rz, tx, ty, tz, 0 if collision else 1])
                
                if (not collision and props.visualize_collisions) or (props.debug_mode):
                    target_for_keyframe = ACSm_obj
                    if use_ACSm_bone: target_for_keyframe = ACSm_obj.pose.bones.get(ACSm_bone_name)
                    if target_for_keyframe:
                        # Set custom properties for input rotations, translations, and collision
                        target_for_keyframe["input_rot_x"] = rx
                        target_for_keyframe["input_rot_y"] = ry
                        target_for_keyframe["input_rot_z"] = rz
                        target_for_keyframe["input_trans_x"] = tx
                        target_for_keyframe["input_trans_y"] = ty
                        target_for_keyframe["input_trans_z"] = tz
                        target_for_keyframe["Valid pose"] = 0 if collision else 1
                        # Insert keyframes for these custom properties
                        target_for_keyframe.keyframe_insert(data_path='["input_rot_x"]', frame=self._non_collision_frame)
                        target_for_keyframe.keyframe_insert(data_path='["input_rot_y"]', frame=self._non_collision_frame)
                        target_for_keyframe.keyframe_insert(data_path='["input_rot_z"]', frame=self._non_collision_frame)
                        target_for_keyframe.keyframe_insert(data_path='["input_trans_x"]', frame=self._non_collision_frame)
                        target_for_keyframe.keyframe_insert(data_path='["input_trans_y"]', frame=self._non_collision_frame)
                        target_for_keyframe.keyframe_insert(data_path='["input_trans_z"]', frame=self._non_collision_frame)
                        target_for_keyframe.keyframe_insert(data_path='["Valid pose"]', frame=self._non_collision_frame)
                        target_for_keyframe.keyframe_insert(data_path="location", frame=self._non_collision_frame)
                        target_for_keyframe.keyframe_insert(data_path="rotation_euler", frame=self._non_collision_frame)
                    self._non_collision_frame += 1
            
            # ... (Standard index incrementing, _completed_iterations, batch_counter, UI update) ...
            # ... (Your existing efficient index increment logic) ...
            self._cur_tz_idx += 1
            if self._cur_tz_idx >= len(self._trans_z_range):
                self._cur_tz_idx = 0; self._cur_ty_idx += 1
                if self._cur_ty_idx >= len(self._trans_y_range):
                    self._cur_ty_idx = 0; self._cur_tx_idx += 1
                    if self._cur_tx_idx >= len(self._trans_x_range):
                        self._cur_tx_idx = 0; self._cur_z_idx += 1
                        if self._cur_z_idx >= len(self._rot_z_range):
                            self._cur_z_idx = 0; self._cur_y_idx += 1
                            if self._cur_y_idx >= len(self._rot_y_range):
                                self._cur_y_idx = 0; self._cur_x_idx += 1
                                if self._cur_x_idx >= len(self._rot_x_range):
                                    self._is_finished = True; break 
            
            self._completed_iterations += 1
            batch_counter += 1
            
            if self._total_iterations > 0: 
                props.calculation_progress = (self._completed_iterations / self._total_iterations) * 100.0
                # (your time remaining and UI refresh logic)

        # ... (Standard end of batch: reset ACSm visual for next batch, check if finished) ...
        # Reset ACSm to its initial state for this batch processing
        # (not strictly necessary if matrix_local is always fully overwritten, but safe for UI)
        if hasattr(self, '_transform_target') and self._transform_target and hasattr(self, '_saved_target_matrix_local'):
             # _saved_target_matrix_local was set in initialize_calculation to ACSm's initial local matrix
            if use_ACSm_bone:
                pose_bone = self._transform_target.pose.bones.get(ACSm_bone_name)
                if pose_bone: pose_bone.matrix = self._saved_target_matrix_local 
            else:
                self._transform_target.matrix_local = self._saved_target_matrix_local
            context.view_layer.update()

        if self._is_finished: self.finalize_calculation(context); return False
        return True

    def finalize_calculation(self, context, cancelled=False):
        props = context.scene.collision_props
        if hasattr(self, '_transform_target') and self._transform_target and hasattr(self, '_saved_target_matrix_local'):
            ACSm_obj = props.ACSm_object # Ensure we use the prop
            ACSm_bone_name = getattr(props, 'ACSm_bone', None)
            use_ACSm_bone = ACSm_obj and ACSm_obj.type == 'ARMATURE' and ACSm_bone_name
            if use_ACSm_bone:
                pose_bone = ACSm_obj.pose.bones.get(ACSm_bone_name)
                if pose_bone: pose_bone.matrix = self._initial_ACSm_bone_matrix_local # Reset to true initial
            else:
                ACSm_obj.matrix_local = self._initial_ACSm_matrix_local # Reset to true initial
            context.view_layer.update()

        if hasattr(self, '_prox_hull_obj') and self._prox_hull_obj:
            if self._prox_hull_obj.name in bpy.data.objects: self.remove_temp_object(self._prox_hull_obj)
        self._prox_hull_obj = None; self._prox_hull_bvh = None

        if not cancelled and hasattr(self, '_csv_data') and props.export_to_csv and props.export_path:
            # ... (CSV export logic from your file) ...
            filepath = bpy.path.abspath(props.export_path)
            dirpath = os.path.dirname(filepath)
            if dirpath: os.makedirs(dirpath, exist_ok=True)
            with open(filepath, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerows(self._csv_data)
            self.report({'INFO'}, f"Collision data exported to {filepath}")
            if hasattr(self, '_non_collision_frame'):
                 self.report({'INFO'}, f"Inserted {self._non_collision_frame-1} keyframes on collision-free poses")

        if hasattr(self, '_start_time') and self._start_time : # Check _start_time exists
            elapsed = time.time() - self._start_time
            mins, secs = divmod(int(elapsed), 60)
            self.report({'INFO'}, f"Total time taken: {mins:02d}:{secs:02d}")
        
        props.is_calculating = False
        # props.calculation_progress = 100.0 if not cancelled and self._completed_iterations >= self._total_iterations else props.calculation_progress
        if not cancelled and hasattr(self, '_completed_iterations') and hasattr(self, '_total_iterations') and self._total_iterations > 0:
            if self._completed_iterations >= self._total_iterations:
                props.calculation_progress = 100.0
        # else keep props.calculation_progress as is if cancelled

        self._is_initialized = False; self._is_finished = True
        
        # Clear instance variables more safely
        attrs_to_clear = ['_rot_x_range', '_rot_y_range', '_rot_z_range', '_trans_x_range', 
                          '_trans_y_range', '_trans_z_range', '_csv_data', '_total_iterations', 
                          '_completed_iterations', '_prox_bvh', '_initial_ACSm_matrix_local', 
                          '_initial_ACSm_bone_matrix_local', '_prox_hull_obj', '_prox_hull_bvh',
                          '_cur_x_idx', '_cur_y_idx', '_cur_z_idx', '_cur_tx_idx', '_cur_ty_idx', 
                          '_cur_tz_idx', '_start_time', '_non_collision_frame', 
                          '_prox_obj', '_dist_obj', '_transform_target', '_saved_target_matrix_local']
        for attr in attrs_to_clear:
            if hasattr(self, attr): delattr(self, attr)


    # Modal, Execute, Cancel methods (assumed largely okay from your structure, but check finalize calls)
    def modal(self, context, event):
        props = context.scene.collision_props
        if not props.is_calculating: # If something external stopped it
            if hasattr(self, '_timer') and self._timer: context.window_manager.event_timer_remove(self._timer); self._timer = None
            if hasattr(self, '_is_initialized') and self._is_initialized : # Ensure finalize is called if it was running
                self.finalize_calculation(context, cancelled=True) # Assume cancelled
            return {'FINISHED'}

        if event.type == 'TIMER':
            if not self.process_batch(context): # process_batch returns False if all done or error/cancel
                # finalize_calculation is called within process_batch if self._is_finished
                # or by COLLISION_OT_cancel if props.is_calculating becomes false.
                # So, if we reach here and process_batch is False, it means it's time to stop modal.
                if hasattr(self, '_timer') and self._timer: context.window_manager.event_timer_remove(self._timer); self._timer = None
                return {'FINISHED'}
        
        elif event.type == 'ESC' and event.value == 'PRESS':
            props.is_calculating = False # Signal cancellation
            # The check at the start of modal will handle finalize and timer removal
            return {'PASS_THROUGH'} # Let the next modal call handle the state change

        return {'PASS_THROUGH'}

    def execute(self, context):
        props = context.scene.collision_props
        if props.is_calculating: self.report({'WARNING'}, "Calculation already in progress."); return {'CANCELLED'}
        if bpy.ops.object.mode_set.poll(): bpy.ops.object.mode_set(mode='OBJECT')
        
        props.is_calculating = True # Set BEFORE initialize
        if not self.initialize_calculation(context):
            props.is_calculating = False # Reset if init failed
            self.report({'ERROR'}, "Initialization failed.")
            return {'CANCELLED'} # Don't start modal

        wm = context.window_manager
        self._timer = wm.event_timer_add(0.01, window=context.window) # Faster timer
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def cancel(self, context): # This is COLLISION_OT_calculate's own cancel method
        props = context.scene.collision_props
        if hasattr(self, '_timer') and self._timer:
            context.window_manager.event_timer_remove(self._timer)
            self._timer = None # Important to set to None after removal
        
        # Finalize, marking as cancelled, only if it was initialized
        if hasattr(self, '_is_initialized') and self._is_initialized:
             self.finalize_calculation(context, cancelled=True)
        else: # If not initialized, just ensure the flag is off
            props.is_calculating = False
        return {'CANCELLED'}

class COLLISION_OT_confirm_calculation(Operator):
    """Confirm orientation/location before collision calculation"""
    bl_idname = "collision.confirm_calculate"
    bl_label = "Calculate Collisions"
    bl_options = {'REGISTER', 'UNDO'}

    warning_message = StringProperty()

    def invoke(self, context, event):
        props = context.scene.collision_props
        acsf = props.ACSf_object
        acsm = props.ACSm_object
        warn = False
        msg_lines = []
        if acsf and acsm:
            # Check location difference
            loc_diff = (acsf.location - acsm.location).length
            if loc_diff > 1e-3:
                warn = True
                msg_lines.append("ACSf and ACSm locations differ; rotation axes may be incorrect.")
            # Check orientation difference
            q1 = acsf.matrix_world.to_quaternion()
            q2 = acsm.matrix_world.to_quaternion()
            quat_diff = q1.rotation_difference(q2)
            angle_deg = math.degrees(quat_diff.angle)
            if angle_deg > 0.1:
                warn = True
                msg_lines.append("ACSf and ACSm orientations differ; input rotations are relative.")
        if warn:
            self.warning_message = "\n".join(msg_lines)
            return context.window_manager.invoke_props_dialog(self)
        return self.execute(context)

    def draw(self, context):
        layout = self.layout
        for line in self.warning_message.split("\n"):
            layout.label(text=line)

    def execute(self, context):
        bpy.ops.collision.calculate('INVOKE_DEFAULT')
        return {'FINISHED'}

