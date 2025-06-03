#import necessary stuff
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


#We need a class for cancelling the computation
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
    
#We need a class for calculating the collisions
class COLLISION_OT_calculate(Operator):
    """Calculate object collisions based on rotations and translations"""
    bl_idname = "collision.calculate"
    bl_label = "Calculate Collisions"
    bl_options = {'REGISTER', 'UNDO'}
    
    #def initialize any properties used once you click go.

    def define_axes(self, props, ACSm_obj, ACSf_obj):
        import mathutils
        # X axis from ACSm, Z axis from ACSf, Y depends on rotation_mode_enum
        x_axis = ACSm_obj.matrix_world.col[0].to_3d().normalized()
        z_axis = ACSf_obj.matrix_world.col[2].to_3d().normalized()
        if props.rotation_mode_enum == 'INTUITIVE':
            y_axis = ACSm_obj.matrix_world.col[1].to_3d().normalized()
        else:
            # ISB uses floating y = cross(x,z)
            y_axis = x_axis.cross(z_axis).normalized()
        self.report({'INFO'}, f"Axes set. X={x_axis}, Y={y_axis}, Z={z_axis}")

    def define_translations(self, props, ACSf_obj, ACSm_obj):
        if props.translation_mode_enum == 'SIMPLE_ACSf':
            self.report({'INFO'}, "Using ACSf local axes for translations.")
        elif props.translation_mode_enum == 'ACSM_LOCAL_POST_ROT':
            self.report({'INFO'}, "Using ACSm local axes post-rotation.")
        else:
            self.report({'INFO'}, "MG_PRISM_HINGE mode not yet fully implemented.")

    def create_convex_hulls(self, prox_obj, dist_obj):
        import bpy
        # Example approach: just print placeholders
        self.report({'INFO'}, f"Created convex hulls for {prox_obj.name} and {dist_obj.name}")

    def calculate_bvh_trees(self, obj1, obj2):
        import mathutils
        # Example minimal BVH creation placeholder
        bvh1 = mathutils.bvhtree.BVHTree.FromObject(obj1, bpy.context.evaluated_depsgraph_get())
        bvh2 = mathutils.bvhtree.BVHTree.FromObject(obj2, bpy.context.evaluated_depsgraph_get())
        return bvh1, bvh2

    def calculate_collision(self, bvh1, bvh2):
        # Check overlap using BVH
        if bvh1 is None or bvh2 is None:
            return False
        overlap = bvh1.overlap(bvh2)
        return len(overlap) > 0

    def initialize_calculation(self, context):
        props = context.scene.collision_props
        self._prox_obj = props.proximal_object
        self._dist_obj = props.distal_object
        self._csv_results = []
        # Build rotation lists using numpy
        import numpy as np
        self._rx_values = np.arange(props.rot_x_min, props.rot_x_max + props.rot_x_inc, props.rot_x_inc)
        self._ry_values = np.arange(props.rot_y_min, props.rot_y_max + props.rot_y_inc, props.rot_y_inc)
        self._rz_values = np.arange(props.rot_z_min, props.rot_z_max + props.rot_z_inc, props.rot_z_inc)
        self._tx_values = np.arange(props.trans_x_min, props.trans_x_max + props.trans_x_inc, props.trans_x_inc)
        self._ty_values = np.arange(props.trans_y_min, props.trans_y_max + props.trans_y_inc, props.trans_y_inc)
        self._tz_values = np.arange(props.trans_z_min, props.trans_z_max + props.trans_z_inc, props.trans_z_inc)
        
        # Debug: print the actual ranges
        self.report({'INFO'}, f"RX: {self._rx_values}")
        self.report({'INFO'}, f"RY: {self._ry_values}")
        self.report({'INFO'}, f"RZ: {self._rz_values}")
        self.report({'INFO'}, f"TX: {self._tx_values}")
        self.report({'INFO'}, f"TY: {self._ty_values}")
        self.report({'INFO'}, f"TZ: {self._tz_values}")

        self._total_iterations = len(self._rx_values) * len(self._ry_values) * len(self._rz_values) * \
                                 len(self._tx_values) * len(self._ty_values) * len(self._tz_values)
        if self._total_iterations == 0:
            self.report({'WARNING'}, "Total iterations is zero. Check input ranges and increments.")
            # props.is_calculating will be set by caller, but good to note.
            # No point in proceeding with initialization if no work to do.
            return False

        self._completed_iterations = 0
        props.calculation_progress = 0.0
        # props.is_calculating = True # This will be set by the caller (execute method)

        # Initialize loop indices
        self._idx_tx = 0
        self._idx_ty = 0
        self._idx_tz = 0
        self._idx_rx = 0
        self._idx_ry = 0
        self._idx_rz = 0        # Store original transforms and target
        acsm_obj = props.ACSm_object
        if not acsm_obj:
            self.report({'ERROR'}, "ACSm object not set.")
            return False
          # Save world coordinates - this is crucial for parented objects
        self._saved_world_matrix = acsm_obj.matrix_world.copy()
        self._saved_world_loc = acsm_obj.matrix_world.translation.copy()
        self._saved_loc = acsm_obj.location.copy()  # Keep for restoration
        self._saved_rot = acsm_obj.rotation_euler.copy()  # Keep for restoration
        self._transform_target = acsm_obj        # Debug info about parent relationships
        if acsm_obj.parent:
            self.report({'INFO'}, f"ACSm is parented to {acsm_obj.parent.name}")
            self.report({'INFO'}, f"ACSm local rotation: {self._saved_rot}")
            self.report({'INFO'}, f"ACSm world location: {self._saved_world_loc}")
        else:
            self.report({'INFO'}, "ACSm has no parent")
            
        # Debug info about translation mode
        self.report({'INFO'}, f"Translation mode: {props.translation_mode_enum}")
        if props.translation_mode_enum == 'SIMPLE_ACSf':
            ACSf_obj = props.ACSf_object
            if ACSf_obj:
                acsf_x = ACSf_obj.matrix_world.col[0].to_3d().normalized()
                acsf_y = ACSf_obj.matrix_world.col[1].to_3d().normalized()
                acsf_z = ACSf_obj.matrix_world.col[2].to_3d().normalized()
                self.report({'INFO'}, f"ACSf local X-axis: {acsf_x}")
                self.report({'INFO'}, f"ACSf local Y-axis: {acsf_y}")
                self.report({'INFO'}, f"ACSf local Z-axis: {acsf_z}")

        # Keyframe ACSm at frame 0
        if self._transform_target:
            self._transform_target.keyframe_insert(data_path="location", frame=0)
            self._transform_target.keyframe_insert(data_path="rotation_euler", frame=0)
        
        self._is_initialized_for_modal = True
        self.report({'INFO'}, f"Total iterations: {self._total_iterations}")
        self.report({'INFO'}, "Initialization complete. Starting modal operation...")
        return True

    def finalize_calculation(self, context, cancelled=False):
        props = context.scene.collision_props

        # Restore original transforms
        if hasattr(self, '_transform_target') and hasattr(self, '_saved_loc') and hasattr(self, '_saved_rot'):
            if self._transform_target: # Check if target still valid (e.g. not deleted)
                try:
                    self._transform_target.location = self._saved_loc
                    self._transform_target.rotation_euler = self._saved_rot
                    bpy.context.view_layer.update()
                except ReferenceError:
                    self.report({'WARNING'}, "Could not restore transform, target object/bone no longer exists.")


        if props.export_to_csv and not cancelled and hasattr(self, '_csv_results') and self._csv_results:
            self.write_csv(props.export_path, self._csv_results)
        
        final_progress = props.calculation_progress # Default to current
        if hasattr(self, '_total_iterations') and self._total_iterations > 0:
            if not cancelled and hasattr(self, '_completed_iterations') and self._completed_iterations >= self._total_iterations:
                final_progress = 100.0
            # If cancelled, keep the progress at the point of cancellation.
            # If finished prematurely, it will also reflect the completed part.
            elif hasattr(self, '_completed_iterations'):
                 final_progress = (self._completed_iterations / self._total_iterations) * 100.0


        props.calculation_progress = final_progress
        props.is_calculating = False # Crucial: signal that calculation is no longer active

        report_msg = "Calculation finished."
        if cancelled:
            report_msg = "Calculation cancelled."
        elif hasattr(self, '_completed_iterations') and hasattr(self, '_total_iterations') and \
             self._total_iterations > 0 and self._completed_iterations < self._total_iterations:
            report_msg = f"Calculation ended prematurely after {self._completed_iterations}/{self._total_iterations} iterations."

        self.report({'INFO'}, report_msg)        # Clean up instance variables
        attrs_to_clear = [
            '_prox_obj', '_dist_obj', '_csv_results',
            '_rx_values', '_ry_values', '_rz_values',
            '_tx_values', '_ty_values', '_tz_values',
            '_total_iterations', '_completed_iterations',
            '_idx_tx', '_idx_ty', '_idx_tz', '_idx_rx', '_idx_ry', '_idx_rz',
            '_saved_loc', '_saved_rot', '_saved_world_matrix', '_saved_world_loc', '_transform_target',
            '_is_initialized_for_modal', '_timer' # Clear timer attribute as well
        ]
        for attr in attrs_to_clear:
            if hasattr(self, attr):
                delattr(self, attr)

    def update_progress(self):
        props = bpy.context.scene.collision_props
        props.calculation_progress = (self._completed_iterations / self._total_iterations) * 100.0
        # Trigger UI redraw for progress bar
        for area in bpy.context.screen.areas:
            if area.type == 'VIEW_3D':
                area.tag_redraw()
        self.report({'INFO'}, f"Progress updated to {props.calculation_progress:.1f}%")

    def write_csv(self, filepath, results):
        import csv
        # Minimal writing approach
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["rot_x", "rot_y", "rot_z", "trans_x", "trans_y", "trans_z", "Valid_pose"])
            for row in results:
                writer.writerow(row)
        self.report({'INFO'}, f"CSV written to: {filepath}")

    def execute(self, context):
        props = context.scene.collision_props

        if props.is_calculating:
            self.report({'WARNING'}, "A calculation is already in progress")
            return {'CANCELLED'}

        if bpy.ops.object.mode_set.poll():
            bpy.ops.object.mode_set(mode='OBJECT')
        
        props.is_calculating = True # Set early, initialize_calculation might return False
        if not self.initialize_calculation(context):
            props.is_calculating = False # Reset if initialization failed
            self.report({'ERROR'}, "Calculation initialization failed. Aborting.")
            # Call finalize to ensure any partial state is cleaned, though initialize should be careful
            self.finalize_calculation(context, cancelled=True) # Ensure cleanup
            return {'CANCELLED'}

        wm = context.window_manager
        self._timer = wm.event_timer_add(0.01, window=context.window) # Faster timer for responsiveness
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def process_batch(self, context):
        props = context.scene.collision_props
        # This check is vital. If COLLISION_OT_cancel sets this, we stop.
        if not props.is_calculating:
            return False 

        if not hasattr(self, '_is_initialized_for_modal') or not self._is_initialized_for_modal:
            self.report({'ERROR'}, "Batch processing called without proper initialization.")
            props.is_calculating = False # Critical to stop further processing
            return False

        try:
            iterations_per_batch = props.batch_size 
        except AttributeError:
            iterations_per_batch = 10 # A sensible default

        if iterations_per_batch <= 0: 
            iterations_per_batch = 1 # Ensure positive
        
        processed_in_batch = 0

        while processed_in_batch < iterations_per_batch:
            if not props.is_calculating: # Re-check for external cancellation within the batch loop
                return False

            # Check if all iterations are complete
            if self._idx_tx >= len(self._tx_values):
                return False # All calculations are done

            # Get current values based on indices
            tx = self._tx_values[self._idx_tx]
            ty = self._ty_values[self._idx_ty]
            tz = self._tz_values[self._idx_tz]
            rx = self._rx_values[self._idx_rx]
            ry = self._ry_values[self._idx_ry]
            rz = self._rz_values[self._idx_rz]            # --- Perform one iteration's work ---
            try:                # Work entirely in world coordinates to avoid parent/local space confusion
                # Start with the saved world location
                base_world_loc = self._saved_world_loc.copy()
                  # Apply translation based on translation mode
                ACSf_obj = props.ACSf_object
                if props.translation_mode_enum == 'SIMPLE_ACSf':
                    # Translate along ACSf's local axes
                    acsf_x_axis = ACSf_obj.matrix_world.col[0].to_3d().normalized()
                    acsf_y_axis = ACSf_obj.matrix_world.col[1].to_3d().normalized() 
                    acsf_z_axis = ACSf_obj.matrix_world.col[2].to_3d().normalized()
                    
                    # Convert to float to avoid numpy/Vector type mixing issues
                    translation_vector = (float(tx) * acsf_x_axis + 
                                        float(ty) * acsf_y_axis + 
                                        float(tz) * acsf_z_axis)
                    translated_world_loc = base_world_loc + translation_vector
                    
                elif props.translation_mode_enum == 'ACSM_LOCAL_POST_ROT':
                    # Translate along ACSm's local axes after applying rx, ry rotations
                    # First get the rotated ACSm matrix (before rz rotation around ACSf)
                    base_rotation_matrix = self._saved_world_matrix.to_3x3()
                    rx_mat = mathutils.Matrix.Rotation(math.radians(rx), 3, 'X')
                    ry_mat = mathutils.Matrix.Rotation(math.radians(ry), 3, 'Y')
                    local_rot = ry_mat @ rx_mat
                    rotated_acsm_matrix = base_rotation_matrix @ local_rot
                    
                    # Get the rotated ACSm axes
                    acsm_x_axis = rotated_acsm_matrix.col[0].normalized()
                    acsm_y_axis = rotated_acsm_matrix.col[1].normalized()
                    acsm_z_axis = rotated_acsm_matrix.col[2].normalized()
                    
                    # Convert to float to avoid numpy/Vector type mixing issues
                    translation_vector = (float(tx) * acsm_x_axis + 
                                        float(ty) * acsm_y_axis + 
                                        float(tz) * acsm_z_axis)
                    translated_world_loc = base_world_loc + translation_vector
                else:
                    # Default to world coordinates (or MG_PRISM_HINGE mode)
                    translated_world_loc = mathutils.Vector((
                        base_world_loc.x + tx, 
                        base_world_loc.y + ty, 
                        base_world_loc.z + tz
                    ))

                # Get ACSf z-axis and location (pivot) in world coordinates
                z_axis = ACSf_obj.matrix_world.col[2].to_3d().normalized()
                pivot = ACSf_obj.matrix_world.translation.copy()

                # First apply local rx and ry rotations to the original world matrix
                base_rotation_matrix = self._saved_world_matrix.to_3x3()
                rx_mat = mathutils.Matrix.Rotation(math.radians(rx), 3, 'X')
                ry_mat = mathutils.Matrix.Rotation(math.radians(ry), 3, 'Y')
                local_rot = ry_mat @ rx_mat
                
                # Apply local rotations to the base rotation
                rotated_matrix = base_rotation_matrix @ local_rot

                # Now apply rz rotation about ACSf z-axis at ACSf location
                rz_angle = math.radians(rz)
                rz_rot_matrix = mathutils.Matrix.Rotation(rz_angle, 4, z_axis)

                # Rotate the location around ACSf pivot
                rel_loc = translated_world_loc - pivot
                final_world_loc = rz_rot_matrix @ rel_loc + pivot
                
                # Rotate the orientation matrix by rz
                rz_rot_matrix_3x3 = rz_rot_matrix.to_3x3()
                final_rotation_matrix = rz_rot_matrix_3x3 @ rotated_matrix
                
                # Create the final 4x4 world matrix
                final_world_matrix = final_rotation_matrix.to_4x4()
                final_world_matrix.translation = final_world_loc
                
                # Set the world matrix directly
                self._transform_target.matrix_world = final_world_matrix

                bpy.context.view_layer.update()

                bvh_prox, bvh_dist = self.calculate_bvh_trees(self._prox_obj, self._dist_obj)
                collided = self.calculate_collision(bvh_prox, bvh_dist)
                valid_pose = not collided
                self._csv_results.append([rx, ry, rz, tx, ty, tz, valid_pose])

                if props.debug_mode or valid_pose:
                    frame = self._completed_iterations + 1 # Keyframes are 1-indexed
                    self._transform_target.keyframe_insert(data_path="location", frame=frame)
                    self._transform_target.keyframe_insert(data_path="rotation_euler", frame=frame)
            except ReferenceError: # Target object might have been deleted
                self.report({'ERROR'}, "Target object for transform is missing. Stopping calculation.")
                props.is_calculating = False
                return False


            self._completed_iterations += 1
            self.update_progress() # This contains the self.report call for progress
            processed_in_batch += 1
            # --- End of one iteration's work ---

            # Advance indices (innermost first)
            self._idx_rz += 1
            if self._idx_rz >= len(self._rz_values):
                self._idx_rz = 0
                self._idx_ry += 1
                if self._idx_ry >= len(self._ry_values):
                    self._idx_ry = 0
                    self._idx_rx += 1
                    if self._idx_rx >= len(self._rx_values):
                        self._idx_rx = 0
                        self._idx_tz += 1
                        if self._idx_tz >= len(self._tz_values):
                            self._idx_tz = 0
                            self._idx_ty += 1
                            if self._idx_ty >= len(self._ty_values):
                                self._idx_ty = 0
                                self._idx_tx += 1
                                if self._idx_tx >= len(self._tx_values):
                                    # All iterations completed in this batch
                                    # The check at the start of the while loop will handle termination.
                                    break # Exit while processed_in_batch < iterations_per_batch
        
        # If loop finished due to all iterations done (idx_tx went out of bounds)
        if self._idx_tx >= len(self._tx_values):
            return False # All calculations are done
            
        return True # Batch limit reached, more work to do

    def modal(self, context, event):
        props = context.scene.collision_props

        if not props.is_calculating:
            # Calculation was stopped (either finished or cancelled by user/error)
            # Finalize_calculation should have been called by the logic that set is_calculating to False.
            # We just need to clean up the modal operator itself.
            if hasattr(self, '_timer') and self._timer: # Ensure timer is removed
                context.window_manager.event_timer_remove(self._timer)
                # self._timer = None # finalize_calculation will delattr this
            # Ensure finalize is called if it somehow wasn't (e.g. external cancel didn't trigger it)
            # This check helps prevent multiple finalizations if already handled.
            if hasattr(self, '_is_initialized_for_modal'): 
                 self.finalize_calculation(context, cancelled=True) # Assume cancelled if is_calculating is false unexpectedly

            return {'FINISHED'}

        if event.type == 'TIMER':
            if not self.process_batch(context):
                # process_batch returned False, meaning all work is done or an error occurred/cancelled.
                # Determine if it was a true finish or an internal cancellation/error
                was_cancelled_internally = True 
                if hasattr(self, '_completed_iterations') and hasattr(self, '_total_iterations') and \
                   self._total_iterations > 0 and self._completed_iterations >= self._total_iterations:
                    was_cancelled_internally = False
                
                self.finalize_calculation(context, cancelled=was_cancelled_internally)
                # props.is_calculating is now False, so the next modal call will enter the 'if not props.is_calculating' block
                # and return {'FINISHED'} after cleaning the timer.
                # No need to return from here, let the standard path handle it.
        
        elif event.type == 'ESC' and event.value == 'PRESS': # Allow ESC to cancel
            self.report({'INFO'}, "ESC pressed, cancelling calculation.")
            # No need to call finalize_calculation here, COLLISION_OT_cancel or the props.is_calculating check will handle it.
            # Setting props.is_calculating = False is the main way to signal cancellation.
            # The COLLISION_OT_cancel operator does this.
            # For direct ESC handling in modal:
            if hasattr(self, '_timer') and self._timer:
                context.window_manager.event_timer_remove(self._timer)
            self.finalize_calculation(context, cancelled=True)
            return {'CANCELLED'}


        return {'PASS_THROUGH'}

    def cancel(self, context): # This is COLLISION_OT_calculate's own cancel method
        # Called by Blender if the operator is cancelled externally, or if modal returns {'CANCELLED'}
        props = context.scene.collision_props
        
        # Ensure timer is removed if it exists
        if hasattr(self, '_timer') and self._timer:
            context.window_manager.event_timer_remove(self._timer)
            # self._timer = None # finalize_calculation will delattr

        # Finalize the calculation, marking it as cancelled.
        # Check if it was initialized to avoid errors if cancelled before/after full run.
        if hasattr(self, '_is_initialized_for_modal'):
            self.finalize_calculation(context, cancelled=True)
        else:
            # If not even initialized, ensure is_calculating is false.
            props.is_calculating = False 
            self.report({'INFO'}, "Calculation cancelled (was not fully initialized or already finalized).")

        return {'CANCELLED'}
