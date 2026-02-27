"""
operators.py — All Blender operators for BlenderROMFinder.

Operators:
  - COLLISION_OT_cancel            — cancel a running calculation
  - COLLISION_OT_calculate         — main ROM calculation (parallel workers)
  - COLLISION_OT_confirm_calculation — pre-flight ACSf/ACSm validation dialog
  - COLLISION_OT_find_min_x_distance — iterative X-axis distance finder
"""

import bpy
import json
import math
import os
import time
from pathlib import Path

from bpy.props import StringProperty
from bpy.types import Operator
from mathutils import Matrix, Vector

from .collision import CollisionDetector
from .processor import (
    ROMProcessor,
    start_headless_workers_async,
)


# ---------------------------------------------------------------------------
# Cancel
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Main calculation operator (formerly COLLISION_OT_calculate_parallel)
# ---------------------------------------------------------------------------

class COLLISION_OT_calculate(Operator):
    """Calculate ROM — parallel headless workers with fallback to in-process"""
    bl_idname = "collision.calculate"
    bl_label = "Calculate ROM"
    bl_options = {'REGISTER', 'UNDO'}
    bl_description = "Run collision detection across all pose combinations"

    _timer = None
    _processor = None

    # -- Helpers -----------------------------------------------------------

    def _cleanup_modal_state(self, context):
        props = context.scene.collision_props
        if self._processor:
            self._processor.reset_acsm_to_initial()
            self._processor.cleanup()
        if self._timer:
            context.window_manager.event_timer_remove(self._timer)
            self._timer = None
        props.is_calculating = False

    def _tag_redraw(self, context):
        try:
            for win in context.window_manager.windows:
                try:
                    for area in win.screen.areas:
                        area.tag_redraw()
                except Exception:
                    continue
        except Exception:
            try:
                bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
            except Exception:
                pass

    # -- Modal -------------------------------------------------------------

    def modal(self, context, event):
        props = context.scene.collision_props

        if not props.is_calculating or (event.type == 'ESC' and event.value == 'PRESS'):
            self.cancel_processing(context)
            return {'FINISHED'}

        if event.type == 'TIMER':
            if self._processor:
                # Keyframe creation phase
                if self._processor.is_creating_keyframes:
                    still_creating = self._processor.process_keyframe_batch(props)
                    if not still_creating:
                        self.finish_processing(context)
                        return {'FINISHED'}
                    return {'PASS_THROUGH'}

                # Headless-worker phase
                if hasattr(self, '_using_workers') and self._using_workers:
                    any_active = False
                    total_processed = 0
                    total_total = 0

                    for w in self._workers:
                        q = w['queue']
                        while not q.empty():
                            msg_type, payload, wid = q.get()
                            w['last_output_time'] = time.time()
                            if msg_type == 'progress':
                                w['last_progress'] = int(payload.get('processed', 0))
                                w['total'] = int(payload.get('total', w.get('total', w['total'])))
                            elif msg_type == 'result':
                                try:
                                    self._processor.merge_worker_payload(payload)
                                except Exception as e:
                                    print(f"Error merging payload from worker {wid}: {e}")
                                w['done'] = True
                            elif msg_type == 'error':
                                print(f"Worker {wid} error: {payload}")
                                w['done'] = True
                            elif msg_type == 'log':
                                if props.debug_mode:
                                    print(f"[Worker {wid}] {payload}")
                                props.time_remaining = f"Worker {wid}: {payload[:80]}"

                        p = w['proc']
                        rc = p.poll()
                        if rc is None and not w.get('done'):
                            any_active = True
                        elif rc is not None and rc != 0 and not w.get('done'):
                            print(f"Worker {w['worker_id']} exited with code {rc}")
                            w['done'] = True

                        total_processed += w.get('last_progress', 0)
                        total_total += w.get('total', 0)

                    pct = (total_processed / total_total * 100.0) if total_total > 0 else 0.0
                    props.calculation_progress = pct

                    if self._processor.start_time and total_processed > 0:
                        elapsed = time.time() - self._processor.start_time
                        pps = total_processed / elapsed if elapsed > 0 else 0
                        remaining = max(0, self._processor.total_poses - total_processed)
                        remaining_time = remaining / pps if pps > 0 else 0
                        mins, secs = divmod(int(remaining_time), 60)
                        props.time_remaining = f"Time remaining: {mins:02d}:{secs:02d}"

                    try:
                        self._tag_redraw(context)
                    except Exception:
                        pass

                    if not any_active:
                        if total_processed == 0 and len(self._workers) > 0:
                            if getattr(props, 'headless_workers_only', False):
                                print("No worker results; aborting (workers-only mode)")
                                self._cleanup_workers()
                                self.cancel_processing(context)
                                self.report({'ERROR'}, "Headless workers failed (workers-only mode). Aborting.")
                                return {'CANCELLED'}

                            print("No worker results; falling back to in-process processing")
                            props.time_remaining = "Falling back to in-process processing"
                            self._cleanup_workers()
                            return {'PASS_THROUGH'}

                        # Workers finished — export + keyframe
                        self._handle_completion(context)
                        self._cleanup_workers()

                        if props.visualize_collisions and self._processor.valid_poses:
                            if self._processor.start_keyframe_creation(props):
                                return {'PASS_THROUGH'}

                        self.finish_processing(context)
                        return {'FINISHED'}

                    # Watchdog
                    for w in self._workers:
                        to = getattr(props, 'headless_worker_timeout', 1800)
                        if (time.time() - w.get('last_output_time', 0) > to
                                and w.get('proc') and w['proc'].poll() is None):
                            print(f"Worker {w['worker_id']} timed out after {to}s, terminating")
                            try:
                                w['proc'].terminate()
                            except Exception:
                                pass
                            w['done'] = True
                            self._using_workers = False
                            break

                    return {'PASS_THROUGH'}

                # In-process batch phase
                batch_size = max(1, props.batch_size)
                still_processing = self._processor.process_batch(batch_size)

                props.calculation_progress = self._processor.get_progress()
                if self._processor.start_time and self._processor.processed_poses > 0:
                    elapsed = time.time() - self._processor.start_time
                    pps = self._processor.processed_poses / elapsed
                    remaining = self._processor.total_poses - self._processor.processed_poses
                    remaining_time = remaining / pps if pps > 0 else 0
                    mins, secs = divmod(int(remaining_time), 60)
                    props.time_remaining = f"Time remaining: {mins:02d}:{secs:02d}"

                try:
                    self._tag_redraw(context)
                except Exception:
                    pass

                if not still_processing:
                    self._handle_completion(context)

                    if props.visualize_collisions and self._processor.valid_poses:
                        if self._processor.start_keyframe_creation(props):
                            return {'PASS_THROUGH'}

                    self.finish_processing(context)
                    return {'FINISHED'}

        return {'PASS_THROUGH'}

    # -- Completion helpers ------------------------------------------------

    def _handle_completion(self, context):
        """Export CSV and report summary."""
        props = context.scene.collision_props
        try:
            if self._processor.export_results(props):
                elapsed = time.time() - self._processor.start_time
                mins, secs = divmod(int(elapsed), 60)
                pps = self._processor.processed_poses / elapsed if elapsed > 0 else 0
                self.report(
                    {'INFO'},
                    f"Complete! {len(self._processor.valid_poses):,} valid poses "
                    f"in {mins:02d}:{secs:02d} ({pps:.0f} poses/sec)")
                print(
                    f"Collision detection complete! {len(self._processor.valid_poses):,} valid poses "
                    f"in {mins:02d}:{secs:02d} ({pps:.0f} poses/sec)")
            else:
                self.report({'WARNING'}, "Export had issues")
        except Exception as e:
            self.report({'ERROR'}, f"Export failed: {e}")

    def _cleanup_workers(self):
        """Wait for workers and clean up temp files."""
        if not hasattr(self, '_workers'):
            return
        for w in self._workers:
            try:
                w['proc'].wait(timeout=0.1)
            except Exception:
                pass
            try:
                pf = w.get('props_file')
                if pf and os.path.exists(pf):
                    os.unlink(pf)
            except Exception:
                pass
        self._using_workers = False
        self._workers = []

    # -- Execute -----------------------------------------------------------

    def execute(self, context):
        props = context.scene.collision_props

        if props.is_calculating:
            self.report({'WARNING'}, "Calculation already in progress")
            return {'CANCELLED'}

        if bpy.ops.object.mode_set.poll():
            bpy.ops.object.mode_set(mode='OBJECT')

        props.is_calculating = True
        props.calculation_progress = 0.0
        props.time_remaining = "Initializing..."

        self._processor = ROMProcessor()

        try:
            self._processor.initialize(props)
            self._processor.start_processing()

            blend_path = bpy.data.filepath
            worker_script = Path(__file__).with_name('worker_headless.py')
            self._using_workers = False
            self._workers = []

            if not blend_path or bpy.data.is_dirty:
                self.report({'INFO'}, "Please save the .blend file to run the calculation.")
                try:
                    bpy.ops.wm.save_as_mainfile('INVOKE_DEFAULT')
                except Exception:
                    pass
                props.is_calculating = False
                return {'CANCELLED'}

            try:
                if blend_path and os.path.exists(worker_script) and self._processor.total_poses > 0:
                    worker_count = max(1, min(props.headless_worker_count, self._processor.total_poses))

                    import base64
                    props_dict = {
                        'proximal_object': props.proximal_object.name if props.proximal_object else None,
                        'distal_object': props.distal_object.name if props.distal_object else None,
                        'ACSf_object': props.ACSf_object.name if props.ACSf_object else None,
                        'ACSm_object': props.ACSm_object.name if props.ACSm_object else None,
                        'ACSm_bone': props.ACSm_bone,
                        'rotation_mode_enum': props.rotation_mode_enum,
                        'rot_x_min': props.rot_x_min, 'rot_x_max': props.rot_x_max, 'rot_x_inc': props.rot_x_inc,
                        'rot_y_min': props.rot_y_min, 'rot_y_max': props.rot_y_max, 'rot_y_inc': props.rot_y_inc,
                        'rot_z_min': props.rot_z_min, 'rot_z_max': props.rot_z_max, 'rot_z_inc': props.rot_z_inc,
                        'trans_x_min': props.trans_x_min, 'trans_x_max': props.trans_x_max, 'trans_x_inc': props.trans_x_inc,
                        'trans_y_min': props.trans_y_min, 'trans_y_max': props.trans_y_max, 'trans_y_inc': props.trans_y_inc,
                        'trans_z_min': props.trans_z_min, 'trans_z_max': props.trans_z_max, 'trans_z_inc': props.trans_z_inc,
                        'use_convex_hull': bool(props.use_convex_hull_optimization),
                        'use_proxy_collision': bool(props.use_proxy_collision),
                        'proxy_decimate_ratio': float(props.proxy_decimate_ratio),
                        'penetration_sample_count': int(props.penetration_sample_count),
                        'only_export_valid_poses': bool(props.only_export_valid_poses),
                    }
                    props_json_b64 = __import__('base64').b64encode(
                        json.dumps(props_dict).encode('utf-8')).decode('ascii')

                    try:
                        self._workers = start_headless_workers_async(
                            blend_path, worker_script,
                            self._processor.total_poses,
                            worker_count=worker_count,
                            chunk_size=props.headless_chunk_size,
                            blender_exec=(props.headless_worker_exec or None),
                            timeout=props.headless_worker_timeout,
                            props_b64=props_json_b64,
                        )
                        self._using_workers = True
                        props.time_remaining = "Launching headless workers..."
                        print(f"Launched {len(self._workers)} headless workers")
                    except Exception as exc:
                        print(f"Failed to launch headless workers: {exc}")
                        self._using_workers = False
                        self._workers = []
                else:
                    print("Headless workers unavailable (unsaved file or worker script missing)")
            except Exception as exc:
                print(f"Error starting workers: {exc}")

            wm = context.window_manager
            self._timer = wm.event_timer_add(0.05, window=context.window)
            wm.modal_handler_add(self)
            return {'RUNNING_MODAL'}

        except Exception as e:
            self.report({'ERROR'}, f"Failed to start processing: {e}")
            props.is_calculating = False
            return {'CANCELLED'}

    def cancel_processing(self, context):
        if self._processor:
            self._processor.cancel()
        if hasattr(self, '_workers') and self._workers:
            for w in self._workers:
                try:
                    p = w.get('proc')
                    if p and p.poll() is None:
                        p.terminate()
                except Exception:
                    pass
                try:
                    pf = w.get('props_file')
                    if pf and os.path.exists(pf):
                        os.unlink(pf)
                except Exception:
                    pass
        self._cleanup_modal_state(context)
        self.report({'INFO'}, "Processing cancelled")

    def finish_processing(self, context):
        self._cleanup_modal_state(context)
        props = context.scene.collision_props
        props.calculation_progress = 100.0
        try:
            self._tag_redraw(context)
        except Exception:
            pass

    def cancel(self, context):
        self.cancel_processing(context)
        return {'CANCELLED'}


# ---------------------------------------------------------------------------
# Pre-flight confirmation dialog
# ---------------------------------------------------------------------------

class COLLISION_OT_confirm_calculation(Operator):
    """Confirm orientation/location before collision calculation"""
    bl_idname = "collision.confirm_calculate"
    bl_label = "Calculate ROM"
    bl_options = {'REGISTER', 'UNDO'}

    warning_message: StringProperty()

    def invoke(self, context, event):
        props = context.scene.collision_props
        acsf = props.ACSf_object
        acsm = props.ACSm_object
        warn = False
        msg_lines = []

        if acsf and acsm:
            loc_diff = (acsf.location - acsm.location).length
            if loc_diff > 1e-3:
                warn = True
                msg_lines.append("ACSf and ACSm locations differ; rotation axes may be incorrect.")
            q1 = acsf.matrix_world.to_quaternion()
            q2 = acsm.matrix_world.to_quaternion()
            angle_deg = math.degrees(q1.rotation_difference(q2).angle)
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


# ---------------------------------------------------------------------------
# Minimum X-distance finder (uses CollisionDetector)
# ---------------------------------------------------------------------------

class COLLISION_OT_find_min_x_distance(Operator):
    """Move ACSm along its local X-axis until distal bone no longer collides with proximal bone"""
    bl_idname = "collision.find_min_x_distance"
    bl_label = "Find Minimum X Distance"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.collision_props

        if not props.proximal_object:
            self.report({'ERROR'}, "Proximal object not set")
            return {'CANCELLED'}
        if not props.distal_object:
            self.report({'ERROR'}, "Distal object not set")
            return {'CANCELLED'}
        if not props.ACSm_object:
            self.report({'ERROR'}, "ACSm object not set")
            return {'CANCELLED'}

        prox_obj = props.proximal_object
        dist_obj = props.distal_object
        ACSm_obj = props.ACSm_object
        increment = props.min_x_distance_increment

        ACSm_bone_name = getattr(props, 'ACSm_bone', None)
        use_ACSm_bone = (ACSm_obj and ACSm_obj.type == 'ARMATURE'
                         and ACSm_bone_name and ACSm_bone_name != 'NONE')

        # Store initial state
        if use_ACSm_bone:
            pose_bone = ACSm_obj.pose.bones.get(ACSm_bone_name)
            if not pose_bone:
                self.report({'ERROR'}, f"ACSm bone '{ACSm_bone_name}' not found")
                return {'CANCELLED'}
            initial_matrix = pose_bone.matrix.copy()
        else:
            initial_matrix = ACSm_obj.matrix_local.copy()

        # Use CollisionDetector for a simple full-BVH check (no hull/sampling needed)
        detector = CollisionDetector()
        detector.initialize(prox_obj, dist_obj,
                            use_convex_hull=False,
                            penetration_sample_count=100)  # minimal sampling

        # Check initial collision
        context.view_layer.update()
        if not detector.check(dist_obj.matrix_world):
            detector.cleanup()
            self.report({'INFO'}, "Objects are not currently colliding. No movement needed.")
            return {'FINISHED'}

        if abs(increment) < 1e-7:
            detector.cleanup()
            self.report({'ERROR'}, "Increment cannot be zero")
            return {'CANCELLED'}

        total_distance = 0.0
        max_distance = 100.0
        max_iterations = int(max_distance / abs(increment)) + 1

        for iteration in range(max_iterations):
            total_distance += increment

            local_x_axis = initial_matrix.to_3x3() @ Vector((1, 0, 0))
            translation = Matrix.Translation(local_x_axis.normalized() * total_distance)
            new_matrix = translation @ initial_matrix

            if use_ACSm_bone:
                pose_bone = ACSm_obj.pose.bones.get(ACSm_bone_name)
                pose_bone.matrix = new_matrix
            else:
                ACSm_obj.matrix_local = new_matrix

            context.view_layer.update()

            if not detector.check(dist_obj.matrix_world):
                props.min_x_distance_result = total_distance
                props.min_x_distance_found = True
                detector.cleanup()
                self.report({'INFO'}, f"No collision at X distance: {total_distance:.6f} "
                            f"(after {iteration + 1} iterations)")
                return {'FINISHED'}

        # Restore initial position
        if use_ACSm_bone:
            pose_bone = ACSm_obj.pose.bones.get(ACSm_bone_name)
            pose_bone.matrix = initial_matrix
        else:
            ACSm_obj.matrix_local = initial_matrix
        context.view_layer.update()

        props.min_x_distance_found = False
        detector.cleanup()
        self.report({'WARNING'}, f"Could not find non-colliding position within {max_distance} units")
        return {'CANCELLED'}
