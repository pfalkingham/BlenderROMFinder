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

    def execute(self, context):
        """Entry point for the collision calculation operator."""
        # 1. Validate input and initialize state
        if not self.validate_and_initialize(context):
            return {'CANCELLED'}

        props = context.scene.collision_props
        frame = 1  # Start keyframing valid poses at frame 1 (frame 0 is original pose)
        valid_poses = 0
        total_poses = 0
        cancelled = False

        batch_size = getattr(props, 'batch_size', 10)
        wm = bpy.context.window_manager
        wm.progress_begin(0, self._total_iterations)
        start_time = time.time()
        for pose_idx, pose_params in enumerate(self.generate_pose_sequence(context)):
            # Check for user cancellation
            if hasattr(props, 'is_calculating') and not props.is_calculating:
                cancelled = True
                break
            # Set frame for this pose
            bpy.context.scene.frame_set(frame)
            # Apply pose
            self.apply_pose(context, pose_params)
            # Check collision
            is_valid = not self.check_collision(context)
            # Record result (keyframes if valid)
            self.record_result(context, pose_params, is_valid)
            if is_valid:
                valid_poses += 1
            total_poses += 1
            frame += 1
            self._completed_iterations += 1
            # Update progress every batch_size iterations
            if self._completed_iterations % batch_size == 0 or self._completed_iterations == self._total_iterations:
                props.calculation_progress = 100.0 * self._completed_iterations / max(1, self._total_iterations)
                elapsed = time.time() - start_time
                if self._completed_iterations > 0:
                    est_total = elapsed / self._completed_iterations * self._total_iterations
                    est_remaining = max(0, est_total - elapsed)
                    props.time_remaining = f"{int(est_remaining // 60)}m {int(est_remaining % 60)}s"
                else:
                    props.time_remaining = ""
                wm.progress_update(self._completed_iterations)
                # Allow Blender to update UI
                bpy.context.view_layer.update()
        wm.progress_end()
        props.calculation_progress = 100.0
        props.is_calculating = False
        props.time_remaining = ""

        # Write CSV
        self.write_csv(context)
        # Restore original pose (for user convenience)
        self.restore_original_pose(context)

        # Clean up cached meshes if not using convex hulls
        if hasattr(self, '_proximal_mesh') and self._proximal_mesh:
            bpy.data.meshes.remove(self._proximal_mesh)
            self._proximal_mesh = None
        if hasattr(self, '_distal_mesh') and self._distal_mesh:
            bpy.data.meshes.remove(self._distal_mesh)
            self._distal_mesh = None

        elapsed = time.time() - self._start_time
        if cancelled:
            self.report({'WARNING'}, f"Calculation cancelled. {valid_poses} valid / {total_poses} poses checked in {elapsed:.2f}s.")
        else:
            self.report({'INFO'}, f"Calculation complete: {valid_poses} valid / {total_poses} poses checked in {elapsed:.2f}s.")
        return {'FINISHED'}

    # --- Helper Functions ---

    def validate_and_initialize(self, context):
        """Validate user input and initialize calculation state."""
        props = context.scene.collision_props
        errors = []

        # Check mesh selection
        if not props.proximal_object:
            errors.append("Proximal object not selected.")
        if not props.distal_object:
            errors.append("Distal object not selected.")
        if props.proximal_object == props.distal_object:
            errors.append("Proximal and distal objects must be different.")

        # Check axis/joint selection (ACSf/ACSm can be blank, that's OK)
        # If present, must be valid objects
        if props.ACSf and not isinstance(props.ACSf, bpy.types.Object):
            errors.append("Fixed (proximal) axis (ACSf) is not a valid object.")
        if props.ACSm and not isinstance(props.ACSm, bpy.types.Object):
            errors.append("Mobile (distal) axis (ACSm) is not a valid object.")

        # Enforce that ACSm (distal axis) is specified for anatomical relevance
        if not props.ACSm:
            errors.append("Mobile (distal) axis (ACSm) must be specified for anatomical relevance.")

        # If a bone is selected, ACSf/ACSm must be an armature
        if props.rotational_bone and (not props.ACSf or props.ACSf.type != 'ARMATURE'):
            errors.append("Fixed (proximal) bone selected, but ACSf is not an armature.")
        if props.rotational_bone_2 and (not props.ACSm or props.ACSm.type != 'ARMATURE'):
            errors.append("Mobile (distal) bone selected, but ACSm is not an armature.")

        # Check rotation increments (should not be zero)
        if props.rot_x_inc == 0 or props.rot_y_inc == 0 or props.rot_z_inc == 0:
            errors.append("Rotation increments must not be zero.")

        # Check that min <= max for all rotation axes
        for axis in ['x', 'y', 'z']:
            if getattr(props, f'rot_{axis}_min') > getattr(props, f'rot_{axis}_max'):
                errors.append(f"Rotation {axis.upper()} min is greater than max.")

        # If any errors, report and return False
        if errors:
            for err in errors:
                self.report({'ERROR'}, err)
            return False

        # Store original pose for restoration later
        self._original_pose = {}
        distal_obj = props.distal_object
        self._original_pose['location'] = distal_obj.location.copy()
        self._original_pose['rotation_euler'] = distal_obj.rotation_euler.copy()
        self._original_pose['rotation_mode'] = distal_obj.rotation_mode

        # If using ACSm/ACSf armatures and bones, store their original pose as well
        if props.ACSm and props.ACSm.type == 'ARMATURE' and props.rotational_bone_2:
            pose_bone = props.ACSm.pose.bones.get(props.rotational_bone_2)
            if pose_bone:
                self._original_pose['ACSm_bone_location'] = pose_bone.location.copy()
                self._original_pose['ACSm_bone_rotation'] = pose_bone.rotation_euler.copy()
                self._original_pose['ACSm_bone_rotation_mode'] = pose_bone.rotation_mode
        if props.ACSf and props.ACSf.type == 'ARMATURE' and props.rotational_bone:
            pose_bone = props.ACSf.pose.bones.get(props.rotational_bone)
            if pose_bone:
                self._original_pose['ACSf_bone_location'] = pose_bone.location.copy()
                self._original_pose['ACSf_bone_rotation'] = pose_bone.rotation_euler.copy()
                self._original_pose['ACSf_bone_rotation_mode'] = pose_bone.rotation_mode

        # Keyframe the original pose at frame 0
        bpy.context.scene.frame_set(0)
        distal_obj.keyframe_insert(data_path="location")
        distal_obj.keyframe_insert(data_path="rotation_euler")
        # Keyframe ACSm bone if relevant
        if props.ACSm and props.ACSm.type == 'ARMATURE' and props.rotational_bone_2:
            pose_bone = props.ACSm.pose.bones.get(props.rotational_bone_2)
            if pose_bone:
                pose_bone.keyframe_insert(data_path="location")
                pose_bone.keyframe_insert(data_path="rotation_euler")
        # Keyframe ACSf bone if relevant
        if props.ACSf and props.ACSf.type == 'ARMATURE' and props.rotational_bone:
            pose_bone = props.ACSf.pose.bones.get(props.rotational_bone)
            if pose_bone:
                pose_bone.keyframe_insert(data_path="location")
                pose_bone.keyframe_insert(data_path="rotation_euler")

        # Check if we're using hulls, and if so generate them
        use_convex_hull = getattr(props, "use_convex_hull", False)
        proximal_obj = props.proximal_object
        distal_obj = props.distal_object

        # Cache convex hull meshes in local space if needed
        if use_convex_hull:
            self._proximal_hull_mesh = self.generate_convex_hull_mesh(proximal_obj)
            self._distal_hull_mesh = self.generate_convex_hull_mesh(distal_obj)
        else:
            self._proximal_mesh = proximal_obj.to_mesh()
            self._distal_mesh = distal_obj.to_mesh()

        # Calculate total iterations for progress reporting
        # Build rotation and translation ranges for progress calculation
        self._rot_x_range = np.arange(props.rot_x_min, props.rot_x_max + props.rot_x_inc, props.rot_x_inc)
        self._rot_y_range = np.arange(props.rot_y_min, props.rot_y_max + props.rot_y_inc, props.rot_y_inc)
        self._rot_z_range = np.arange(props.rot_z_min, props.rot_z_max + props.rot_z_inc, props.rot_z_inc)
        self._trans_x_range = np.arange(props.trans_x_min, props.trans_x_max + props.trans_x_inc, props.trans_x_inc)
        self._trans_y_range = np.arange(props.trans_y_min, props.trans_y_max + props.trans_y_inc, props.trans_y_inc)
        self._trans_z_range = np.arange(props.trans_z_min, props.trans_z_max + props.trans_z_inc, props.trans_z_inc)
        self._total_iterations = len(self._rot_x_range) * len(self._rot_y_range) * len(self._rot_z_range) * \
                                len(self._trans_x_range) * len(self._trans_y_range) * len(self._trans_z_range)
        self._completed_iterations = 0
        props.calculation_progress = 0.0
        props.is_calculating = True
        props.time_remaining = ""

        # Initialize any other state needed for the calculation here
        # (e.g., progress counters, CSV data, etc.)
        self._csv_data = [[
            'rot_x', 'rot_y', 'rot_z',
            'trans_x', 'trans_y', 'trans_z',
            'valid_pose']
        ]
        self._valid_pose_count = 0
        self._total_pose_count = 0
        self._start_time = time.time()

        return True

    def generate_convex_hull_mesh(self, obj):
        """Return a mesh (bpy.types.Mesh) that is the convex hull of the given object's mesh, in local space."""
        mesh = obj.to_mesh()
        bm = bmesh.new()
        bm.from_mesh(mesh)
        bmesh.ops.convex_hull(bm, input=bm.verts)
        hull_mesh = bpy.data.meshes.new(name=f"{obj.name}_ConvexHull")
        bm.to_mesh(hull_mesh)
        bm.free()
        bpy.data.meshes.remove(mesh)
        return hull_mesh

    def get_transformed_vertices(self, mesh, matrix):
        """Return a list of mesh vertices transformed by the given matrix."""
        return [matrix @ v.co for v in mesh.vertices]



    def generate_pose_sequence(self, context):
        """Yield all rotation pose parameter sets in the desired order (neutral->max, then neutral->min for each axis)."""
        props = context.scene.collision_props
        # Build rotation and translation ranges
        rot_x_range = np.arange(props.rot_x_min, props.rot_x_max + props.rot_x_inc, props.rot_x_inc)
        rot_y_range = np.arange(props.rot_y_min, props.rot_y_max + props.rot_y_inc, props.rot_y_inc)
        rot_z_range = np.arange(props.rot_z_min, props.rot_z_max + props.rot_z_inc, props.rot_z_inc)
        trans_x_range = np.arange(props.trans_x_min, props.trans_x_max + props.trans_x_inc, props.trans_x_inc)
        trans_y_range = np.arange(props.trans_y_min, props.trans_y_max + props.trans_y_inc, props.trans_y_inc)
        trans_z_range = np.arange(props.trans_z_min, props.trans_z_max + props.trans_z_inc, props.trans_z_inc)

        def sweep_axis(axis_range):
            zero = 0
            max_vals = [v for v in axis_range if v > zero]
            min_vals = [v for v in axis_range if v < zero]
            return [zero] + max_vals + min_vals

        x_sweep = sweep_axis(rot_x_range)
        y_sweep = sweep_axis(rot_y_range)
        z_sweep = sweep_axis(rot_z_range)
        for x in x_sweep:
            for y in y_sweep:
                for z in z_sweep:
                    for tx in trans_x_range:
                        for ty in trans_y_range:
                            for tz in trans_z_range:
                                yield {
                                    'rot_x': x,
                                    'rot_y': y,
                                    'rot_z': z,
                                    'trans_x': tx,
                                    'trans_y': ty,
                                    'trans_z': tz
                                }
        
    def get_anatomical_axes(self, context):
        """
        Compute anatomical axes for rotation based on ACSf and ACSm.
        Returns a dict with 'x', 'y', 'z' as normalized mathutils.Vector in world space.
        """
        props = context.scene.collision_props
        ACSf = props.ACSf
        ACSm = props.ACSm

        # Z: ACSf's Z axis in world space (flexion/extension)
        Z = ACSf.matrix_world.to_3x3() @ mathutils.Vector((0, 0, 1))
        # X: ACSm's X axis in world space (long axis of distal segment)
        X = ACSm.matrix_world.to_3x3() @ mathutils.Vector((1, 0, 0))
        # Y: perpendicular to both (ad/abduction axis)
        Y = Z.cross(X)
        if Y.length == 0:
            # Fallback: use ACSm's local Y in world space
            Y = ACSm.matrix_world.to_3x3() @ mathutils.Vector((0, 1, 0))
        return {
            'x': X.normalized(),
            'y': Y.normalized(),
            'z': Z.normalized()
        }

    def apply_pose(self, context, pose_params):
        """Apply the given pose (rotations) to the distal object using ACSm and ACSf as reference axes."""
        props = context.scene.collision_props
        distal_obj = props.distal_object
        axes = self.get_anatomical_axes(context)
        origin = props.ACSm.matrix_world.translation

        # Convert degrees to radians
        rx = math.radians(pose_params['rot_x'])
        ry = math.radians(pose_params['rot_y'])
        rz = math.radians(pose_params['rot_z'])

        # Build rotation matrices (order: Z, then Y, then X)
        Rz = mathutils.Matrix.Rotation(rz, 4, axes['z'])
        Ry = mathutils.Matrix.Rotation(ry, 4, axes['y'])
        Rx = mathutils.Matrix.Rotation(rx, 4, axes['x'])

        # Compose the rotation: R = Rz @ Ry @ Rx
        R = Rz @ Ry @ Rx

        # Apply rotation about ACSm origin
        # Move to origin, rotate, move back
        T_neg = mathutils.Matrix.Translation(-origin)
        T_pos = mathutils.Matrix.Translation(origin)
        new_matrix = T_pos @ R @ T_neg @ distal_obj.matrix_world

        # Apply translation from pose_params
        translation = mathutils.Vector((
            pose_params.get('trans_x', 0.0),
            pose_params.get('trans_y', 0.0),
            pose_params.get('trans_z', 0.0)
        ))
        new_matrix.translation += translation

        distal_obj.matrix_world = new_matrix
        pass

    def check_collision(self, context):
        """Check for collision between the proximal and distal objects using BVH and optional convex hull pre-check."""
        props = context.scene.collision_props
        use_convex_hull = getattr(props, "use_convex_hull", False)
        proximal_obj = props.proximal_object
        distal_obj = props.distal_object

        if use_convex_hull:
            # Transform hull vertices to world space for current pose
            prox_verts = self.get_transformed_vertices(self._proximal_hull_mesh, proximal_obj.matrix_world)
            dist_verts = self.get_transformed_vertices(self._distal_hull_mesh, distal_obj.matrix_world)
            # Build temporary meshes for BVH
            prox_temp = bpy.data.meshes.new("ProxTemp")
            dist_temp = bpy.data.meshes.new("DistTemp")
            prox_temp.from_pydata(prox_verts, [], [f.vertices for f in self._proximal_hull_mesh.polygons])
            dist_temp.from_pydata(dist_verts, [], [f.vertices for f in self._distal_hull_mesh.polygons])
            prox_bvh = mathutils.bvhtree.BVHTree.FromMesh(prox_temp)
            dist_bvh = mathutils.bvhtree.BVHTree.FromMesh(dist_temp)
            overlap = prox_bvh.overlap(dist_bvh)
            bpy.data.meshes.remove(prox_temp)
            bpy.data.meshes.remove(dist_temp)
            return bool(overlap)
        else:
            # Use full mesh, transform to world space for current pose
            dist_verts = self.get_transformed_vertices(self._distal_mesh, distal_obj.matrix_world)
            prox_verts = self.get_transformed_vertices(self._proximal_mesh, proximal_obj.matrix_world)
            prox_temp = bpy.data.meshes.new("ProxTemp")
            dist_temp = bpy.data.meshes.new("DistTemp")
            prox_temp.from_pydata(prox_verts, [], [f.vertices for f in self._proximal_mesh.polygons])
            dist_temp.from_pydata(dist_verts, [], [f.vertices for f in self._distal_mesh.polygons])
            prox_bvh = mathutils.bvhtree.BVHTree.FromMesh(prox_temp)
            dist_bvh = mathutils.bvhtree.BVHTree.FromMesh(dist_temp)
            overlap = prox_bvh.overlap(dist_bvh)
            bpy.data.meshes.remove(prox_temp)
            bpy.data.meshes.remove(dist_temp)
            return bool(overlap)

    def record_result(self, context, pose_params, is_valid):
        """Record the result (CSV, keyframe if valid, etc)."""
        # Record the pose parameters and validity in the CSV data
        row = [
            pose_params.get('rot_x', 0.0),
            pose_params.get('rot_y', 0.0),
            pose_params.get('rot_z', 0.0),
            pose_params.get('trans_x', 0.0),
            pose_params.get('trans_y', 0.0),
            pose_params.get('trans_z', 0.0),
            int(is_valid)
        ]
        self._csv_data.append(row)
        self._total_pose_count += 1
        if is_valid:
            self._valid_pose_count += 1
            props = context.scene.collision_props
            distal_obj = props.distal_object
            # Insert keyframes for the distal object
            distal_obj.keyframe_insert(data_path="location")
            distal_obj.keyframe_insert(data_path="rotation_euler")
            # If distal object is an armature and a bone is specified, keyframe the bone
            if props.ACSm and props.ACSm.type == 'ARMATURE' and props.rotational_bone_2:
                pose_bone = props.ACSm.pose.bones.get(props.rotational_bone_2)
                if pose_bone:
                    pose_bone.keyframe_insert(data_path="location")
                    pose_bone.keyframe_insert(data_path="rotation_euler")

    def restore_original_pose(self, context):
        """Restore the distal object to its original pose after calculation."""
        # Restore the distal object's original pose
        props = context.scene.collision_props
        distal_obj = props.distal_object
        orig = self._original_pose
        distal_obj.location = orig['location'].copy()
        distal_obj.rotation_euler = orig['rotation_euler'].copy()
        distal_obj.rotation_mode = orig['rotation_mode']
        # Restore ACSm bone if relevant
        if props.ACSm and props.ACSm.type == 'ARMATURE' and props.rotational_bone_2:
            pose_bone = props.ACSm.pose.bones.get(props.rotational_bone_2)
            if pose_bone:
                pose_bone.location = orig.get('ACSm_bone_location', pose_bone.location).copy()
                pose_bone.rotation_euler = orig.get('ACSm_bone_rotation', pose_bone.rotation_euler).copy()
                pose_bone.rotation_mode = orig.get('ACSm_bone_rotation_mode', pose_bone.rotation_mode)
        # Restore ACSf bone if relevant
        if props.ACSf and props.ACSf.type == 'ARMATURE' and props.rotational_bone:
            pose_bone = props.ACSf.pose.bones.get(props.rotational_bone)
            if pose_bone:
                pose_bone.location = orig.get('ACSf_bone_location', pose_bone.location).copy()
                pose_bone.rotation_euler = orig.get('ACSf_bone_rotation', pose_bone.rotation_euler).copy()
                pose_bone.rotation_mode = orig.get('ACSf_bone_rotation_mode', pose_bone.rotation_mode)

    def write_csv(self, context):
        props = context.scene.collision_props
        csv_path = getattr(props, 'export_path', None)
        if not csv_path:
            self.report({'ERROR'}, "No CSV output path specified.")
            return
        try:
            with open(bpy.path.abspath(csv_path), 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerows(self._csv_data)
            self.report({'INFO'}, f"Results written to {csv_path}")
        except Exception as e:
            self.report({'ERROR'}, f"Failed to write CSV: {e}")

    # Add more helpers as needed for modularity.



