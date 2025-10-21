import bpy
import bmesh
import mathutils
import math
import csv
import os
import time
from mathutils import Matrix, Vector
from bpy.types import Operator
from bpy.props import StringProperty

# Import existing calculation functions
from .poseCalculations import (
    calculate_pose_for_isb_standard_mode,
    calculate_pose_for_intuitive_mode,
    calculate_pose_for_mg_hinge_mode
)

class OptimizedROMProcessor:
    """Optimized ROM processor using the same collision detection as original but with better batching"""

    def __init__(self):
        self.pose_combinations = []
        self.total_poses = 0
        self.processed_poses = 0
        self.valid_poses = []
        self.csv_data = []
        self.is_cancelled = False
        self.start_time = None
        
        # Use the same collision detection approach as original
        self.prox_obj = None
        self.dist_obj = None
        self.acsm_obj = None
        self.prox_bvh = None
        self.acsm_initial_local = None
        self.operational_mode = None

    def initialize(self, props):
        """Initialize using the same approach as the original operators.py"""
        print("🚀 Initializing optimized ROM processor...")
        
        # Get objects (same as original)
        self.prox_obj = props.proximal_object
        self.dist_obj = props.distal_object
        acsf_obj = props.ACSf_object
        self.acsm_obj = props.ACSm_object

        if not all([self.prox_obj, self.dist_obj, acsf_obj, self.acsm_obj]):
            raise ValueError("Missing required objects")

        # Get ACSm initial matrix (same as original)
        acsm_bone_name = getattr(props, 'ACSm_bone', None)
        use_acsm_bone = self.acsm_obj.type == 'ARMATURE' and acsm_bone_name
        
        if use_acsm_bone:
            pose_bone = self.acsm_obj.pose.bones.get(acsm_bone_name)
            if pose_bone:
                self.acsm_initial_local = pose_bone.matrix.copy()
            else:
                raise ValueError(f"ACSm bone '{acsm_bone_name}' not found")
        else:
            self.acsm_initial_local = self.acsm_obj.matrix_local.copy()

        # Create proximal BVH once (same as original)
        self.prox_bvh = self._create_bvh_tree(self.prox_obj)
        if not self.prox_bvh:
            raise ValueError("Failed to create proximal BVH")

        # Store operational mode
        self.operational_mode = props.rotation_mode_enum
        # Keep props reference for translation calculations that need ACSf/ACSm objects
        self.props = props

        # Generate pose combinations
        self.pose_combinations = self._generate_pose_combinations(props)
        self.total_poses = len(self.pose_combinations)

        # Initialize CSV data
        self.csv_data = [["rot_x", "rot_y", "rot_z", "trans_x", "trans_y", "trans_z", "Valid_pose"]]
        self.processed_poses = 0
        self.valid_poses = []

        print(f"✅ Initialized for {self.total_poses:,} poses")
        return True

    def _create_bvh_tree(self, obj, transform_matrix=None):
        """Use the EXACT same BVH creation as original operators.py"""
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

    def _generate_pose_combinations(self, props):
        """Generate pose combinations (same logic as original)"""
        def gen_range(min_val, max_val, inc):
            if inc <= 0:
                return [round(min_val, 6)]
            
            vals = []
            current = min_val
            eps = inc * 1e-6
            while current <= max_val + eps:
                vals.append(round(current, 6))
                current += inc
            
            # Remove duplicates
            seen = set()
            unique_vals = []
            for v in vals:
                if v not in seen:
                    seen.add(v)
                    unique_vals.append(v)
            return unique_vals

        # Generate ranges (same as original)
        rot_x_range = gen_range(props.rot_x_min, props.rot_x_max, props.rot_x_inc)
        rot_y_range = gen_range(props.rot_y_min, props.rot_y_max, props.rot_y_inc)
        rot_z_range = gen_range(props.rot_z_min, props.rot_z_max, props.rot_z_inc)
        trans_x_range = gen_range(props.trans_x_min, props.trans_x_max, props.trans_x_inc)
        trans_y_range = gen_range(props.trans_y_min, props.trans_y_max, props.trans_y_inc)
        trans_z_range = gen_range(props.trans_z_min, props.trans_z_max, props.trans_z_inc)

        # Generate all combinations
        combinations = []
        for rx in rot_x_range:
            for ry in rot_y_range:
                for rz in rot_z_range:
                    for tx in trans_x_range:
                        for ty in trans_y_range:
                            for tz in trans_z_range:
                                combinations.append((rx, ry, rz, tx, ty, tz))

        return combinations

    def process_batch(self, batch_size=1000):
        """Process a batch of poses using the same logic as original"""
        if self.is_cancelled or self.processed_poses >= self.total_poses:
            return False

        batch_start = self.processed_poses
        batch_end = min(batch_start + batch_size, self.total_poses)
        batch_poses = self.pose_combinations[batch_start:batch_end]

        print(f"⚙️  Processing batch {batch_start:,} to {batch_end:,} ({len(batch_poses):,} poses)")
        
        valid_in_batch = 0
        for rx, ry, rz, tx, ty, tz in batch_poses:
            if self.is_cancelled:
                break

            # Calculate pose matrix (same as original)
            if self.operational_mode == 'ISB_STANDARD':
                pose_matrix = calculate_pose_for_isb_standard_mode(
                    rx, ry, rz, tx, ty, tz, None, self.acsm_initial_local, acsm_obj=self.acsm_obj)
            elif self.operational_mode == 'INTUITIVE':
                pose_matrix = calculate_pose_for_intuitive_mode(
                    rx, ry, rz, tx, ty, tz, None, self.acsm_initial_local, acsm_obj=self.acsm_obj)
            elif self.operational_mode == 'MG_HINGE':
                pose_matrix = calculate_pose_for_mg_hinge_mode(
                    rx, ry, rz, tx, ty, tz, self.props, self.acsm_initial_local)
            else:
                continue

            # Apply pose to ACSm (same as original)
            if self.acsm_obj.type == 'ARMATURE':
                acsm_bone_name = getattr(bpy.context.scene.collision_props, 'ACSm_bone', None)
                if acsm_bone_name:
                    pose_bone = self.acsm_obj.pose.bones.get(acsm_bone_name)
                    if pose_bone:
                        pose_bone.matrix = pose_matrix
            else:
                self.acsm_obj.matrix_local = pose_matrix

            # Update scene (same as original)
            bpy.context.view_layer.update()

            # Check collision using EXACT same method as original
            collision = self._check_collision_original_method()
            
            # Add to CSV data
            self.csv_data.append([rx, ry, rz, tx, ty, tz, 0 if collision else 1])
            
            if not collision:
                self.valid_poses.append({
                    'pose_params': (rx, ry, rz, tx, ty, tz),
                    'pose_matrix': pose_matrix.copy(),
                    'rx': rx, 'ry': ry, 'rz': rz,
                    'tx': tx, 'ty': ty, 'tz': tz
                })
                valid_in_batch += 1

        self.processed_poses = batch_end
        
        # Progress update
        progress = (self.processed_poses / self.total_poses) * 100
        elapsed = time.time() - self.start_time if self.start_time else 0
        if elapsed > 0:
            poses_per_second = self.processed_poses / elapsed
            remaining_time = (self.total_poses - self.processed_poses) / poses_per_second if poses_per_second > 0 else 0
            print(f"📈 Progress: {progress:.1f}% - {poses_per_second:.0f} poses/sec - Valid: {valid_in_batch} (Total: {len(self.valid_poses)})")
            print(f"⏰ Est. remaining: {remaining_time/60:.1f} minutes")

        return self.processed_poses < self.total_poses and not self.is_cancelled

    def _check_collision_original_method(self):
        """Use the EXACT same collision detection as the original operators.py"""
        # Create BVH for distal object in its current pose
        dist_bvh = self._create_bvh_tree(self.dist_obj)
        if not dist_bvh:
            return True  # Assume collision on error

        # Check overlap (same as original)
        overlaps = self.prox_bvh.overlap(dist_bvh)
        return len(overlaps) > 0

    def start_processing(self):
        """Start the processing timer"""
        self.start_time = time.time()
        print(f"🚀 Starting optimized processing of {self.total_poses:,} poses")

    def get_progress(self):
        """Get current progress percentage"""
        if self.total_poses == 0:
            return 100.0
        return (self.processed_poses / self.total_poses) * 100.0

    def cancel(self):
        """Cancel processing"""
        self.is_cancelled = True
        print("🛑 Processing cancelled")

    def reset_acsm_to_initial(self):
        """Reset ACSm to initial state"""
        if self.acsm_obj and self.acsm_initial_local:
            if self.acsm_obj.type == 'ARMATURE':
                acsm_bone_name = getattr(bpy.context.scene.collision_props, 'ACSm_bone', None)
                if acsm_bone_name:
                    pose_bone = self.acsm_obj.pose.bones.get(acsm_bone_name)
                    if pose_bone:
                        pose_bone.matrix = self.acsm_initial_local
            else:
                self.acsm_obj.matrix_local = self.acsm_initial_local
            
            bpy.context.view_layer.update()

    def export_results(self, props):
        """Export results (same as original)"""
        if not self.csv_data or len(self.csv_data) <= 1:
            return False

        success = True

        # Export CSV
        if props.export_to_csv and props.export_path:
            try:
                filepath = bpy.path.abspath(props.export_path)
                dirpath = os.path.dirname(filepath)
                if dirpath:
                    os.makedirs(dirpath, exist_ok=True)
                
                with open(filepath, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerows(self.csv_data)
                
                print(f"📊 Results exported to: {filepath}")
                
            except Exception as e:
                print(f"❌ Error exporting CSV: {e}")
                success = False

        # Create animation keyframes if requested
        if props.visualize_collisions and self.valid_poses:
            try:
                self._create_animation_keyframes(props)
                print(f"🎬 Created {len(self.valid_poses)} animation keyframes")
            except Exception as e:
                print(f"❌ Error creating animation: {e}")
                success = False

        return success

    def _create_animation_keyframes(self, props):
        """Create animation keyframes with ULTRA-optimized bulk operations"""
        if not self.valid_poses:
            return
            
        print(f"🎬 Creating {len(self.valid_poses):,} animation keyframes (optimized)...")
        
        acsm_bone_name = getattr(props, 'ACSm_bone', None)
        use_acsm_bone = self.acsm_obj and self.acsm_obj.type == 'ARMATURE' and acsm_bone_name

        if use_acsm_bone:
            target = self.acsm_obj.pose.bones.get(acsm_bone_name)
        else:
            target = self.acsm_obj

        if not target:
            return

        # CRITICAL: Disable auto-keyframing and other heavy operations
        scene = bpy.context.scene
        original_use_keyframe_insert_auto = scene.tool_settings.use_keyframe_insert_auto
        scene.tool_settings.use_keyframe_insert_auto = False
        
        try:
            # Set keyframe at frame 0 (initial state)
            target.keyframe_insert(data_path="location", frame=0)
            target.keyframe_insert(data_path="rotation_euler", frame=0)

            # OPTIMIZED: Large batches with minimal scene updates
            total_poses = len(self.valid_poses)
            batch_size = 1000  # Much larger batches for speed
            
            for batch_start in range(0, total_poses, batch_size):
                batch_end = min(batch_start + batch_size, total_poses)
                batch_poses = self.valid_poses[batch_start:batch_end]
                
                # Progress feedback
                progress = (batch_end / total_poses) * 100
                print(f"⚡ Keyframe batch: {progress:.1f}% ({batch_end:,}/{total_poses:,})")
                
                # Process entire batch without scene updates
                for i, pose_data in enumerate(batch_poses):
                    frame = batch_start + i + 1
                    
                    # Apply pose matrix
                    if use_acsm_bone:
                        target.matrix = pose_data['pose_matrix']
                    else:
                        target.matrix_local = pose_data['pose_matrix']

                    # Set all custom properties
                    target["input_rot_x"] = pose_data['rx']
                    target["input_rot_y"] = pose_data['ry']
                    target["input_rot_z"] = pose_data['rz']
                    target["input_trans_x"] = pose_data['tx']
                    target["input_trans_y"] = pose_data['ty']
                    target["input_trans_z"] = pose_data['tz']

                    # Insert all keyframes
                    target.keyframe_insert(data_path="location", frame=frame)
                    target.keyframe_insert(data_path="rotation_euler", frame=frame)
                    target.keyframe_insert(data_path='["input_rot_x"]', frame=frame)
                    target.keyframe_insert(data_path='["input_rot_y"]', frame=frame)
                    target.keyframe_insert(data_path='["input_rot_z"]', frame=frame)
                    target.keyframe_insert(data_path='["input_trans_x"]', frame=frame)
                    target.keyframe_insert(data_path='["input_trans_y"]', frame=frame)
                    target.keyframe_insert(data_path='["input_trans_z"]', frame=frame)
                
                # Only update scene once per large batch
                bpy.context.view_layer.update()

        finally:
            # Restore original settings
            scene.tool_settings.use_keyframe_insert_auto = original_use_keyframe_insert_auto

        print(f"✅ Animation keyframes complete: {len(self.valid_poses):,} poses")
        total_poses = len(self.valid_poses)
        
        for batch_start in range(0, total_poses, batch_size):
            batch_end = min(batch_start + batch_size, total_poses)
            batch_poses = self.valid_poses[batch_start:batch_end]
            
            # Progress feedback for large datasets
            if total_poses > 1000:
                progress = ((batch_end) / total_poses) * 100
                print(f"⏳ Keyframe progress: {progress:.1f}% ({batch_end:,}/{total_poses:,})")
            
            # Process batch
            for i, pose_data in enumerate(batch_poses):
                frame = batch_start + i + 1
                try:
                    # Apply pose matrix
                    if use_acsm_bone:
                        target.matrix = pose_data['pose_matrix']
                    else:
                        target.matrix_local = pose_data['pose_matrix']

                    # Set custom properties (only essential ones)
                    target["input_rot_x"] = pose_data['rx']
                    target["input_rot_y"] = pose_data['ry']
                    target["input_rot_z"] = pose_data['rz']
                    target["input_trans_x"] = pose_data['tx']
                    target["input_trans_y"] = pose_data['ty']
                    target["input_trans_z"] = pose_data['tz']

                    # Insert keyframes (reduced set for performance)
                    target.keyframe_insert(data_path="location", frame=frame)
                    target.keyframe_insert(data_path="rotation_euler", frame=frame)
                    
                    # Only insert custom property keyframes for every 10th frame to reduce overhead
                    if frame % 10 == 1 or total_poses < 1000:
                        target.keyframe_insert(data_path='["input_rot_x"]', frame=frame)
                        target.keyframe_insert(data_path='["input_rot_y"]', frame=frame)
                        target.keyframe_insert(data_path='["input_rot_z"]', frame=frame)
                        target.keyframe_insert(data_path='["input_trans_x"]', frame=frame)
                        target.keyframe_insert(data_path='["input_trans_y"]', frame=frame)
                        target.keyframe_insert(data_path='["input_trans_z"]', frame=frame)

                except Exception as e:
                    if batch_start == 0 and i < 10:  # Only log first few errors
                        print(f"⚠️  Error creating keyframe {frame}: {e}")
                    continue
            
            # Update scene less frequently for better performance
            if batch_end % (batch_size * 5) == 0 or batch_end == total_poses:
                bpy.context.view_layer.update()

        print(f"✅ Animation keyframes complete: {len(self.valid_poses):,} poses")

    def export_results(self, props):
        """Export results to CSV and create animation if requested"""
        if not self.csv_data or len(self.csv_data) <= 1:
            return False

        success = True

        # Export CSV
        if props.export_to_csv and props.export_path:
            try:
                filepath = bpy.path.abspath(props.export_path)
                dirpath = os.path.dirname(filepath)
                if dirpath:
                    os.makedirs(dirpath, exist_ok=True)
                
                with open(filepath, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerows(self.csv_data)
                
                print(f"📊 Results exported to: {filepath}")
                
            except Exception as e:
                print(f"❌ Error exporting CSV: {e}")
                success = False

        # Create animation keyframes if requested
        if props.visualize_collisions and self.valid_poses:
            try:
                self._create_animation_keyframes(props)
                print(f"🎬 Created {len(self.valid_poses)} animation keyframes")
            except Exception as e:
                print(f"❌ Error creating animation: {e}")
                success = False

        return success

    def _create_animation_keyframes(self, props):
        """Create animation keyframes for valid poses"""
        acsm_obj = props.ACSm_object
        acsm_bone_name = getattr(props, 'ACSm_bone', None)
        use_acsm_bone = acsm_obj and acsm_obj.type == 'ARMATURE' and acsm_bone_name

        if use_acsm_bone:
            target = acsm_obj.pose.bones.get(acsm_bone_name)
        else:
            target = acsm_obj

        if not target:
            return

        # Set keyframe at frame 0 (initial state)
        target.keyframe_insert(data_path="location", frame=0)
        target.keyframe_insert(data_path="rotation_euler", frame=0)

        # Create keyframes for valid poses
        for frame, pose_data in enumerate(self.valid_poses, start=1):
            try:
                # Apply pose matrix
                if use_acsm_bone:
                    target.matrix = pose_data['pose_matrix']
                else:
                    target.matrix_local = pose_data['pose_matrix']

                # Set custom properties for input parameters
                target["input_rot_x"] = pose_data['rx']
                target["input_rot_y"] = pose_data['ry']
                target["input_rot_z"] = pose_data['rz']
                target["input_trans_x"] = pose_data['tx']
                target["input_trans_y"] = pose_data['ty']
                target["input_trans_z"] = pose_data['tz']

                # Insert keyframes
                target.keyframe_insert(data_path="location", frame=frame)
                target.keyframe_insert(data_path="rotation_euler", frame=frame)
                target.keyframe_insert(data_path='["input_rot_x"]', frame=frame)
                target.keyframe_insert(data_path='["input_rot_y"]', frame=frame)
                target.keyframe_insert(data_path='["input_rot_z"]', frame=frame)
                target.keyframe_insert(data_path='["input_trans_x"]', frame=frame)
                target.keyframe_insert(data_path='["input_trans_y"]', frame=frame)
                target.keyframe_insert(data_path='["input_trans_z"]', frame=frame)

            except Exception as e:
                print(f"⚠️  Error creating keyframe {frame}: {e}")
                continue


class COLLISION_OT_calculate_parallel(Operator):
    """Optimized collision calculation using efficient batching"""
    bl_idname = "collision.calculate_parallel"
    bl_label = "Calculate Optimized (High Performance)"
    bl_options = {'REGISTER', 'UNDO'}
    bl_description = "Optimized collision detection using efficient batching and the same collision logic as original"

    _timer = None
    _processor = None

    def modal(self, context, event):
        props = context.scene.collision_props

        # Check for cancellation
        if not props.is_calculating or (event.type == 'ESC' and event.value == 'PRESS'):
            self.cancel_processing(context)
            return {'FINISHED'}

        if event.type == 'TIMER':
            if self._processor:
                # Process a batch
                batch_size = max(1, props.batch_size * 10)  # Use larger batches
                still_processing = self._processor.process_batch(batch_size)
                
                # Update progress
                progress = self._processor.get_progress()
                props.calculation_progress = progress
                
                # Update time remaining
                if self._processor.start_time and self._processor.processed_poses > 0:
                    elapsed = time.time() - self._processor.start_time
                    poses_per_second = self._processor.processed_poses / elapsed
                    remaining_poses = self._processor.total_poses - self._processor.processed_poses
                    remaining_time = remaining_poses / poses_per_second if poses_per_second > 0 else 0
                    mins, secs = divmod(int(remaining_time), 60)
                    props.time_remaining = f"Time remaining: {mins:02d}:{secs:02d}"

                if not still_processing:
                    # Processing complete
                    try:
                        if self._processor.export_results(props):
                            elapsed = time.time() - self._processor.start_time
                            mins, secs = divmod(int(elapsed), 60)
                            poses_per_second = self._processor.processed_poses / elapsed if elapsed > 0 else 0
                            
                            self.report({'INFO'}, 
                                f"Optimized processing complete! "
                                f"Found {len(self._processor.valid_poses):,} valid poses "
                                f"in {mins:02d}:{secs:02d} "
                                f"({poses_per_second:.0f} poses/sec)")
                        else:
                            self.report({'WARNING'}, "Processing complete but export had issues")
                    except Exception as e:
                        self.report({'ERROR'}, f"Export failed: {e}")

                    self.finish_processing(context)
                    return {'FINISHED'}

        return {'PASS_THROUGH'}

    def execute(self, context):
        props = context.scene.collision_props
        
        if props.is_calculating:
            self.report({'WARNING'}, "Calculation already in progress")
            return {'CANCELLED'}

        # Ensure we're in object mode
        if bpy.ops.object.mode_set.poll():
            bpy.ops.object.mode_set(mode='OBJECT')

        # Initialize
        props.is_calculating = True
        props.calculation_progress = 0.0
        props.time_remaining = "Initializing..."
        
        self._processor = OptimizedROMProcessor()
        
        try:
            # Initialize processor
            self.report({'INFO'}, "Initializing optimized ROM processor...")
            self._processor.initialize(props)
            self._processor.start_processing()
            
            # Start modal timer for batch processing
            wm = context.window_manager
            self._timer = wm.event_timer_add(0.05, window=context.window)  # 20 FPS update
            wm.modal_handler_add(self)
            
            return {'RUNNING_MODAL'}
            
        except Exception as e:
            self.report({'ERROR'}, f"Failed to start optimized processing: {e}")
            props.is_calculating = False
            return {'CANCELLED'}

    def cancel_processing(self, context):
        """Cancel the processing"""
        props = context.scene.collision_props
        
        if self._processor:
            self._processor.cancel()
            self._processor.reset_acsm_to_initial()
        
        if self._timer:
            context.window_manager.event_timer_remove(self._timer)
            self._timer = None
        
        props.is_calculating = False
        self.report({'INFO'}, "Optimized processing cancelled")

    def finish_processing(self, context):
        """Clean up after processing completion"""
        props = context.scene.collision_props
        
        if self._processor:
            self._processor.reset_acsm_to_initial()
        
        if self._timer:
            context.window_manager.event_timer_remove(self._timer)
            self._timer = None
        
        props.is_calculating = False
        props.calculation_progress = 100.0

    def cancel(self, context):
        """Operator cancel method"""
        self.cancel_processing(context)
        return {'CANCELLED'}
