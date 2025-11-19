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
        
        # Keyframe creation state
        self.keyframe_batch_index = 0
        self.keyframe_batch_size = 500
        self.is_creating_keyframes = False
        self.keyframe_target = None

    def initialize(self, props):
        """Initialize using the same approach as the original operators.py"""
        print(" Initializing optimized ROM processor...")
        
        # Get objects (same as original)
        self.prox_obj = props.proximal_object
        self.dist_obj = props.distal_object
        acsf_obj = props.ACSf_object
        self.acsm_obj = props.ACSm_object

        if not all([self.prox_obj, self.dist_obj, acsf_obj, self.acsm_obj]):
            raise ValueError("Missing required objects")

        # Get ACSm initial matrix (same as original)
        acsm_bone_name = getattr(props, 'ACSm_bone', None)
        # Check if bone name is valid (not 'NONE' placeholder)
        use_acsm_bone = (self.acsm_obj.type == 'ARMATURE' and 
                         acsm_bone_name and 
                         acsm_bone_name != 'NONE')
        
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

        print(f"  Processing batch {batch_start:,} to {batch_end:,} ({len(batch_poses):,} poses)")
        
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
            print(f" Progress: {progress:.1f}% - {poses_per_second:.0f} poses/sec - Valid: {valid_in_batch} (Total: {len(self.valid_poses)})")
            print(f" Est. remaining: {remaining_time/60:.1f} minutes")

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
        print(f" Starting optimized processing of {self.total_poses:,} poses")

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
                
                print(f" Results exported to: {filepath}")
                
            except Exception as e:
                print(f"❌ Error exporting CSV: {e}")
                success = False

        # Create animation keyframes if requested
        if props.visualize_collisions and self.valid_poses:
            try:
                self._create_animation_keyframes(props)
                print(f" Created {len(self.valid_poses)} animation keyframes")
            except Exception as e:
                print(f"❌ Error creating animation: {e}")
                success = False

        return success

    def _create_animation_keyframes(self, props):
        """Create animation keyframes with ULTRA-optimized bulk operations"""
        if not self.valid_poses:
            return
            
        print(f" Creating {len(self.valid_poses):,} animation keyframes (optimized)...")
        
        # Reset UI for keyframe progress
        props.calculation_progress = 0.0
        props.time_remaining = "Creating animation keyframes..."
        
        acsm_bone_name = getattr(props, 'ACSm_bone', None)
        # Check if bone name is valid (not 'NONE' placeholder)
        use_acsm_bone = (self.acsm_obj and 
                         self.acsm_obj.type == 'ARMATURE' and 
                         acsm_bone_name and 
                         acsm_bone_name != 'NONE')

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

            # OPTIMIZED: Smaller batches for more frequent UI updates
            total_poses = len(self.valid_poses)
            batch_size = 500  # Smaller batches for more frequent progress updates
            
            for batch_start in range(0, total_poses, batch_size):
                batch_end = min(batch_start + batch_size, total_poses)
                batch_poses = self.valid_poses[batch_start:batch_end]
                
                # Update UI progress
                progress = (batch_end / total_poses) * 100
                props.calculation_progress = progress
                props.time_remaining = f"Creating keyframes: {batch_end:,}/{total_poses:,} ({progress:.1f}%)"
                
                # CRITICAL: Force UI redraw to show progress
                for window in bpy.context.window_manager.windows:
                    for area in window.screen.areas:
                        if area.type == 'VIEW_3D':
                            area.tag_redraw()
                
                # Force Blender to process events and update UI
                bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
                
                # Console feedback with flush
                print(f" Keyframe progress: {progress:.1f}% ({batch_end:,}/{total_poses:,})", flush=True)
                
                # Process entire batch without scene updates
                for i, pose_data in enumerate(batch_poses):
                    frame = batch_start + i + 1
                    
                    try:
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
                    
                    except Exception as e:
                        if batch_start == 0 and i < 10:  # Only log first few errors
                            print(f"⚠️  Error creating keyframe {frame}: {e}")
                        continue
                
                # Only update scene once per batch
                bpy.context.view_layer.update()

        finally:
            # Restore original settings
            scene.tool_settings.use_keyframe_insert_auto = original_use_keyframe_insert_auto
            # Reset UI
            props.calculation_progress = 100.0

        print(f"✅ Animation keyframes complete: {len(self.valid_poses):,} poses")

    def start_keyframe_creation(self, props):
        """Initialize keyframe creation process"""
        if not self.valid_poses:
            return False
            
        print(f"🎬 Starting keyframe creation for {len(self.valid_poses):,} poses...")
        
        # Reset state
        self.keyframe_batch_index = 0
        self.is_creating_keyframes = True
        
        # Determine target
        acsm_bone_name = getattr(props, 'ACSm_bone', None)
        use_acsm_bone = (self.acsm_obj and 
                         self.acsm_obj.type == 'ARMATURE' and 
                         acsm_bone_name and 
                         acsm_bone_name != 'NONE')

        if use_acsm_bone:
            self.keyframe_target = self.acsm_obj.pose.bones.get(acsm_bone_name)
        else:
            self.keyframe_target = self.acsm_obj

        if not self.keyframe_target:
            print("❌ No valid target for keyframes")
            return False

        # Disable auto-keyframing
        scene = bpy.context.scene
        self.original_use_keyframe_insert_auto = scene.tool_settings.use_keyframe_insert_auto
        scene.tool_settings.use_keyframe_insert_auto = False
        
        # Set initial keyframe at frame 0
        self.keyframe_target.keyframe_insert(data_path="location", frame=0)
        self.keyframe_target.keyframe_insert(data_path="rotation_euler", frame=0)
        
        # Update UI
        props.calculation_progress = 0.0
        props.time_remaining = "Creating animation keyframes..."
        
        return True

    def process_keyframe_batch(self, props):
        """Process one batch of keyframes - returns True if more batches remain"""
        if not self.is_creating_keyframes or not self.keyframe_target:
            return False
            
        total_poses = len(self.valid_poses)
        batch_start = self.keyframe_batch_index * self.keyframe_batch_size
        
        if batch_start >= total_poses:
            # All done
            self.finish_keyframe_creation(props)
            return False
            
        batch_end = min(batch_start + self.keyframe_batch_size, total_poses)
        batch_poses = self.valid_poses[batch_start:batch_end]
        
        # Update UI
        progress = (batch_end / total_poses) * 100
        props.calculation_progress = progress
        props.time_remaining = f"Creating keyframes: {batch_end:,}/{total_poses:,} ({progress:.1f}%)"
        
        # Console feedback
        print(f"Keyframe batch: {progress:.1f}% ({batch_end:,}/{total_poses:,})", flush=True)
        
        # Determine if using bone
        use_acsm_bone = hasattr(self.keyframe_target, 'bone')
        
        # Process batch
        for i, pose_data in enumerate(batch_poses):
            frame = batch_start + i + 1
            
            try:
                # Apply pose matrix
                if use_acsm_bone:
                    self.keyframe_target.matrix = pose_data['pose_matrix']
                else:
                    self.keyframe_target.matrix_local = pose_data['pose_matrix']

                # Set all custom properties
                self.keyframe_target["input_rot_x"] = pose_data['rx']
                self.keyframe_target["input_rot_y"] = pose_data['ry']
                self.keyframe_target["input_rot_z"] = pose_data['rz']
                self.keyframe_target["input_trans_x"] = pose_data['tx']
                self.keyframe_target["input_trans_y"] = pose_data['ty']
                self.keyframe_target["input_trans_z"] = pose_data['tz']

                # Insert all keyframes
                self.keyframe_target.keyframe_insert(data_path="location", frame=frame)
                self.keyframe_target.keyframe_insert(data_path="rotation_euler", frame=frame)
                self.keyframe_target.keyframe_insert(data_path='["input_rot_x"]', frame=frame)
                self.keyframe_target.keyframe_insert(data_path='["input_rot_y"]', frame=frame)
                self.keyframe_target.keyframe_insert(data_path='["input_rot_z"]', frame=frame)
                self.keyframe_target.keyframe_insert(data_path='["input_trans_x"]', frame=frame)
                self.keyframe_target.keyframe_insert(data_path='["input_trans_y"]', frame=frame)
                self.keyframe_target.keyframe_insert(data_path='["input_trans_z"]', frame=frame)
            
            except Exception as e:
                if self.keyframe_batch_index == 0 and i < 10:
                    print(f"⚠️  Error creating keyframe {frame}: {e}")
                continue
        
        # Update scene once per batch
        bpy.context.view_layer.update()
        
        # Move to next batch
        self.keyframe_batch_index += 1
        
        return True  # More batches remain

    def finish_keyframe_creation(self, props):
        """Clean up after keyframe creation"""
        if hasattr(self, 'original_use_keyframe_insert_auto'):
            scene = bpy.context.scene
            scene.tool_settings.use_keyframe_insert_auto = self.original_use_keyframe_insert_auto
        
        self.is_creating_keyframes = False
        props.calculation_progress = 100.0
        
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
                
                print(f"Results exported to: {filepath}")
                
            except Exception as e:
                print(f"❌ Error exporting CSV: {e}")
                success = False

        # NOTE: Keyframe creation is now handled in modal loop for better UI responsiveness
        # The modal operator will call start_keyframe_creation() after export

        return success

    def _create_animation_keyframes(self, props):
        """Create animation keyframes for valid poses"""
        acsm_obj = props.ACSm_object
        acsm_bone_name = getattr(props, 'ACSm_bone', None)
        # Check if bone name is valid (not 'NONE' placeholder)
        use_acsm_bone = (acsm_obj and 
                         acsm_obj.type == 'ARMATURE' and 
                         acsm_bone_name and 
                         acsm_bone_name != 'NONE')

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
                # Check if we're creating keyframes
                if self._processor.is_creating_keyframes:
                    # Process keyframe batch
                    still_creating = self._processor.process_keyframe_batch(props)
                    
                    if not still_creating:
                        # Keyframe creation complete
                        self.finish_processing(context)
                        return {'FINISHED'}
                    
                    return {'PASS_THROUGH'}
                
                # Normal collision detection processing
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
                    # Collision detection complete - export CSV
                    try:
                        if self._processor.export_results(props):
                            elapsed = time.time() - self._processor.start_time
                            mins, secs = divmod(int(elapsed), 60)
                            poses_per_second = self._processor.processed_poses / elapsed if elapsed > 0 else 0
                            
                            self.report({'INFO'}, 
                                f"Collision detection complete! "
                                f"Found {len(self._processor.valid_poses):,} valid poses "
                                f"in {mins:02d}:{secs:02d} "
                                f"({poses_per_second:.0f} poses/sec)")
                        else:
                            self.report({'WARNING'}, "Export had issues")
                    except Exception as e:
                        self.report({'ERROR'}, f"Export failed: {e}")
                    
                    # Now start keyframe creation if requested
                    if props.visualize_collisions and self._processor.valid_poses:
                        if self._processor.start_keyframe_creation(props):
                            # Continue modal loop for keyframe creation
                            return {'PASS_THROUGH'}
                        else:
                            self.report({'WARNING'}, "Could not start keyframe creation")
                    
                    # No keyframes requested or couldn't start - finish now
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
