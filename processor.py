"""
processor.py — ROM processing engine for BlenderROMFinder.

Contains `ROMProcessor` (formerly OptimizedROMProcessor) which handles:
  - Pose range generation and iteration
  - Batch processing (in-process and worker-delegated)
  - Keyframe creation (Blender 5.0 Layered Animation API)
  - CSV export
  - Worker result merging

Collision detection is delegated to `CollisionDetector` from collision.py.
"""

import bpy
import csv
import json
import math
import os
import queue
import subprocess
import sys
import threading
import time
import traceback
from itertools import islice, product
from pathlib import Path

from bpy.props import StringProperty
from bpy.types import Operator
from mathutils import Matrix, Vector

from .collision import CollisionDetector
from .pose_math import (
    calculate_pose_for_isb_standard_mode,
    calculate_pose_for_intuitive_mode,
    calculate_pose_for_mg_hinge_mode,
)


# ---------------------------------------------------------------------------
# Utility: range counting (used by UI to show total pose count)
# ---------------------------------------------------------------------------

def compute_num_inc(min_val, max_val, inc):
    """Return the number of sampled points for a range as a float.

    Counts both endpoints, so min=170, max=190, inc=10 → 3 (170, 180, 190).
    Returns 1.0 if min == max or inc is zero (single position).
    A non-integer result means the increment doesn't divide the range evenly.
    """
    if min_val == max_val or inc == 0:
        return 1.0
    return (max_val - min_val) / inc + 1


def generate_range_values(min_val, max_val, inc):
    """Return the list of unique sample points for a range.

    Uses index-based stepping (min + i*step) to avoid float accumulation.
    When the range divides evenly (compute_num_inc is integer within epsilon),
    the final point is clamped to exactly max_val so it is never missed.
    """
    if inc == 0:
        return [round(min_val, 6)]

    diff = max_val - min_val
    if diff == 0:
        return [round(min_val, 6)]

    # Reject if step direction opposes the range direction
    if (diff > 0 and inc < 0) or (diff < 0 and inc > 0):
        return []

    num_inc_f = diff / inc
    rounded = round(num_inc_f)
    is_exact = abs(num_inc_f - rounded) < 1e-9

    n_steps = rounded if is_exact else math.floor(num_inc_f)

    seen = set()
    vals = []
    for i in range(n_steps + 1):
        if is_exact and i == n_steps:
            val = round(max_val, 6)
        else:
            val = round(min_val + i * inc, 6)
        if val not in seen:
            seen.add(val)
            vals.append(val)
    return vals


def count_range_steps(min_val, max_val, inc):
    """Return the number of unique sample points in the range (thin wrapper)."""
    return len(generate_range_values(min_val, max_val, inc))


def get_total_pose_count(props):
    """Product of step counts across all 6 axes."""
    rx = count_range_steps(props.rot_x_min, props.rot_x_max, props.rot_x_inc)
    ry = count_range_steps(props.rot_y_min, props.rot_y_max, props.rot_y_inc)
    rz = count_range_steps(props.rot_z_min, props.rot_z_max, props.rot_z_inc)
    tx = count_range_steps(props.trans_x_min, props.trans_x_max, props.trans_x_inc)
    ty = count_range_steps(props.trans_y_min, props.trans_y_max, props.trans_y_inc)
    tz = count_range_steps(props.trans_z_min, props.trans_z_max, props.trans_z_inc)
    return rx * ry * rz * tx * ty * tz


# ---------------------------------------------------------------------------
# ROMProcessor
# ---------------------------------------------------------------------------

class ROMProcessor:
    """ROM processor that delegates collision detection to CollisionDetector."""

    def __init__(self):
        self.total_poses = 0
        self.processed_poses = 0
        self.valid_poses = []
        self.csv_data = []
        self.is_cancelled = False
        self.start_time = None
        self.pose_iterator = None
        self._iterator_pos = 0    # how many poses have been consumed from self.pose_iterator
        self._skip_to = 0         # iterator must reach this position before main-process work begins

        # Object references
        self.prox_obj = None
        self.dist_obj = None
        self.acsm_obj = None
        self.acsm_initial_local = None
        self.operational_mode = None

        # Collision detector (replaces all inline BVH/hull/AABB code)
        self.detector = CollisionDetector()

        # Keyframe creation state
        self.keyframe_batch_index = 0
        self.keyframe_batch_size = 1000
        self.is_creating_keyframes = False
        self.keyframe_target = None

        # Pose ranges (populated by initialize)
        self.rot_x_range = []
        self.rot_y_range = []
        self.rot_z_range = []
        self.trans_x_range = []
        self.trans_y_range = []
        self.trans_z_range = []

    # ------------------------------------------------------------------
    # Debug helper
    # ------------------------------------------------------------------

    def _debug_print(self, *args, **kwargs):
        try:
            if getattr(self, 'props', None) and getattr(self.props, 'debug_mode', False):
                print(*args, **kwargs)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # ACSm bone helper
    # ------------------------------------------------------------------

    def _get_acsm_bone_info(self, props):
        acsm_bone_name = getattr(props, 'ACSm_bone', None)
        use_acsm_bone = (
            self.acsm_obj
            and self.acsm_obj.type == 'ARMATURE'
            and acsm_bone_name
            and acsm_bone_name != 'NONE'
        )
        return acsm_bone_name, use_acsm_bone

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def initialize(self, props):
        """Set up objects, collision detector, pose ranges."""
        print(" Initializing ROM processor...")

        self.prox_obj = props.proximal_object
        self.dist_obj = props.distal_object
        self.acsm_obj = props.ACSm_object
        acsf_obj = props.ACSf_object

        if not all([self.prox_obj, self.dist_obj, acsf_obj, self.acsm_obj]):
            raise ValueError("Missing required objects")

        # ACSm initial matrix
        acsm_bone_name, use_acsm_bone = self._get_acsm_bone_info(props)
        if use_acsm_bone:
            pose_bone = self.acsm_obj.pose.bones.get(acsm_bone_name)
            if pose_bone:
                self.acsm_initial_local = pose_bone.matrix.copy()
            else:
                raise ValueError(f"ACSm bone '{acsm_bone_name}' not found")
        else:
            self.acsm_initial_local = self.acsm_obj.matrix_local.copy()

        # Initialise collision detector
        use_convex_hull = getattr(props, 'use_convex_hull_optimization', True)
        use_proxy = getattr(props, 'use_proxy_collision', False)
        proxy_ratio = getattr(props, 'proxy_decimate_ratio', 0.25)
        sample_count = getattr(props, 'penetration_sample_count', 10000)
        debug = getattr(props, 'debug_mode', False)

        self.detector.initialize(
            self.prox_obj, self.dist_obj,
            use_convex_hull=use_convex_hull,
            use_proxy=use_proxy,
            proxy_decimate_ratio=proxy_ratio,
            penetration_sample_count=sample_count,
            debug=debug,
        )

        # Keyframe target: insert frame-0 keyframe for the initial state
        target = self._get_keyframe_target(props)
        if target:
            self.reset_acsm_to_initial(update_scene=False)
            target.keyframe_insert(data_path="location", frame=0)
            target.keyframe_insert(data_path="rotation_euler", frame=0)

        # Store mode & props reference
        self.operational_mode = props.rotation_mode_enum
        self.props = props

        # Generate pose ranges
        (self.rot_x_range, self.rot_y_range, self.rot_z_range,
         self.trans_x_range, self.trans_y_range, self.trans_z_range) = self._generate_pose_ranges(props)

        self.total_poses = (
            len(self.rot_x_range)
            * len(self.rot_y_range)
            * len(self.rot_z_range)
            * len(self.trans_x_range)
            * len(self.trans_y_range)
            * len(self.trans_z_range)
        )
        self.pose_iterator = self._iter_pose_combinations()
        self._iterator_pos = 0
        self._skip_to = 0

        # CSV header + reset counters
        self.csv_data = [["rot_x", "rot_y", "rot_z", "trans_x", "trans_y", "trans_z", "Valid_pose"]]
        self.processed_poses = 0
        self.valid_poses = []

        print(f"  Initialized for {self.total_poses:,} poses")
        return True

    # ------------------------------------------------------------------
    # Pose range generation
    # ------------------------------------------------------------------

    @staticmethod
    def _generate_pose_ranges(props):
        return (
            generate_range_values(props.rot_x_min, props.rot_x_max, props.rot_x_inc),
            generate_range_values(props.rot_y_min, props.rot_y_max, props.rot_y_inc),
            generate_range_values(props.rot_z_min, props.rot_z_max, props.rot_z_inc),
            generate_range_values(props.trans_x_min, props.trans_x_max, props.trans_x_inc),
            generate_range_values(props.trans_y_min, props.trans_y_max, props.trans_y_inc),
            generate_range_values(props.trans_z_min, props.trans_z_max, props.trans_z_inc),
        )

    def _iter_pose_combinations(self):
        return product(
            self.rot_x_range, self.rot_y_range, self.rot_z_range,
            self.trans_x_range, self.trans_y_range, self.trans_z_range,
        )

    # ------------------------------------------------------------------
    # Keyframe target
    # ------------------------------------------------------------------

    def _get_keyframe_target(self, props):
        acsm_bone_name, use_acsm_bone = self._get_acsm_bone_info(props)
        if use_acsm_bone:
            return self.acsm_obj.pose.bones.get(acsm_bone_name)
        return self.acsm_obj

    # ------------------------------------------------------------------
    # Pose computation helper (shared by process_batch & process_pose_range)
    # ------------------------------------------------------------------

    def _compute_pose_matrix(self, rx, ry, rz, tx, ty, tz):
        """Return the local pose matrix for the current operational mode."""
        if self.operational_mode == 'ISB_STANDARD':
            return calculate_pose_for_isb_standard_mode(
                rx, ry, rz, tx, ty, tz, None, self.acsm_initial_local,
                acsm_obj=self.acsm_obj)
        elif self.operational_mode == 'INTUITIVE':
            return calculate_pose_for_intuitive_mode(
                rx, ry, rz, tx, ty, tz, None, self.acsm_initial_local,
                acsm_obj=self.acsm_obj)
        elif self.operational_mode == 'MG_HINGE':
            return calculate_pose_for_mg_hinge_mode(
                rx, ry, rz, tx, ty, tz, self.props, self.acsm_initial_local)
        return None

    def _apply_pose(self, pose_matrix):
        """Apply *pose_matrix* to ACSm (bone or object) and update the scene."""
        acsm_bone_name, use_acsm_bone = self._get_acsm_bone_info(
            bpy.context.scene.collision_props)
        if use_acsm_bone:
            pb = self.acsm_obj.pose.bones.get(acsm_bone_name)
            if pb:
                pb.matrix = pose_matrix
        else:
            self.acsm_obj.matrix_local = pose_matrix
        bpy.context.view_layer.update()

    def _check_collision(self):
        """Run the collision cascade via `CollisionDetector`."""
        return self.detector.check(self.dist_obj.matrix_world)

    # ------------------------------------------------------------------
    # Batch processing (in-process)
    # ------------------------------------------------------------------

    def process_batch(self, batch_size=1000):
        if self.is_cancelled or self.processed_poses >= self.total_poses:
            return False

        # Skip poses already processed by headless workers.
        # _skip_to  : iterator must be at or past this index before we process.
        # _iterator_pos : how many items have actually been consumed from pose_iterator.
        # Using a target position (not a countdown) means late-arriving worker merges
        # never cause us to skip poses the main process hasn't consumed yet.
        skip_to = self._skip_to
        to_skip = max(0, skip_to - self._iterator_pos)
        if to_skip > 0:
            chunk = min(to_skip, batch_size)
            actual = len(list(islice(self.pose_iterator, chunk)))
            self._iterator_pos += actual
            if to_skip > chunk:
                # Still more to skip — yield to Blender's event loop and come back
                return True
            # Finished skipping; fall through to process the next poses

        batch_poses = list(islice(self.pose_iterator, batch_size))
        if not batch_poses:
            return False
        self._iterator_pos += len(batch_poses)

        batch_start = self.processed_poses
        batch_end = batch_start + len(batch_poses)

        print(f"  Processing batch {batch_start:,} to {batch_end:,} ({len(batch_poses):,} poses)")

        valid_in_batch = 0
        # Read debug flags live from the scene property (avoids stale RNA wrapper issues)
        try:
            live_props = bpy.context.scene.collision_props
            debug_all = (getattr(live_props, 'debug_mode', False)
                         and getattr(live_props, 'turn_off_collisions', False))
        except Exception:
            debug_all = False

        for i, (rx, ry, rz, tx, ty, tz) in enumerate(batch_poses):
            if self.is_cancelled:
                break

            pose_index = batch_start + i
            pose_matrix = self._compute_pose_matrix(rx, ry, rz, tx, ty, tz)
            if pose_matrix is None:
                continue

            self._apply_pose(pose_matrix)
            collision = False if debug_all else self._check_collision()

            if not getattr(self.props, 'only_export_valid_poses', False) or (not collision):
                self.csv_data.append([pose_index, rx, ry, rz, tx, ty, tz, 0 if collision else 1])

            if not collision:
                self.valid_poses.append({
                    'pose_index': pose_index,
                    'pose_params': (rx, ry, rz, tx, ty, tz),
                    'pose_matrix': pose_matrix.copy(),
                    'rx': rx, 'ry': ry, 'rz': rz,
                    'tx': tx, 'ty': ty, 'tz': tz,
                })
                valid_in_batch += 1

        self.processed_poses = batch_end

        stats = self._calculate_progress_stats()
        print(f" Progress: {stats['progress']:.1f}% - {stats['poses_per_second']:.0f} poses/sec "
              f"- Valid: {valid_in_batch} (Total: {len(self.valid_poses)})")
        print(f" Est. remaining: {stats['remaining']/60:.1f} minutes")

        return self.processed_poses < self.total_poses and not self.is_cancelled

    # ------------------------------------------------------------------
    # Worker-range processing (used by headless workers)
    # ------------------------------------------------------------------

    def process_pose_range(self, start_index, end_index):
        """Process [start_index, end_index] and return a result dict.

        Does NOT mutate class-level csv_data/valid_poses.
        """
        results_csv = []
        results_valids = []

        debug_all = (getattr(self.props, 'debug_mode', False)
                     and getattr(self.props, 'turn_off_collisions', False))

        iter_all = product(
            self.rot_x_range, self.rot_y_range, self.rot_z_range,
            self.trans_x_range, self.trans_y_range, self.trans_z_range,
        )
        sliced = islice(iter_all, start_index, end_index + 1)

        idx = start_index
        for rx, ry, rz, tx, ty, tz in sliced:
            if self.is_cancelled:
                break

            pose_matrix = self._compute_pose_matrix(rx, ry, rz, tx, ty, tz)
            if pose_matrix is None:
                idx += 1
                continue

            self._apply_pose(pose_matrix)
            collision = False if debug_all else self._check_collision()

            results_csv.append([idx, rx, ry, rz, tx, ty, tz, 0 if collision else 1])

            if not collision:
                results_valids.append({
                    "pose_index": idx,
                    "pose_params": [rx, ry, rz, tx, ty, tz],
                    "pose_matrix": self._matrix_to_list(pose_matrix),
                    "rx": rx, "ry": ry, "rz": rz,
                    "tx": tx, "ty": ty, "tz": tz,
                })
            idx += 1

        return {"csv_rows": results_csv, "valids": results_valids}

    # ------------------------------------------------------------------
    # Worker result merging
    # ------------------------------------------------------------------

    def merge_worker_payload(self, payload):
        csv_rows = payload.get('csv_rows', [])
        valids = payload.get('valids', [])

        for row in csv_rows:
            if not getattr(self.props, 'only_export_valid_poses', False) or row[-1] == 1:
                self.csv_data.append(row)

        for v in valids:
            matrix = self._list_to_matrix(v.get('pose_matrix'))
            self.valid_poses.append({
                'pose_index': v.get('pose_index'),
                'pose_params': tuple(v.get('pose_params', [])),
                'pose_matrix': matrix,
                'rx': v.get('rx'), 'ry': v.get('ry'), 'rz': v.get('rz'),
                'tx': v.get('tx'), 'ty': v.get('ty'), 'tz': v.get('tz'),
            })

        prev = self.processed_poses
        self.processed_poses += len(csv_rows)
        # Advance the skip target to just past the highest pose index this worker covered.
        # Deriving it from the actual indices (not chunk size) means it's correct regardless
        # of how many main-process batches ran before this merge arrived.
        if csv_rows:
            worker_indices = [row[0] for row in csv_rows
                              if len(row) >= 1 and isinstance(row[0], int)]
            if worker_indices:
                self._skip_to = max(self._skip_to, max(worker_indices) + 1)
        print(f"Merged worker payload: +{len(csv_rows)} csv rows, +{len(valids)} valids "
              f"(processed {prev} -> {self.processed_poses}, skip_to={self._skip_to})")

    # ------------------------------------------------------------------
    # Lifecycle helpers
    # ------------------------------------------------------------------

    def start_processing(self):
        self.start_time = time.time()
        print(f" Starting processing of {self.total_poses:,} poses")

    def get_progress(self):
        if self.total_poses == 0:
            return 100.0
        return (self.processed_poses / self.total_poses) * 100.0

    def cancel(self):
        self.is_cancelled = True
        print("Processing cancelled")

    def reset_acsm_to_initial(self, update_scene=True):
        if self.acsm_obj and self.acsm_initial_local:
            acsm_bone_name, use_acsm_bone = self._get_acsm_bone_info(
                bpy.context.scene.collision_props)
            if use_acsm_bone:
                pb = self.acsm_obj.pose.bones.get(acsm_bone_name)
                if pb:
                    pb.matrix = self.acsm_initial_local
            else:
                self.acsm_obj.matrix_local = self.acsm_initial_local
            if update_scene:
                bpy.context.view_layer.update()

    def cleanup(self):
        """Release all temporary resources."""
        self.detector.cleanup()

    # ------------------------------------------------------------------
    # Matrix serialisation helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _matrix_to_list(mat):
        return [float(v) for row in mat for v in row]

    @staticmethod
    def _list_to_matrix(lst):
        if not lst or len(lst) != 16:
            return None
        return Matrix([[lst[i * 4 + j] for j in range(4)] for i in range(4)])

    # ------------------------------------------------------------------
    # Keyframe creation (Blender 5.0 Layered Animation API)
    # ------------------------------------------------------------------

    def start_keyframe_creation(self, props):
        if not self.valid_poses:
            return False

        print(f"Starting keyframe creation for {len(self.valid_poses):,} poses...")

        self.keyframe_batch_index = 0
        self.is_creating_keyframes = True

        acsm_bone_name, use_acsm_bone = self._get_acsm_bone_info(props)
        if use_acsm_bone:
            self.keyframe_target = self.acsm_obj.pose.bones.get(acsm_bone_name)
        else:
            self.keyframe_target = self.acsm_obj

        if not self.keyframe_target:
            print("No valid target for keyframes")
            return False

        try:
            self.valid_poses.sort(key=lambda v: v.get('pose_index', 0))
        except Exception:
            pass

        scene = bpy.context.scene
        self.original_use_keyframe_insert_auto = scene.tool_settings.use_keyframe_insert_auto
        scene.tool_settings.use_keyframe_insert_auto = False

        self.reset_acsm_to_initial()
        self.keyframe_target.keyframe_insert(data_path="location", frame=0)
        self.keyframe_target.keyframe_insert(data_path="rotation_euler", frame=0)

        props.calculation_progress = 0.0
        props.time_remaining = "Creating animation keyframes..."
        return True

    def process_keyframe_batch(self, props):
        if not self.is_creating_keyframes or not self.keyframe_target:
            return False

        total_poses = len(self.valid_poses)
        batch_start = self.keyframe_batch_index * self.keyframe_batch_size
        is_pose_bone = isinstance(self.keyframe_target, bpy.types.PoseBone)

        if batch_start >= total_poses:
            self.apply_collected_keyframes_5_0()
            self.finish_keyframe_creation(props)
            return False

        batch_end = min(batch_start + self.keyframe_batch_size, total_poses)
        batch_poses = self.valid_poses[batch_start:batch_end]

        progress = (batch_end / total_poses) * 100
        props.calculation_progress = progress
        props.time_remaining = f"Calculating data: {batch_end:,}/{total_poses:,} ({progress:.1f}%)"

        rot_mode = self.keyframe_target.rotation_mode

        if self.keyframe_batch_index == 0:
            self.collected_data = {
                "location": {0: [], 1: [], 2: []},
                '["input_rot_x"]': {0: []},
                '["input_rot_y"]': {0: []},
                '["input_rot_z"]': {0: []},
                '["input_trans_x"]': {0: []},
                '["input_trans_y"]': {0: []},
                '["input_trans_z"]': {0: []},
            }
            if rot_mode == 'QUATERNION':
                self.collected_data['rotation_quaternion'] = {0: [], 1: [], 2: [], 3: []}
            else:
                self.collected_data['rotation_euler'] = {0: [], 1: [], 2: []}
            self.frames = []

        for i, pose_data in enumerate(batch_poses):
            frame = batch_start + i + 1
            self.frames.append(frame)

            matrix = pose_data['pose_matrix']
            if isinstance(matrix, list):
                matrix = self._list_to_matrix(matrix)

            if is_pose_bone:
                self.keyframe_target.matrix = matrix
            else:
                self.keyframe_target.matrix_local = matrix

            self.keyframe_target["input_rot_x"] = pose_data['rx']
            self.keyframe_target["input_rot_y"] = pose_data['ry']
            self.keyframe_target["input_rot_z"] = pose_data['rz']
            self.keyframe_target["input_trans_x"] = pose_data['tx']
            self.keyframe_target["input_trans_y"] = pose_data['ty']
            self.keyframe_target["input_trans_z"] = pose_data['tz']
            self.keyframe_target["Valid pose"] = 1

            bpy.context.view_layer.update()

            if is_pose_bone:
                loc = self.keyframe_target.location
            else:
                loc = self.keyframe_target.location

            self.collected_data["location"][0].append(loc.x)
            self.collected_data["location"][1].append(loc.y)
            self.collected_data["location"][2].append(loc.z)

            if rot_mode == 'QUATERNION':
                q = self.keyframe_target.rotation_quaternion
                self.collected_data['rotation_quaternion'][0].append(q.w)
                self.collected_data['rotation_quaternion'][1].append(q.x)
                self.collected_data['rotation_quaternion'][2].append(q.y)
                self.collected_data['rotation_quaternion'][3].append(q.z)
            else:
                e = self.keyframe_target.rotation_euler
                self.collected_data['rotation_euler'][0].append(e.x)
                self.collected_data['rotation_euler'][1].append(e.y)
                self.collected_data['rotation_euler'][2].append(e.z)

            self.collected_data['["input_rot_x"]'][0].append(self.keyframe_target.get('input_rot_x'))
            self.collected_data['["input_rot_y"]'][0].append(self.keyframe_target.get('input_rot_y'))
            self.collected_data['["input_rot_z"]'][0].append(self.keyframe_target.get('input_rot_z'))
            self.collected_data['["input_trans_x"]'][0].append(self.keyframe_target.get('input_trans_x'))
            self.collected_data['["input_trans_y"]'][0].append(self.keyframe_target.get('input_trans_y'))
            self.collected_data['["input_trans_z"]'][0].append(self.keyframe_target.get('input_trans_z'))

        self.keyframe_batch_index += 1
        return True

    def apply_collected_keyframes_5_0(self):
        """Write all collected keyframe data via Blender 5.0 Layered Animation API."""
        target = self.keyframe_target
        if not self.frames:
            return

        is_pose_bone = isinstance(target, bpy.types.PoseBone)
        arm_obj = self.acsm_obj if is_pose_bone else target

        if not arm_obj.animation_data:
            arm_obj.animation_data_create()

        action = arm_obj.animation_data.action
        if not action:
            action = bpy.data.actions.new(name=f"{arm_obj.name}_Action")
            arm_obj.animation_data.action = action

        if len(action.slots) == 0:
            slot = action.slots.new(id_type='OBJECT', name=arm_obj.name)
        else:
            slot = action.slots[0]

        arm_obj.animation_data.action_slot = slot

        if len(action.layers) == 0:
            layer = action.layers.new(name="Base Layer")
        else:
            layer = action.layers[0]

        if len(layer.strips) == 0:
            strip = layer.strips.new(type='KEYFRAME', start=0)
        else:
            strip = layer.strips[0]

        channel_bag = strip.channelbag(slot, ensure=True)
        fcurves_collection = channel_bag.fcurves
        frame_count = len(self.frames)

        def build_data_path(base_path):
            if not is_pose_bone:
                return base_path
            bone_name = target.name
            if base_path.startswith('["'):
                return f'pose.bones["{bone_name}"]' + base_path
            return f'pose.bones["{bone_name}"].{base_path}'

        for data_path, indices in self.collected_data.items():
            for array_index, values in indices.items():
                full_path = build_data_path(data_path)
                fcurve = fcurves_collection.find(data_path=full_path, index=array_index)
                if not fcurve:
                    fcurve = fcurves_collection.new(data_path=full_path, index=array_index)

                fcurve.keyframe_points.clear()

                kfp_flat = [0.0] * (frame_count * 2)
                kfp_flat[0::2] = self.frames
                kfp_flat[1::2] = values

                fcurve.keyframe_points.add(frame_count)
                fcurve.keyframe_points.foreach_set("co", kfp_flat)
                fcurve.update()

        self.collected_data = None
        self.frames = None
        bpy.context.view_layer.update()
        print("Blender 5.0 Keyframes Applied Successfully.")

    def finish_keyframe_creation(self, props):
        if hasattr(self, 'original_use_keyframe_insert_auto'):
            bpy.context.scene.tool_settings.use_keyframe_insert_auto = self.original_use_keyframe_insert_auto

        self.reset_acsm_to_initial()
        self.keyframe_target.keyframe_insert(data_path="location", frame=0)
        self.keyframe_target.keyframe_insert(data_path="rotation_euler", frame=0)

        self.is_creating_keyframes = False
        props.calculation_progress = 100.0
        print(f"Animation keyframes complete: {len(self.valid_poses):,} poses")

    # ------------------------------------------------------------------
    # CSV export
    # ------------------------------------------------------------------

    def export_results(self, props):
        if not self.csv_data or len(self.csv_data) <= 1:
            return False

        success = True
        if props.export_to_csv and props.export_path:
            try:
                filepath = bpy.path.abspath(props.export_path)
                dirpath = os.path.dirname(filepath)
                if dirpath:
                    os.makedirs(dirpath, exist_ok=True)

                header = self.csv_data[0]
                rows = self.csv_data[1:]

                indexed = [r for r in rows if len(r) >= 8 and isinstance(r[0], int)]
                nonindexed = [r for r in rows if not (len(r) >= 8 and isinstance(r[0], int))]
                indexed_sorted = sorted(indexed, key=lambda r: r[0]) if indexed else []
                rows_to_write = [r[1:] for r in indexed_sorted] + nonindexed

                with open(filepath, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(header)
                    writer.writerows(rows_to_write)

                print(f"Results exported to: {filepath}")
            except Exception as e:
                print(f"Error exporting CSV: {e}")
                success = False

        return success

    # ------------------------------------------------------------------
    # Progress stats
    # ------------------------------------------------------------------

    def _calculate_progress_stats(self):
        if self.total_poses == 0:
            return {'progress': 100.0, 'elapsed': 0, 'remaining': 0, 'poses_per_second': 0}

        elapsed = time.time() - self.start_time if self.start_time else 0
        progress = (self.processed_poses / self.total_poses) * 100.0
        pps = self.processed_poses / elapsed if elapsed > 0 else 0
        remaining = (self.total_poses - self.processed_poses) / pps if pps > 0 else 0

        return {
            'progress': progress,
            'elapsed': elapsed,
            'remaining': remaining,
            'poses_per_second': pps,
        }


# ---------------------------------------------------------------------------
# Headless worker spawn & reader helpers
# ---------------------------------------------------------------------------

def _reader_thread(proc, q, worker_id):
    """Parse stdout from a headless worker and enqueue structured messages."""
    buffer = None
    try:
        for raw in iter(proc.stdout.readline, ''):
            if raw is None:
                break
            line = raw.strip()
            if not line:
                continue

            if line.startswith('ROMF_PROGRESS'):
                try:
                    payload = json.loads(line[len('ROMF_PROGRESS'):].strip())
                    q.put(("progress", payload, worker_id))
                except Exception:
                    continue
            elif line == 'ROMF_RESULT_START':
                buffer = []
            elif line == 'ROMF_RESULT_END':
                if buffer is not None:
                    try:
                        blob = ''.join(buffer)
                        payload = json.loads(blob)
                        q.put(("result", payload, worker_id))
                    except Exception:
                        q.put(("error", {"msg": "Invalid result JSON"}, worker_id))
                    buffer = None
            elif line.startswith('ROMF_ERROR'):
                try:
                    payload = json.loads(line[len('ROMF_ERROR'):].strip())
                except Exception:
                    payload = {"msg": line}
                q.put(("error", payload, worker_id))
            else:
                if buffer is not None:
                    buffer.append(line)
                else:
                    q.put(("log", line, worker_id))
    except Exception as exc:
        q.put(("error", {"msg": str(exc), "trace": traceback.format_exc()}, worker_id))
    finally:
        try:
            proc.stdout.close()
        except Exception:
            pass


def start_headless_workers_async(blend_file, worker_script, total_poses,
                                  worker_count=1, chunk_size=1000,
                                  blender_exec=None, timeout=1800, props_b64=None):
    """Spawn headless Blender worker processes. Returns list of worker dicts."""
    workers = []

    if not blender_exec:
        blender_exec = bpy.app.binary_path

    chunk = math.ceil(total_poses / max(1, worker_count))

    for i in range(worker_count):
        start = i * chunk
        end = min(total_poses - 1, (i + 1) * chunk - 1)
        if start > end:
            continue

        cmd = [blender_exec, blend_file, '--background', '--python', str(worker_script), '--',
               '--start-index', str(start), '--end-index', str(end),
               '--emit-chunk', str(min(chunk, chunk_size)), '--worker-id', str(i)]

        props_file_path = None
        if props_b64:
            try:
                import tempfile, base64
                tmpf = tempfile.NamedTemporaryFile(delete=False, prefix='romf_props_', suffix='.json')
                tmpf.write(base64.b64decode(props_b64.encode('ascii')))
                tmpf.close()
                props_file_path = tmpf.name
                cmd.extend(['--props-json-file', props_file_path])
            except Exception:
                cmd.extend(['--props-json-b64', props_b64])

        print(f"Starting worker {i}: {' '.join(cmd)}")

        if '--start-index' not in cmd or '--end-index' not in cmd:
            raise RuntimeError(f"Worker command missing required args: {cmd!r}")

        try:
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                 text=True, bufsize=1)
        except Exception as exc:
            for w in workers:
                try:
                    w['proc'].terminate()
                except Exception:
                    pass
            raise

        try:
            time.sleep(0.05)
        except Exception:
            pass

        rc = p.poll()
        if rc is not None and rc != 0:
            try:
                out, _ = p.communicate(timeout=1)
            except Exception:
                out = ''
            print(f"Worker {i} exited immediately with code {rc}. Output:\n{out[:4000]}")
            q_obj = queue.Queue()
            workers.append({
                'proc': p, 'queue': q_obj, 'thread': None,
                'start': start, 'end': end, 'worker_id': i,
                'last_progress': 0, 'total': (end - start + 1),
                'done': True, 'last_output_time': time.time(),
                'props_file': props_file_path,
            })
            continue

        q_obj = queue.Queue()
        t = threading.Thread(target=_reader_thread, args=(p, q_obj, i), daemon=True)
        t.start()

        workers.append({
            'proc': p, 'queue': q_obj, 'thread': t,
            'start': start, 'end': end, 'worker_id': i,
            'last_progress': 0, 'total': (end - start + 1),
            'done': False, 'last_output_time': time.time(),
            'props_file': props_file_path,
        })

        try:
            time.sleep(0.02)
        except Exception:
            pass

    return workers
