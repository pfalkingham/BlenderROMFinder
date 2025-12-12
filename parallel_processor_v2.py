import bpy
import bmesh
import mathutils
import math
from itertools import islice, product
import csv
import os
import time
import numpy as np
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
        self.total_poses = 0
        self.processed_poses = 0
        self.valid_poses = []
        self.csv_data = []
        self.is_cancelled = False
        self.start_time = None
        self.pose_iterator = None
        
        # Use the same collision detection approach as original
        self.prox_obj = None
        self.dist_obj = None
        self.acsm_obj = None
        self.prox_bvh = None
        self.acsm_initial_local = None
        self.operational_mode = None
        self._prox_bounds = None
        self.use_aabb_precheck = True
        self.aabb_margin = 0.0
        self.use_proxy_collision = False
        self.proxy_decimate_ratio = 1.0
        self.proxy_obj = None
        
        # Keyframe creation state
        self.keyframe_batch_index = 0
        self.keyframe_batch_size = 1000
        self.is_creating_keyframes = False
        self.keyframe_target = None
        
        # Cached distal mesh data for optimized BVH creation
        # Instead of regenerating BVH every pose, we cache the local mesh data
        # and transform it by the computed pose matrix
        self._dist_local_verts = None  # Vertices in distal object's local space
        self._dist_local_faces = None  # Face indices
        self._dist_initial_world = None  # Distal object's initial world matrix
        
        # Convex hull optimization
        self.use_convex_hull = False
        self._prox_hull_bvh = None
        self._dist_hull_local_verts = None
        self._dist_hull_local_faces = None
        
        # NumPy arrays for fast vertex transformations
        self._np_verts = None
        self._np_hull_verts = None

    def initialize(self, props):
        """Initialize using the same approach as the original operators.py"""
        print(" Initializing optimized ROM processor...")
        
        # Get objects (same as original)
        self.prox_obj = props.proximal_object
        self.dist_obj = props.distal_object
        acsf_obj = props.ACSf_object
        self.acsm_obj = props.ACSm_object

        self.use_aabb_precheck = getattr(props, 'use_aabb_precheck', True)
        self.aabb_margin = getattr(props, 'aabb_margin', 0.0)
        self.use_proxy_collision = getattr(props, 'use_proxy_collision', False)
        self.proxy_decimate_ratio = getattr(props, 'proxy_decimate_ratio', 1.0)
        self.use_convex_hull = getattr(props, 'use_convex_hull_optimization', False)

        # Clear any temp objects from prior runs
        self._cleanup_temp_objects()

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
        self._prox_bounds = self._get_world_bounds(self.prox_obj)

        # Optional proxy mesh for the distal object
        if self.use_proxy_collision:
            self.proxy_obj = self._create_proxy_object(self.dist_obj, self.proxy_decimate_ratio)
            if self.proxy_obj is None:
                # Fallback gracefully to full mesh if proxy creation failed or ratio ~1
                self.use_proxy_collision = False

        # OPTIMIZATION: Cache the distal mesh data once instead of regenerating BVH every pose
        # We store the mesh in local coordinates and apply the transform at collision time
        collision_source = self.proxy_obj if self.use_proxy_collision and self.proxy_obj else self.dist_obj
        self._dist_initial_world = self.dist_obj.matrix_world.copy()
        self._cache_mesh_data(collision_source)
        
        # If using convex hull optimization, create hulls once
        if self.use_convex_hull:
            self._setup_convex_hulls(collision_source)

        # Insert frame 0 keyframe for the pre-move pose (parity with standard operator)
        target = self._get_keyframe_target(props)
        if target:
            self._apply_initial_pose_for_keyframe(target)
            target.keyframe_insert(data_path="location", frame=0)
            target.keyframe_insert(data_path="rotation_euler", frame=0)

        # Store operational mode
        self.operational_mode = props.rotation_mode_enum
        # Keep props reference for translation calculations that need ACSf/ACSm objects
        self.props = props

        # Generate pose ranges and stream combinations to avoid large memory usage
        self.rot_x_range, self.rot_y_range, self.rot_z_range, self.trans_x_range, self.trans_y_range, self.trans_z_range = self._generate_pose_ranges(props)
        self.total_poses = (
            len(self.rot_x_range)
            * len(self.rot_y_range)
            * len(self.rot_z_range)
            * len(self.trans_x_range)
            * len(self.trans_y_range)
            * len(self.trans_z_range)
        )
        self.pose_iterator = self._iter_pose_combinations()

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

    def _cache_mesh_data(self, obj):
        """Cache mesh vertices and faces in local coordinates for fast BVH creation.
        
        This is the key optimization: instead of calling to_mesh() and creating a 
        bmesh every single pose, we extract the vertex/face data once and use 
        BVHTree.FromPolygons() with a transform applied.
        """
        mesh = obj.to_mesh()
        # Store vertices in local space (will be transformed per-pose)
        self._dist_local_verts = [v.co.copy() for v in mesh.vertices]
        self._dist_local_faces = [tuple(p.vertices) for p in mesh.polygons]
        
        # Create NumPy array for fast matrix multiplication (Nx4 for homogeneous coords)
        self._np_verts = np.array([[v.co.x, v.co.y, v.co.z, 1.0] for v in mesh.vertices])
        
        obj.to_mesh_clear()
        print(f"  Cached distal mesh: {len(self._dist_local_verts)} verts, {len(self._dist_local_faces)} faces")

    def _setup_convex_hulls(self, dist_collision_obj):
        """Create convex hulls once at initialization for fast pre-check.
        
        The proximal hull BVH is static. The distal hull vertices are cached
        and transformed per-pose, same as the full mesh.
        """
        print("  Setting up convex hull optimization...")
        
        # Create proximal convex hull BVH (static, done once)
        prox_hull_verts, prox_hull_faces = self._compute_convex_hull_data(self.prox_obj)
        if prox_hull_verts and prox_hull_faces:
            # Transform to world space (proximal is static)
            world_verts = [self.prox_obj.matrix_world @ v for v in prox_hull_verts]
            self._prox_hull_bvh = mathutils.bvhtree.BVHTree.FromPolygons(world_verts, prox_hull_faces)
            print(f"    Proximal hull: {len(prox_hull_verts)} verts, {len(prox_hull_faces)} faces")
        
        # Cache distal convex hull in local space (will be transformed per-pose)
        dist_hull_verts, dist_hull_faces = self._compute_convex_hull_data(dist_collision_obj)
        if dist_hull_verts and dist_hull_faces:
            self._dist_hull_local_verts = dist_hull_verts
            self._dist_hull_local_faces = dist_hull_faces
            
            # Create NumPy array for fast matrix multiplication
            self._np_hull_verts = np.array([[v.x, v.y, v.z, 1.0] for v in dist_hull_verts])
            
            print(f"    Distal hull: {len(dist_hull_verts)} verts, {len(dist_hull_faces)} faces")
        else:
            # Fallback: disable convex hull if creation failed
            self.use_convex_hull = False
            print("    Convex hull creation failed, falling back to full mesh")

    def _compute_convex_hull_data(self, obj):
        """Compute convex hull vertices and faces for an object.
        Returns (vertices, faces) in local space, or (None, None) on failure.
        """
        try:
            bm = bmesh.new()
            mesh = obj.to_mesh()
            bm.from_mesh(mesh)
            obj.to_mesh_clear()
            
            # Compute convex hull - returns dict with 'geom', 'geom_interior', etc.
            result = bmesh.ops.convex_hull(bm, input=bm.verts)
            
            # The result contains the hull geometry:
            # 'geom' - all hull geometry (verts, edges, faces)
            # 'geom_interior' - interior (non-hull) geometry to delete
            # We need to delete the interior and keep only the hull
            
            # Delete interior geometry (everything not part of the hull)
            interior_geom = result.get('geom_interior', [])
            if interior_geom:
                bmesh.ops.delete(bm, geom=interior_geom, context='VERTS')
            
            # Also delete any unused geometry
            unused_geom = result.get('geom_unused', [])
            if unused_geom:
                bmesh.ops.delete(bm, geom=unused_geom, context='VERTS')
            
            # Now the bmesh should contain only the hull
            bm.verts.ensure_lookup_table()
            bm.faces.ensure_lookup_table()
            
            # Get vertices and faces from the hull
            verts = [v.co.copy() for v in bm.verts]
            faces = [tuple(v.index for v in f.verts) for f in bm.faces]
            
            bm.free()
            return verts, faces
        except Exception as e:
            print(f"    Convex hull computation failed: {e}")
            return None, None

    def _create_bvh_from_cached(self, np_verts, faces, transform_matrix):
        """Create BVH tree from cached NumPy vertex array with matrix multiplication.
        
        This is MUCH faster than Python loops because:
        1. NumPy matrix multiplication in C
        2. No Python list comprehensions
        3. Vectorized operations
        """
        # Convert mathutils matrix to numpy and transpose for multiplication
        np_matrix = np.array(transform_matrix)
        # Transform vertices: (matrix @ verts.T).T gives Nx4 -> Nx3 after homogeneous divide
        transformed_homogeneous = np_matrix @ np_verts.T
        # Convert back to 3D coordinates (divide by w, but w=1 so just take xyz)
        transformed_verts = transformed_homogeneous[:3].T.tolist()
        return mathutils.bvhtree.BVHTree.FromPolygons(transformed_verts, faces)

    def _generate_pose_ranges(self, props):
        """Generate numeric ranges with de-duplication (shared with original logic)."""

        def gen_range(min_val, max_val, inc):
            if inc <= 0:
                return [round(min_val, 6)]

            vals = []
            current = min_val
            eps = inc * 1e-6
            while current <= max_val + eps:
                vals.append(round(current, 6))
                current += inc

            seen = set()
            unique_vals = []
            for v in vals:
                if v not in seen:
                    seen.add(v)
                    unique_vals.append(v)
            return unique_vals

        rot_x_range = gen_range(props.rot_x_min, props.rot_x_max, props.rot_x_inc)
        rot_y_range = gen_range(props.rot_y_min, props.rot_y_max, props.rot_y_inc)
        rot_z_range = gen_range(props.rot_z_min, props.rot_z_max, props.rot_z_inc)
        trans_x_range = gen_range(props.trans_x_min, props.trans_x_max, props.trans_x_inc)
        trans_y_range = gen_range(props.trans_y_min, props.trans_y_max, props.trans_y_inc)
        trans_z_range = gen_range(props.trans_z_min, props.trans_z_max, props.trans_z_inc)

        return rot_x_range, rot_y_range, rot_z_range, trans_x_range, trans_y_range, trans_z_range

    def _iter_pose_combinations(self):
        """Stream pose combinations without holding them all in memory."""
        return product(
            self.rot_x_range,
            self.rot_y_range,
            self.rot_z_range,
            self.trans_x_range,
            self.trans_y_range,
            self.trans_z_range,
        )

    def _get_world_bounds(self, obj):
        corners = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
        min_v = Vector((
            min(c.x for c in corners),
            min(c.y for c in corners),
            min(c.z for c in corners),
        ))
        max_v = Vector((
            max(c.x for c in corners),
            max(c.y for c in corners),
            max(c.z for c in corners),
        ))
        return min_v, max_v

    def _aabb_overlap(self, min_a, max_a, min_b, max_b, margin=0.0):
        for i in range(3):
            if (max_a[i] + margin) < min_b[i] or (max_b[i] + margin) < min_a[i]:
                return False
        return True

    def _create_proxy_object(self, source_obj, ratio):
        # Avoid proxy if ratio keeps full detail
        if ratio >= 0.999:
            return None

        try:
            if bpy.ops.object.mode_set.poll():
                bpy.ops.object.mode_set(mode='OBJECT')

            bpy.ops.object.select_all(action='DESELECT')
            source_obj.select_set(True)
            bpy.context.view_layer.objects.active = source_obj
            bpy.ops.object.duplicate()
            proxy_obj = bpy.context.active_object
            proxy_obj.name = f"{source_obj.name}_romf_proxy"

            decimate_mod = proxy_obj.modifiers.new(name="ROMF_Decimate", type='DECIMATE')
            decimate_mod.ratio = max(0.05, min(1.0, ratio))
            bpy.ops.object.modifier_apply(modifier=decimate_mod.name)

            proxy_obj.hide_set(True)
            proxy_obj.hide_render = True
            proxy_obj.display_type = 'WIRE'
            return proxy_obj
        except Exception as exc:
            print(f"❌ Proxy creation failed: {exc}")
            return None

    def _get_keyframe_target(self, props):
        acsm_bone_name = getattr(props, 'ACSm_bone', None)
        use_acsm_bone = (
            self.acsm_obj
            and self.acsm_obj.type == 'ARMATURE'
            and acsm_bone_name
            and acsm_bone_name != 'NONE'
        )
        if use_acsm_bone:
            return self.acsm_obj.pose.bones.get(acsm_bone_name)
        return self.acsm_obj

    def _apply_initial_pose_for_keyframe(self, target):
        """Ensure target is at stored initial transform before inserting frame 0 keyframe."""
        if not self.acsm_initial_local:
            return

        if hasattr(target, 'bone'):
            target.matrix = self.acsm_initial_local.copy()
        else:
            target.matrix_local = self.acsm_initial_local.copy()
        bpy.context.view_layer.update()

    def _cleanup_temp_objects(self):
        if self.proxy_obj and self.proxy_obj.name in bpy.data.objects:
            bpy.data.objects.remove(self.proxy_obj, do_unlink=True)
        self.proxy_obj = None

    def process_batch(self, batch_size=1000):
        """Process a batch of poses using the same logic as original"""
        if self.is_cancelled or self.processed_poses >= self.total_poses:
            return False

        batch_poses = list(islice(self.pose_iterator, batch_size))
        if not batch_poses:
            return False

        batch_start = self.processed_poses
        batch_end = batch_start + len(batch_poses)

        print(f"  Processing batch {batch_start:,} to {batch_end:,} ({len(batch_poses):,} poses)")
        
        valid_in_batch = 0
        total_pose_time = 0
        total_update_time = 0
        total_collision_time = 0
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
            pose_start = time.time()
            if self.acsm_obj.type == 'ARMATURE':
                acsm_bone_name = getattr(bpy.context.scene.collision_props, 'ACSm_bone', None)
                if acsm_bone_name:
                    pose_bone = self.acsm_obj.pose.bones.get(acsm_bone_name)
                    if pose_bone:
                        pose_bone.matrix = pose_matrix
            else:
                self.acsm_obj.matrix_local = pose_matrix

            # Update scene (same as original)
            update_start = time.time()
            bpy.context.view_layer.update()
            update_time = time.time() - update_start
            
            pose_time = time.time() - pose_start
            total_pose_time += pose_time
            total_update_time += update_time

            # Check collision using EXACT same method as original
            collision_start = time.time()
            collision = self._check_collision_original_method()
            collision_time = time.time() - collision_start
            total_collision_time += collision_time
            
            # Check collision using EXACT same method as original
            collision_start = time.time()
            collision = self._check_collision_original_method()
            collision_time = time.time() - collision_start
            
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
            
            # Debug timing (print average times every 5 batches)
            if hasattr(self, '_batch_count'):
                self._batch_count += 1
            else:
                self._batch_count = 1
                
            if self._batch_count % 5 == 0 and len(batch_poses) > 0:
                avg_pose_time = total_pose_time / len(batch_poses)
                avg_update_time = total_update_time / len(batch_poses)  
                avg_collision_time = total_collision_time / len(batch_poses)
                print(f"  Timing per pose - Pose calc: {avg_pose_time:.4f}s, Scene update: {avg_update_time:.4f}s, Collision: {avg_collision_time:.4f}s")

        return self.processed_poses < self.total_poses and not self.is_cancelled

    def _check_collision_original_method(self):
        """Optimized collision detection using cached mesh data.
        
        Instead of regenerating BVH from mesh every pose, we use cached vertex/face 
        data and transform it by the current distal world matrix.
        """
        import time
        start_time = time.time()
        
        # Optional debug short-circuit
        if self.props.debug_mode and self.props.turn_off_collisions:
            return False

        # Get current distal transform
        dist_world_matrix = self.dist_obj.matrix_world

        # Coarse AABB rejection (fast, uses bounding box)
        if self.use_aabb_precheck and self._prox_bounds:
            dist_min, dist_max = self._get_world_bounds(self.dist_obj)
            if not self._aabb_overlap(self._prox_bounds[0], self._prox_bounds[1], dist_min, dist_max, self.aabb_margin):
                print(f"    AABB early exit: {time.time() - start_time:.6f}s")
                return False

        bvh_start = time.time()
        
        # Convex hull pre-check (if enabled) - also uses cached data now!
        if self.use_convex_hull and self._prox_hull_bvh and self._dist_hull_local_verts:
            dist_hull_bvh = self._create_bvh_from_cached(
                self._np_hull_verts, 
                self._dist_hull_local_faces, 
                dist_world_matrix
            )
            if dist_hull_bvh and not self._prox_hull_bvh.overlap(dist_hull_bvh):
                print(f"    Hull early exit: {time.time() - start_time:.6f}s")
                return False  # Hulls don't overlap, no collision possible

        # Full mesh collision check using cached data
        if self._dist_local_verts and self._dist_local_faces:
            dist_bvh = self._create_bvh_from_cached(
                self._np_verts, 
                self._dist_local_faces, 
                dist_world_matrix
            )
        else:
            # Fallback to original method if caching failed
            collision_obj = self.proxy_obj if self.use_proxy_collision and self.proxy_obj else self.dist_obj
            dist_bvh = self._create_bvh_tree(collision_obj, dist_world_matrix)
        
        bvh_time = time.time() - bvh_start
        overlap_start = time.time()
        
        if not dist_bvh or not self.prox_bvh:
            return True  # Assume collision on error

        overlaps = self.prox_bvh.overlap(dist_bvh)
        
        overlap_time = time.time() - overlap_start
        total_time = time.time() - start_time
        
        # Only print timing for slow collisions (>0.01s)
        if total_time > 0.01:
            print(f"    Slow collision - BVH: {bvh_time:.4f}s, Overlap: {overlap_time:.4f}s, Total: {total_time:.4f}s")
        
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

        # Animation is created in the modal keyframe phase; avoid duplicate creation here

        return success



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
        
        # Reset to initial pose and keyframe at frame 0
        self.reset_acsm_to_initial()
        self._apply_initial_pose_for_keyframe(self.keyframe_target)
        self.keyframe_target.keyframe_insert(data_path="location", frame=0)
        self.keyframe_target.keyframe_insert(data_path="rotation_euler", frame=0)
        
        # Update UI
        props.calculation_progress = 0.0
        props.time_remaining = "Creating animation keyframes..."
        
        return True

    def process_keyframe_batch(self, props):
        """
        Process one batch of keyframes.
        Collects data in memory first, then writes to Blender 5.0 structures at the end.
        """
        if not self.is_creating_keyframes or not self.keyframe_target:
            return False
            
        total_poses = len(self.valid_poses)
        batch_start = self.keyframe_batch_index * self.keyframe_batch_size
        
        # --- 1. INITIALIZATION (Run once on first batch) ---
        if self.keyframe_batch_index == 0:
            self.collected_data = {
                "location": {0: [], 1: [], 2: []},
                "rotation_euler": {0: [], 1: [], 2: []},
                # If using quaternions, add rotation_quaternion here
                '["input_rot_x"]': {0: []},
                '["input_rot_y"]': {0: []},
                '["input_rot_z"]': {0: []},
                '["input_trans_x"]': {0: []},
                '["input_trans_y"]': {0: []},
                '["input_trans_z"]': {0: []},
            }
            self.frames = []

        # --- 2. FINISH CONDITION ---
        if batch_start >= total_poses:
            self.apply_collected_keyframes_5_0() # <--- New 5.0 Writer
            self.finish_keyframe_creation(props)
            return False
            
        batch_end = min(batch_start + self.keyframe_batch_size, total_poses)
        batch_poses = self.valid_poses[batch_start:batch_end]
        
        # Update UI
        progress = (batch_end / total_poses) * 100
        props.calculation_progress = progress
        props.time_remaining = f"Calculating data: {batch_end:,}/{total_poses:,} ({progress:.1f}%)"
        
        # --- 3. FAST BATCH PROCESSING (Pure Math, No Scene Updates) ---
        # Determine rotation mode once
        rot_mode = self.keyframe_target.rotation_mode
        
        for i, pose_data in enumerate(batch_poses):
            frame = batch_start + i + 1
            self.frames.append(frame)
            
            # Decompose Matrix
            matrix = pose_data['pose_matrix']
            trans = matrix.to_translation()
            rot = matrix.to_euler(rot_mode) # Force euler based on obj settings

            # Store Location
            self.collected_data["location"][0].append(trans.x)
            self.collected_data["location"][1].append(trans.y)
            self.collected_data["location"][2].append(trans.z)

            # Store Rotation
            self.collected_data["rotation_euler"][0].append(rot.x)
            self.collected_data["rotation_euler"][1].append(rot.y)
            self.collected_data["rotation_euler"][2].append(rot.z)

            # Store Custom Props
            self.collected_data['["input_rot_x"]'][0].append(pose_data['rx'])
            self.collected_data['["input_rot_y"]'][0].append(pose_data['ry'])
            self.collected_data['["input_rot_z"]'][0].append(pose_data['rz'])
            
            self.collected_data['["input_trans_x"]'][0].append(pose_data['tx'])
            self.collected_data['["input_trans_y"]'][0].append(pose_data['ty'])
            self.collected_data['["input_trans_z"]'][0].append(pose_data['tz'])

        self.keyframe_batch_index += 1
        return True 

    def apply_collected_keyframes_5_0(self):
        """
        Writes data using Blender 5.0 Layered Animation API (Project Baklava).
        Hierarchy: Action -> Slot -> Layer -> Strip -> ChannelBag -> FCurves
        """
        obj = self.keyframe_target
        
        if not self.frames:
            return

        # 1. Ensure Animation Data
        if not obj.animation_data:
            obj.animation_data_create()
        
        # 2. Get or Create Action
        action = obj.animation_data.action
        if not action:
            action = bpy.data.actions.new(name=f"{obj.name}_Action")
            obj.animation_data.action = action
        
        # 3. Get or Create Slot (The "User" ID)
        if len(action.slots) == 0:
            # id_type='OBJECT' is required in 5.0
            slot = action.slots.new(id_type='OBJECT', name=obj.name)
        else:
            slot = action.slots[0]

        # 4. Bind Slot to Object (Note: 'action_slot' is the new property name)
        obj.animation_data.action_slot = slot

        # 5. Get or Create Layer
        if len(action.layers) == 0:
            layer = action.layers.new(name="Base Layer")
        else:
            layer = action.layers[0]

        # 6. Get or Create Strip
        if len(layer.strips) == 0:
            strip = layer.strips.new(type='KEYFRAME', start=0)
        else:
            strip = layer.strips[0]

        # 7. Get the ChannelBag
        # This retrieves the specific bucket of curves for this slot on this strip
        channel_bag = strip.channelbag(slot, ensure=True)
        fcurves_collection = channel_bag.fcurves

        # 8. Write Data (foreach_set optimization)
        frame_count = len(self.frames)
        
        for data_path, indices in self.collected_data.items():
            for array_index, values in indices.items():
                
                # Find or New F-Curve in the Channel Bag
                fcurve = fcurves_collection.find(data_path=data_path, index=array_index)
                if not fcurve:
                    fcurve = fcurves_collection.new(data_path=data_path, index=array_index)
                
                # --- CRITICAL FIX START ---
                # Clear existing points so we don't get an array length mismatch
                fcurve.keyframe_points.clear()
                # --- CRITICAL FIX END ---
                
                # Prepare buffer: [frame, val, frame, val...]
                kfp_flat = [0.0] * (frame_count * 2)
                kfp_flat[0::2] = self.frames
                kfp_flat[1::2] = values 
                
                # Write to C memory
                fcurve.keyframe_points.add(frame_count)
                fcurve.keyframe_points.foreach_set("co", kfp_flat)
                
                fcurve.update() 

        # Cleanup memory
        self.collected_data = None
        self.frames = None
        
        # Final Update
        bpy.context.view_layer.update()
        print("Blender 5.0 Keyframes Applied Successfully.")

    def finish_keyframe_creation(self, props):
        """Clean up after keyframe creation"""
        if hasattr(self, 'original_use_keyframe_insert_auto'):
            scene = bpy.context.scene
            scene.tool_settings.use_keyframe_insert_auto = self.original_use_keyframe_insert_auto
        
        # Re-keyframe frame 0 with initial pose after batch keyframing (since apply_collected_keyframes_5_0 clears all keyframes)
        self.reset_acsm_to_initial()
        self._apply_initial_pose_for_keyframe(self.keyframe_target)
        self.keyframe_target.keyframe_insert(data_path="location", frame=0)
        self.keyframe_target.keyframe_insert(data_path="rotation_euler", frame=0)
        
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
                batch_size = max(1, props.batch_size)  # Use user-specified batch size
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
            self._processor._cleanup_temp_objects()
        
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
            self._processor._cleanup_temp_objects()
        
        if self._timer:
            context.window_manager.event_timer_remove(self._timer)
            self._timer = None
        
        props.is_calculating = False
        props.calculation_progress = 100.0

    def cancel(self, context):
        """Operator cancel method"""
        self.cancel_processing(context)
        return {'CANCELLED'}
