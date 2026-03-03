"""
collision.py — Self-contained collision detection for BlenderROMFinder.

Provides `CollisionDetector`, a reusable class that implements a 3-tier
collision cascade:

    1. **Convex-hull pre-check** — if the convex hulls of proximal and distal
       meshes do not overlap, the actual meshes definitely don't either.
       Early-out → *valid pose*.

    2. **Penetration-sample pre-check** — ray-parity test on evenly-sampled
       distal surface vertices.  If any vertex is inside the proximal mesh,
       there is *definite* interpenetration.  Early-out → *invalid pose*.

    3. **Full BVH triangle overlap** — definitive triangle-pair test for the
       ambiguous "hulls overlap but no sampled vertex inside" case.
"""

import bmesh
import mathutils
import numpy as np
from mathutils import Matrix, Vector

import bpy


class CollisionDetector:
    """Stateful collision detector for a fixed proximal / moving distal mesh pair.

    Typical usage::

        detector = CollisionDetector()
        detector.initialize(prox_obj, dist_obj, use_convex_hull=True,
                            penetration_sample_count=10000)
        ...
        for pose in poses:
            apply_pose(...)
            bpy.context.view_layer.update()
            if detector.check(dist_obj.matrix_world):
                ...  # collision
        detector.cleanup()
    """

    def __init__(self):
        # References
        self._prox_obj = None
        self._dist_obj = None   # original distal (used only for matrix_world fallback)
        self._proxy_obj = None

        # Static proximal BVH (world-space, computed once)
        self._prox_bvh = None

        # Convex hull data
        self._use_convex_hull = False
        self._prox_hull_bvh = None
        self._np_hull_verts = None          # distal hull verts (Nx4 homogeneous, local)
        self._dist_hull_local_faces = None

        # Cached distal mesh data (local coords)
        self._dist_local_verts = None
        self._dist_local_faces = None
        self._np_verts = None               # full vertex array (Nx4 homogeneous)

        # Penetration-sample data
        self._np_sample_verts = None        # subsampled distal surface verts (Nx4)
        self._penetration_sample_count = 5000
        self._penetration_definite_threshold = 10  # min inside-samples to trust T2 early-out

        # Proxy mesh settings
        self._use_proxy = False
        self._proxy_decimate_ratio = 1.0

        # Debug
        self._debug = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def initialize(self, prox_obj, dist_obj, *,
                   use_convex_hull=True,
                   use_proxy=False,
                   proxy_decimate_ratio=0.25,
                   penetration_sample_count=10000,
                   penetration_definite_threshold=20,
                   debug=False):
        """Build all static data structures.

        Parameters
        ----------
        prox_obj : bpy.types.Object
            Proximal (fixed) mesh object.
        dist_obj : bpy.types.Object
            Distal (moving) mesh object.
        use_convex_hull : bool
            Enable the convex-hull pre-check (tier 1).
        use_proxy : bool
            If True, create a decimated copy of *dist_obj* and use it for
            all collision geometry instead.
        proxy_decimate_ratio : float
            Face-ratio for the proxy DECIMATE modifier (0.05 – 1.0).
        penetration_sample_count : int
            Number of evenly-sampled distal surface vertices for the
            ray-parity penetration pre-check (tier 2).
        penetration_definite_threshold : int
            Minimum number of sampled vertices that must test as "inside" before
            tier 2 short-circuits to *collision*.  Cases where inside-count is
            between 3 and this value are considered **ambiguous** and fall through
            to the full tier-3 BVH test, eliminating architecture-dependent flips.
        debug : bool
            Print timing / diagnostic messages.
        """
        self._prox_obj = prox_obj
        self._dist_obj = dist_obj
        self._debug = debug
        self._penetration_sample_count = max(100, penetration_sample_count)
        self._penetration_definite_threshold = max(3, penetration_definite_threshold)
        self._use_convex_hull = use_convex_hull
        self._use_proxy = use_proxy
        self._proxy_decimate_ratio = proxy_decimate_ratio

        # Clean up leftovers from a prior run
        self.cleanup()

        # 1. Static proximal BVH (world-space, built once)
        self._prox_bvh = self._create_bvh_tree(prox_obj)
        if not self._prox_bvh:
            raise ValueError("Failed to create proximal BVH")

        # 2. Optional proxy mesh
        if self._use_proxy:
            self._proxy_obj = self._create_proxy_object(dist_obj, self._proxy_decimate_ratio)
            if self._proxy_obj is None:
                self._use_proxy = False  # fallback to full mesh

        collision_source = self._proxy_obj if (self._use_proxy and self._proxy_obj) else dist_obj

        # 3. Cache distal mesh vertices/faces + NumPy arrays
        self._cache_mesh_data(collision_source)

        # 4. Convex hulls
        if self._use_convex_hull:
            self._setup_convex_hulls(prox_obj, collision_source)

        if self._debug:
            print(f"[CollisionDetector] Initialised: convex_hull={self._use_convex_hull}, "
                  f"proxy={self._use_proxy}, samples={self._penetration_sample_count}, "
                  f"definite_threshold={self._penetration_definite_threshold}")

    def check(self, dist_world_matrix):
        """Run the 3-tier collision cascade.

        Parameters
        ----------
        dist_world_matrix : mathutils.Matrix
            Current world-space transform of the distal object.

        Returns
        -------
        bool
            True if collision detected, False if the pose is valid (no collision).
        """
        # --- Tier 1: convex-hull non-overlap → definitely valid ---
        if self._use_convex_hull and self._prox_hull_bvh and self._np_hull_verts is not None:
            dist_hull_bvh = self._create_bvh_from_cached(
                self._np_hull_verts,
                self._dist_hull_local_faces,
                dist_world_matrix,
            )
            if dist_hull_bvh and not self._prox_hull_bvh.overlap(dist_hull_bvh):
                return False  # hulls don't touch → no collision possible

        # --- Tier 2: sampled-vertex penetration → definitely invalid ---
        # Returns: 1 = definite collision (many samples inside, skip T3)
        #          0 = ambiguous (few samples inside, fall through to T3)
        #         -1 = no penetration found by sampling (fall through to T3)
        if self._np_sample_verts is not None:
            t2_result = self._positive_penetration_check(dist_world_matrix)
            if t2_result == 1:
                return True  # high-confidence: many samples inside proximal
            # t2_result 0 (ambiguous) or -1 (clean): both fall through to T3

        # --- Tier 3: full BVH triangle overlap (definitive) ---
        if self._np_verts is not None and self._dist_local_faces:
            dist_bvh = self._create_bvh_from_cached(
                self._np_verts,
                self._dist_local_faces,
                dist_world_matrix,
            )
        else:
            # Fallback: build from mesh object directly
            collision_obj = self._proxy_obj if (self._use_proxy and self._proxy_obj) else self._dist_obj
            dist_bvh = self._create_bvh_tree(collision_obj, dist_world_matrix)

        if not dist_bvh or not self._prox_bvh:
            return True  # assume collision on error

        return len(self._prox_bvh.overlap(dist_bvh)) > 0

    def cleanup(self):
        """Remove any temporary Blender objects (proxy mesh)."""
        if self._proxy_obj and self._proxy_obj.name in bpy.data.objects:
            bpy.data.objects.remove(self._proxy_obj, do_unlink=True)
        self._proxy_obj = None

    # ------------------------------------------------------------------
    # Internal: BVH helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _create_bvh_tree(obj, transform_matrix=None):
        """Create a BVH tree from a Blender mesh object."""
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

    @staticmethod
    def _create_bvh_from_cached(np_verts, faces, transform_matrix):
        """Transform cached Nx4 NumPy vertices by *transform_matrix* and build a BVH."""
        np_matrix = np.array(transform_matrix)
        transformed = (np_matrix @ np_verts.T)[:3].T.tolist()
        return mathutils.bvhtree.BVHTree.FromPolygons(transformed, faces)

    # ------------------------------------------------------------------
    # Internal: mesh caching
    # ------------------------------------------------------------------

    def _cache_mesh_data(self, obj):
        """Extract vertices/faces once for fast per-pose BVH creation."""
        mesh = obj.to_mesh()

        self._dist_local_verts = [v.co.copy() for v in mesh.vertices]
        self._dist_local_faces = [tuple(p.vertices) for p in mesh.polygons]

        # Full vertex array (Nx4 homogeneous)
        self._np_verts = np.array(
            [[v.co.x, v.co.y, v.co.z, 1.0] for v in mesh.vertices]
        )

        # Penetration-sample subset
        n_verts = len(mesh.vertices)
        n_samples = self._penetration_sample_count
        if n_verts <= n_samples:
            sample_indices = range(n_verts)
        else:
            stride = n_verts // n_samples
            sample_indices = range(0, n_verts, stride)

        self._np_sample_verts = np.array(
            [[mesh.vertices[i].co.x, mesh.vertices[i].co.y, mesh.vertices[i].co.z, 1.0]
             for i in sample_indices]
        )

        obj.to_mesh_clear()

        if self._debug:
            print(f"  Cached distal mesh: {len(self._dist_local_verts)} verts, "
                  f"{len(self._dist_local_faces)} faces | "
                  f"{len(self._np_sample_verts)} penetration samples")

    # ------------------------------------------------------------------
    # Internal: convex hulls
    # ------------------------------------------------------------------

    def _setup_convex_hulls(self, prox_obj, dist_obj):
        """Build static proximal hull BVH + cache distal hull in local space."""
        # Proximal hull (static, world-space BVH)
        prox_verts, prox_faces = self._compute_convex_hull_data(prox_obj)
        if prox_verts and prox_faces:
            world_verts = [prox_obj.matrix_world @ v for v in prox_verts]
            self._prox_hull_bvh = mathutils.bvhtree.BVHTree.FromPolygons(world_verts, prox_faces)
        else:
            self._use_convex_hull = False
            return

        # Distal hull (local-space cache → transformed per-pose)
        dist_verts, dist_faces = self._compute_convex_hull_data(dist_obj)
        if dist_verts and dist_faces:
            self._dist_hull_local_faces = dist_faces
            self._np_hull_verts = np.array(
                [[v.x, v.y, v.z, 1.0] for v in dist_verts]
            )
        else:
            self._use_convex_hull = False

    @staticmethod
    def _compute_convex_hull_data(obj):
        """Return (verts, faces) of the convex hull in object-local space."""
        try:
            bm = bmesh.new()
            mesh = obj.to_mesh()
            bm.from_mesh(mesh)
            obj.to_mesh_clear()

            result = bmesh.ops.convex_hull(bm, input=bm.verts)

            interior = result.get('geom_interior', [])
            if interior:
                bmesh.ops.delete(bm, geom=interior, context='VERTS')
            unused = result.get('geom_unused', [])
            if unused:
                bmesh.ops.delete(bm, geom=unused, context='VERTS')

            bm.verts.ensure_lookup_table()
            bm.faces.ensure_lookup_table()

            verts = [v.co.copy() for v in bm.verts]
            faces = [tuple(v.index for v in f.verts) for f in bm.faces]
            bm.free()
            return verts, faces
        except Exception as e:
            print(f"Convex hull computation failed: {e}")
            return None, None

    # ------------------------------------------------------------------
    # Internal: penetration sampling (ray-parity / crossing-number)
    # ------------------------------------------------------------------

    # Diverse ray directions to avoid systematic edge-cases on axis-aligned meshes.
    # Defined once as a class constant so they are not reconstructed per pose.
    _PARITY_RAY_DIRS = [
        Vector((1.0,  1e-5,  1e-5)).normalized(),
        Vector((-1.0, 1e-5,  2e-5)).normalized(),
        Vector((1e-5, 1.0,   1e-5)).normalized(),
        Vector((1e-5, -1.0,  2e-5)).normalized(),
        Vector((1e-5, 1e-5,  1.0)).normalized(),
        Vector((2e-5, 1e-5, -1.0)).normalized(),
    ]

    def _positive_penetration_check(self, dist_world_matrix):
        """Check how many sampled distal vertices are inside the proximal mesh.

        Uses ray-parity: cast a ray from each sample point, count surface
        crossings. Odd count → point is inside.

        Returns
        -------
        int
             1  — definite collision: inside_count >= ``_penetration_definite_threshold``.
                  Caller can short-circuit to *collision* with high confidence.
             0  — ambiguous: inside_count is between 3 and the definite threshold.
                  Floating-point parity flips on different architectures can
                  produce these; caller should fall through to Tier 3.
            -1  — no penetration found (inside_count < 3).
        """
        if self._np_sample_verts is None or self._prox_bvh is None:
            return -1

        np_matrix = np.array(dist_world_matrix)
        world_pts = (np_matrix @ self._np_sample_verts.T)[:3].T

        ray_dirs = self._PARITY_RAY_DIRS
        EPS = 1e-4  # Increased epsilon for robustness
        MAX_CROSSINGS = 100
        inside_count = 0

        for i, pt_xyz in enumerate(world_pts):
            origin = Vector((float(pt_xyz[0]), float(pt_xyz[1]), float(pt_xyz[2])))
            
            # Deterministically select a ray direction based on point index
            # This ensures the same point always uses the same ray, preventing
            # run-to-run variation while avoiding systematic alignment issues.
            ray_dir = ray_dirs[i % len(ray_dirs)]
            
            hit_count = 0
            curr_origin = origin
            
            while hit_count < MAX_CROSSINGS:
                hit_loc, _, _, _ = self._prox_bvh.ray_cast(curr_origin, ray_dir)
                if hit_loc is None:
                    break
                hit_count += 1
                # Advance safely past the hit
                curr_origin = hit_loc + ray_dir * EPS

            if hit_count % 2 == 1:  # odd crossings -> point is inside
                inside_count += 1
                if inside_count >= self._penetration_definite_threshold:
                    return 1  # definite — many samples unambiguously inside

        if inside_count >= 3:
            return 0   # ambiguous — some samples inside but below definite threshold
        return -1      # no meaningful penetration detected

    # ------------------------------------------------------------------
    # Internal: proxy mesh
    # ------------------------------------------------------------------

    @staticmethod
    def _create_proxy_object(source_obj, ratio):
        """Duplicate *source_obj* and apply a DECIMATE modifier."""
        if ratio >= 0.999:
            return None
        try:
            if bpy.ops.object.mode_set.poll():
                bpy.ops.object.mode_set(mode='OBJECT')

            bpy.ops.object.select_all(action='DESELECT')
            source_obj.select_set(True)
            bpy.context.view_layer.objects.active = source_obj
            bpy.ops.object.duplicate()
            proxy = bpy.context.active_object
            proxy.name = f"{source_obj.name}_romf_proxy"

            mod = proxy.modifiers.new(name="ROMF_Decimate", type='DECIMATE')
            mod.ratio = max(0.05, min(1.0, ratio))
            bpy.ops.object.modifier_apply(modifier=mod.name)

            proxy.hide_set(True)
            proxy.hide_render = True
            proxy.display_type = 'WIRE'
            return proxy
        except Exception as exc:
            print(f"Proxy creation failed: {exc}")
            return None
