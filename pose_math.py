"""
pose_math.py — JCS pose calculation functions for BlenderROMFinder.

Computes the local-to-ACSf orientation matrix for a given set of
Flexion/Extension, Adduction/Abduction, and Long-Axis Rotation angles,
then combines it with optional translations to produce a final pose matrix
for the ACSm object or bone.
"""

import math
from mathutils import Matrix, Vector


# ---------------------------------------------------------------------------
# Core JCS orientation
# ---------------------------------------------------------------------------

def calculate_jcs_orientation_matrix_local_to_acsf(rz_deg, ry_deg, rx_deg, adab_mode_is_isb_standard):
    """Calculate the 3-DOF JCS orientation matrix, local to ACSf.

    Parameters
    ----------
    rz_deg : float
        Flexion/Extension angle (degrees, around ACSf Z).
    ry_deg : float
        Adduction/Abduction angle (degrees).
    rx_deg : float
        Long-Axis Rotation angle (degrees, around local X after FE + AD/AB).
    adab_mode_is_isb_standard : bool
        If True, use ISB floating-axis convention for AD/AB.
        If False, use local-Y (Intuitive / M&G Hinge).
    """
    current_transform = Matrix.Identity(4)
    axis_fe = Vector((0, 0, 1))
    mat_fe = Matrix.Rotation(math.radians(rz_deg), 4, axis_fe)
    current_transform = current_transform @ mat_fe

    if adab_mode_is_isb_standard:
        x_axis_of_acsm_after_fe_in_acsf_frame = current_transform.col[0].to_3d().normalized()
        axis_adab_acsf = axis_fe.cross(x_axis_of_acsm_after_fe_in_acsf_frame)
        if axis_adab_acsf.length < 1e-8:
            axis_adab_acsf = Vector((0, 1, 0))
        else:
            axis_adab_acsf.normalize()
        mat_adab = Matrix.Rotation(math.radians(ry_deg), 4, axis_adab_acsf)
        current_transform = mat_adab @ current_transform  # ISB: pre-multiply
    else:
        axis_adab_local = Vector((0, 1, 0))
        mat_adab = Matrix.Rotation(math.radians(ry_deg), 4, axis_adab_local)
        current_transform = current_transform @ mat_adab  # post-multiply for local

    axis_lar_local = Vector((1, 0, 0))
    mat_lar = Matrix.Rotation(math.radians(rx_deg), 4, axis_lar_local)
    current_transform = current_transform @ mat_lar  # post-multiply for local

    return current_transform


# ---------------------------------------------------------------------------
# Helper: world-delta to local translation
# ---------------------------------------------------------------------------

def _add_world_delta_to_local(base_pose_local_matrix, world_delta, acsm_obj):
    """Convert a world-space delta vector into the local translation to add
    to *base_pose_local_matrix*.

    Ensures the final world movement equals *world_delta* regardless of
    parent scaling.  Falls back to simple local addition when *acsm_obj*
    or its parent is unavailable.
    """
    final = base_pose_local_matrix.copy()
    try:
        if acsm_obj is None:
            final.translation = base_pose_local_matrix.to_translation() + world_delta
            return final

        parent = acsm_obj.parent
        parent_inv = parent.matrix_world.inverted() if parent else Matrix.Identity(4)

        delta_local = parent_inv.to_3x3() @ world_delta
        final.translation = base_pose_local_matrix.to_translation() + delta_local
        return final
    except Exception:
        final.translation = base_pose_local_matrix.to_translation() + world_delta
        return final


# ---------------------------------------------------------------------------
# Per-mode pose calculators
# ---------------------------------------------------------------------------

def calculate_pose_for_isb_standard_mode(rx_deg, ry_deg, rz_deg, tx, ty, tz,
                                          props, acsm_initial_local_matrix,
                                          acsm_obj=None, acsf_obj=None):
    jcs_orientation_matrix = calculate_jcs_orientation_matrix_local_to_acsf(
        rz_deg, ry_deg, rx_deg, adab_mode_is_isb_standard=True)

    base_pose_with_jcs = acsm_initial_local_matrix @ jcs_orientation_matrix

    if tx != 0 or ty != 0 or tz != 0:
        if acsm_obj is not None:
            parent_world = acsm_obj.parent.matrix_world if acsm_obj.parent else Matrix.Identity(4)
            acsm_world = parent_world @ base_pose_with_jcs
            q = acsm_world.to_quaternion()
            axis_x_world = q @ Vector((1, 0, 0))
            axis_y_world = q @ Vector((0, 1, 0))
            axis_z_world = q @ Vector((0, 0, 1))
            world_delta = (axis_x_world * tx) + (axis_y_world * ty) + (axis_z_world * tz)
        else:
            world_delta = base_pose_with_jcs.to_3x3() @ Vector((tx, ty, tz))

        final_matrix = _add_world_delta_to_local(base_pose_with_jcs, world_delta, acsm_obj)
    else:
        final_matrix = base_pose_with_jcs
    return final_matrix


def calculate_pose_for_intuitive_mode(rx_deg, ry_deg, rz_deg, tx, ty, tz,
                                       props, acsm_initial_local_matrix,
                                       acsm_obj=None, acsf_obj=None):
    jcs_orientation_matrix = calculate_jcs_orientation_matrix_local_to_acsf(
        rz_deg, ry_deg, rx_deg, adab_mode_is_isb_standard=False)

    base_pose_with_jcs = acsm_initial_local_matrix @ jcs_orientation_matrix

    if tx != 0 or ty != 0 or tz != 0:
        if acsm_obj is not None:
            parent_world = acsm_obj.parent.matrix_world if acsm_obj.parent else Matrix.Identity(4)
            acsm_world = parent_world @ base_pose_with_jcs
            q = acsm_world.to_quaternion()
            axis_x_world = q @ Vector((1, 0, 0))
            axis_y_world = q @ Vector((0, 1, 0))
            axis_z_world = q @ Vector((0, 0, 1))
            world_delta = (axis_x_world * tx) + (axis_y_world * ty) + (axis_z_world * tz)
        else:
            world_delta = base_pose_with_jcs.to_3x3() @ Vector((tx, ty, tz))

        final_matrix = _add_world_delta_to_local(base_pose_with_jcs, world_delta, acsm_obj)
    else:
        final_matrix = base_pose_with_jcs
    return final_matrix


def calculate_pose_for_mg_hinge_mode(rx_deg, ry_deg, rz_deg, tx, ty, tz,
                                      props, acsm_initial_local_matrix):
    jcs_orientation_matrix = calculate_jcs_orientation_matrix_local_to_acsf(
        rz_deg, ry_deg, rx_deg, adab_mode_is_isb_standard=False)

    base_pose_with_jcs = acsm_initial_local_matrix @ jcs_orientation_matrix

    if tx != 0 or ty != 0 or tz != 0:
        final_matrix = base_pose_with_jcs.copy()

        initial_prism_axis_for_tx = Vector((1, 0, 0))
        initial_prism_axis_for_ty = Vector((0, 1, 0))
        initial_prism_axis_for_tz = Vector((0, 0, 1))

        fe_angle_rad = math.radians(rz_deg)
        fe_only_rot_matrix_in_acsf = Matrix.Rotation(fe_angle_rad, 4, Vector((0, 0, 1)))

        dir_tx_in_acsf = (fe_only_rot_matrix_in_acsf @ initial_prism_axis_for_tx.to_4d()).to_3d()
        dir_ty_in_acsf = (fe_only_rot_matrix_in_acsf @ initial_prism_axis_for_ty.to_4d()).to_3d()
        dir_tz_in_acsf = (fe_only_rot_matrix_in_acsf @ initial_prism_axis_for_tz.to_4d()).to_3d()

        translation_offset_in_acsf = (
            (dir_tx_in_acsf * tx) +
            (dir_ty_in_acsf * ty) +
            (dir_tz_in_acsf * tz)
        )

        acsf_obj = None
        acsm_obj = None
        if props is not None:
            acsf_obj = getattr(props, 'ACSf_object', None)
            acsm_obj = getattr(props, 'ACSm_object', None)

        if acsf_obj is not None:
            acsf_world = acsf_obj.matrix_world
            q_acsf = acsf_world.to_quaternion()
            world_delta = q_acsf @ translation_offset_in_acsf
            final_matrix = _add_world_delta_to_local(base_pose_with_jcs, world_delta, acsm_obj)
        else:
            final_matrix.translation = acsm_initial_local_matrix.to_translation() + translation_offset_in_acsf
    else:
        final_matrix = base_pose_with_jcs
    return final_matrix
