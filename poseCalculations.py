import math
from mathutils import Matrix, Vector

# --- JCS Calculation Helper Methods (standalone) ---
def get_jcs_rotation_part(rz_deg, ry_deg, rx_deg, props):
    """Calculates the 3-DOF JCS orientation matrix, local to ACSf."""
    current_transform = Matrix.Identity(4) # Start with ACSm aligned with ACSf (local space)

    # 1. Flexion/Extension (around ACSf's local Z)
    axis_fe = Vector((0, 0, 1)) # ACSf's local Z
    mat_fe = Matrix.Rotation(math.radians(rz_deg), 4, axis_fe)
    current_transform = current_transform @ mat_fe

    # 2. Adduction/Abduction - Logic depends on props.rotation_mode_enum
    axis_adab = Vector((0,1,0)) # Initialize / Default for safety
    operational_mode = getattr(props, 'rotation_mode_enum', 'ISB_STANDARD')
    if operational_mode == 'ISB_STANDARD':
        x_axis_of_acsm_after_fe_in_acsf_frame = current_transform.col[0].to_3d().normalized()
        axis_adab = axis_fe.cross(x_axis_of_acsm_after_fe_in_acsf_frame)
        if axis_adab.length < 1e-8:
            axis_adab = Vector((0, 1, 0))
        else:
            axis_adab.normalize()
    elif operational_mode == 'INTUITIVE' or operational_mode == 'MG_HINGE':
        axis_adab = current_transform.col[1].to_3d().normalized()
        if axis_adab.length < 1e-8:
            axis_adab = Vector((0,1,0))
    else:
        axis_adab = Vector((0,1,0))
    mat_adab = Matrix.Rotation(math.radians(ry_deg), 4, axis_adab)
    current_transform = current_transform @ mat_adab
    # 3. Long-Axis Rotation (around ACSm's *new* local X, after FE and AD/AB)
    axis_lar = Vector((1, 0, 0))
    mat_lar = Matrix.Rotation(math.radians(rx_deg), 4, axis_lar)
    current_transform = current_transform @ mat_lar
    return current_transform

def calculate_jcs_orientation_matrix_local_to_acsf(rz_deg, ry_deg, rx_deg, adab_mode_is_isb_standard):
    current_transform = Matrix.Identity(4)
    axis_fe = Vector((0, 0, 1))
    mat_fe = Matrix.Rotation(math.radians(rz_deg), 4, axis_fe)
    current_transform = current_transform @ mat_fe
    if adab_mode_is_isb_standard:
        x_axis_of_acsm_after_fe_in_acsf_frame = current_transform.col[0].to_3d().normalized()
        axis_adab_acsf = axis_fe.cross(x_axis_of_acsm_after_fe_in_acsf_frame)
        if axis_adab_acsf.length < 1e-8:
            # This case implies x_axis_of_acsm_after_fe is parallel to axis_fe (ACSf Z).
            # This can happen if FE is +/-90 and ACSm X aligns with ACSf Z, which is unusual.
            # Defaulting to ACSf Y as a fallback, though ISB definition might need specific handling here.
            axis_adab_acsf = Vector((0, 1, 0))
        else:
            axis_adab_acsf.normalize()
        mat_adab = Matrix.Rotation(math.radians(ry_deg), 4, axis_adab_acsf)
        current_transform = mat_adab @ current_transform # ISB: Pre-multiply by world-axis rotation
    else:
        # Non-ISB modes: rotate around local Y-axis.
        axis_adab_local = Vector((0, 1, 0))
        mat_adab = Matrix.Rotation(math.radians(ry_deg), 4, axis_adab_local) # Reverted: Use ry_deg
        current_transform = current_transform @ mat_adab # Post-multiply for local rotation
    # Now, get the new X axis after FE and AD/AB for LAR
    # LAR is always around the segment's local X-axis after prior rotations.
    axis_lar_local = Vector((1, 0, 0))
    mat_lar = Matrix.Rotation(math.radians(rx_deg), 4, axis_lar_local)
    current_transform = current_transform @ mat_lar # Post-multiply for local rotation
    # Debug printout for each pose
    #eul = current_transform.to_euler('XYZ')
    #print(f"[DEBUG] Inputs: FE(Z)={rz_deg:.1f}, AD/AB(Y)={ry_deg:.1f}, LAR(X)={rx_deg:.1f} | Blender Euler: X={math.degrees(eul.x):.1f}, Y={math.degrees(eul.y):.1f}, Z={math.degrees(eul.z):.1f}")
    #print(f"[DEBUG] Matrix X: {current_transform.col[0].to_3d().normalized()}")
    #print(f"[DEBUG] Matrix Y: {current_transform.col[1].to_3d().normalized()}")
    #print(f"[DEBUG] Matrix Z: {current_transform.col[2].to_3d().normalized()}")
    return current_transform

def calculate_pose_for_isb_standard_mode(rx_deg, ry_deg, rz_deg, tx, ty, tz, props, acsm_initial_local_matrix):
    jcs_orientation_matrix = calculate_jcs_orientation_matrix_local_to_acsf(rz_deg, ry_deg, rx_deg, adab_mode_is_isb_standard=True)
    
    # Apply JCS rotations relative to the initial pose of ACSm
    base_pose_with_jcs = acsm_initial_local_matrix @ jcs_orientation_matrix

    if tx != 0 or ty != 0 or tz != 0:
        translation_vec_local_to_rotated_acsm = Vector((tx, ty, tz))
        translation_matrix_offset = Matrix.Translation(translation_vec_local_to_rotated_acsm)
        # Apply translation offset in the new local frame of ACSm
        final_matrix = base_pose_with_jcs @ translation_matrix_offset
    else:
        final_matrix = base_pose_with_jcs
    return final_matrix

def calculate_pose_for_intuitive_mode(rx_deg, ry_deg, rz_deg, tx, ty, tz, props, acsm_initial_local_matrix):
    jcs_orientation_matrix = calculate_jcs_orientation_matrix_local_to_acsf(rz_deg, ry_deg, rx_deg, adab_mode_is_isb_standard=False)

    # Apply JCS rotations relative to the initial pose of ACSm
    base_pose_with_jcs = acsm_initial_local_matrix @ jcs_orientation_matrix

    if tx != 0 or ty != 0 or tz != 0:
        translation_vec_local_to_rotated_acsm = Vector((tx, ty, tz))
        translation_matrix_offset = Matrix.Translation(translation_vec_local_to_rotated_acsm)
        # Apply translation offset in the new local frame of ACSm
        final_matrix = base_pose_with_jcs @ translation_matrix_offset
    else:
        final_matrix = base_pose_with_jcs
    return final_matrix

def calculate_pose_for_mg_hinge_mode(rx_deg, ry_deg, rz_deg, tx, ty, tz, props, acsm_initial_local_matrix):
    jcs_orientation_matrix = calculate_jcs_orientation_matrix_local_to_acsf(rz_deg, ry_deg, rx_deg, adab_mode_is_isb_standard=False)

    # Apply JCS rotations relative to the initial pose of ACSm
    base_pose_with_jcs = acsm_initial_local_matrix @ jcs_orientation_matrix

    if tx != 0 or ty != 0 or tz != 0:
        # MG_HINGE mode has a special translation logic.
        # The rotation is from base_pose_with_jcs.
        # The translation starts from acsm_initial_local_matrix.to_translation()
        # and adds an offset calculated based on FE angle in ACSf frame.
        final_matrix = base_pose_with_jcs.copy() # This has the correct rotation and initial translation part

        initial_prism_axis_for_tx = Vector((1,0,0))
        initial_prism_axis_for_ty = Vector((0,1,0))
        initial_prism_axis_for_tz = Vector((0,0,1))

        fe_angle_rad = math.radians(rz_deg)
        # This rotation is around ACSf's Z axis, used to determine translation directions in ACSf
        fe_only_rot_matrix_in_acsf = Matrix.Rotation(fe_angle_rad, 4, Vector((0,0,1)))

        # Calculate translation directions in ACSf frame, influenced by FE angle
        dir_tx_in_acsf = (fe_only_rot_matrix_in_acsf @ initial_prism_axis_for_tx.to_4d()).to_3d()
        dir_ty_in_acsf = (fe_only_rot_matrix_in_acsf @ initial_prism_axis_for_ty.to_4d()).to_3d()
        dir_tz_in_acsf = (fe_only_rot_matrix_in_acsf @ initial_prism_axis_for_tz.to_4d()).to_3d()

        # Calculate the total translation offset in the ACSf frame
        translation_offset_in_acsf = (
            (dir_tx_in_acsf * tx) +
            (dir_ty_in_acsf * ty) +
            (dir_tz_in_acsf * tz)
        )
        # Add this offset to the original translation from acsm_initial_local_matrix
        final_matrix.translation = acsm_initial_local_matrix.to_translation() + translation_offset_in_acsf
    else:
        final_matrix = base_pose_with_jcs
    return final_matrix
