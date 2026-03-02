"""
ui.py — Sidebar panel for BlenderROMFinder.
"""

import bpy
from bpy.types import Panel
from . import bl_info
from .processor import get_total_pose_count


DECIMAL_PRECISION_THRESHOLD = 6  # flag step values needing more than this many decimal places


def _step_is_imprecise(val):
    """Return True if *val* needs more than DECIMAL_PRECISION_THRESHOLD
    decimal places to represent — e.g. back-calculation of 60/7 gives
    8.5714286, which has 7 dp and is flagged as potentially imprecise.

    Blender FloatProperty stores values as 32-bit floats, so a clean 0.01
    may be read back as 0.009999999776… in Python.  We normalise to 7
    decimal places first (float32 has ~7 significant digits) to suppress
    that storage noise before checking.
    """
    if val == 0.0:
        return False
    # Normalise away float32 storage noise, then count decimal places
    s = f"{round(val, 7):.15g}"
    if '.' not in s:
        return False
    after_dot = s.split('.')[1].rstrip('0')
    return len(after_dot) > DECIMAL_PRECISION_THRESHOLD


def _inc_is_non_integer(val):
    """Return True if *val* is not (close to) a whole number."""
    return abs(val - round(val)) > 1e-4


class COLLISION_PT_panel(Panel):
    """Creates a panel in the 3D View sidebar"""
    bl_label = "Range of Motion Finder"
    bl_idname = "COLLISION_PT_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Collision"

    def draw(self, context):
        layout = self.layout
        props = context.scene.collision_props

        # Version header
        version = bl_info.get("version", ("?",))
        version_str = ".".join(str(v) for v in version)
        layout.label(text=f"Range of Motion Finder v{version_str}", icon='PLUGIN')

        # Rotation logic dropdown
        layout.prop(props, "rotation_mode_enum", text="Rotation Logic")

        # --- Progress display (when calculating) ---
        if props.is_calculating:
            box = layout.box()
            box.label(text="Calculating collisions...", icon='INFO')
            progress_row = box.row()
            progress_row.prop(props, "calculation_progress", text="Progress")
            if props.time_remaining:
                box.label(text=props.time_remaining)
            cancel_row = box.row()
            cancel_row.operator("collision.cancel", icon='X')
            return

        # --- Object selection ---
        box = layout.box()
        box.label(text="Objects:")
        box.prop(props, "proximal_object")
        box.prop(props, "distal_object")
        box.prop(props, "ACSf_object")
        if props.ACSf_object and props.ACSf_object.type == 'ARMATURE':
            box.prop(props, "ACSf_bone")
        box.prop(props, "ACSm_object")
        if props.ACSm_object and props.ACSm_object.type == 'ARMATURE':
            box.prop(props, "ACSm_bone")

        # --- Rotation parameters ---
        box = layout.box()
        box.label(text="Rotation (degrees):")
        header_row = box.row(align=True)
        header_row.label(text="")
        header_row.label(text="Min")
        header_row.label(text="Max")
        header_row.label(text="Increment")
        header_row.label(text="Poses")

        row = box.row(align=True)
        row.label(text="X:")
        row.prop(props, "rot_x_min", text="")
        row.prop(props, "rot_x_max", text="")
        step_x = row.row(align=True)
        step_x.alert = _step_is_imprecise(props.rot_x_inc)
        step_x.prop(props, "rot_x_inc", text="")
        inc_x = row.row(align=True)
        inc_x.alert = _inc_is_non_integer(props.rot_x_num_inc)
        inc_x.prop(props, "rot_x_num_inc", text="")

        row = box.row(align=True)
        row.label(text="Y:")
        row.prop(props, "rot_y_min", text="")
        row.prop(props, "rot_y_max", text="")
        step_y = row.row(align=True)
        step_y.alert = _step_is_imprecise(props.rot_y_inc)
        step_y.prop(props, "rot_y_inc", text="")
        inc_y = row.row(align=True)
        inc_y.alert = _inc_is_non_integer(props.rot_y_num_inc)
        inc_y.prop(props, "rot_y_num_inc", text="")

        row = box.row(align=True)
        row.label(text="Z:")
        row.prop(props, "rot_z_min", text="")
        row.prop(props, "rot_z_max", text="")
        step_z = row.row(align=True)
        step_z.alert = _step_is_imprecise(props.rot_z_inc)
        step_z.prop(props, "rot_z_inc", text="")
        inc_z = row.row(align=True)
        inc_z.alert = _inc_is_non_integer(props.rot_z_num_inc)
        inc_z.prop(props, "rot_z_num_inc", text="")

        # --- Translation parameters ---
        box = layout.box()
        box.label(text="Translation:")
        header_row = box.row(align=True)
        header_row.label(text="")
        header_row.label(text="Min")
        header_row.label(text="Max")
        header_row.label(text="Increment")
        header_row.label(text="Poses")

        row = box.row(align=True)
        row.label(text="X:")
        row.prop(props, "trans_x_min", text="")
        row.prop(props, "trans_x_max", text="")
        step_x = row.row(align=True)
        step_x.alert = _step_is_imprecise(props.trans_x_inc)
        step_x.prop(props, "trans_x_inc", text="")
        inc_x = row.row(align=True)
        inc_x.alert = _inc_is_non_integer(props.trans_x_num_inc)
        inc_x.prop(props, "trans_x_num_inc", text="")

        row = box.row(align=True)
        row.label(text="Y:")
        row.prop(props, "trans_y_min", text="")
        row.prop(props, "trans_y_max", text="")
        step_y = row.row(align=True)
        step_y.alert = _step_is_imprecise(props.trans_y_inc)
        step_y.prop(props, "trans_y_inc", text="")
        inc_y = row.row(align=True)
        inc_y.alert = _inc_is_non_integer(props.trans_y_num_inc)
        inc_y.prop(props, "trans_y_num_inc", text="")

        row = box.row(align=True)
        row.label(text="Z:")
        row.prop(props, "trans_z_min", text="")
        row.prop(props, "trans_z_max", text="")
        step_z = row.row(align=True)
        step_z.alert = _step_is_imprecise(props.trans_z_inc)
        step_z.prop(props, "trans_z_inc", text="")
        inc_z = row.row(align=True)
        inc_z.alert = _inc_is_non_integer(props.trans_z_num_inc)
        inc_z.prop(props, "trans_z_num_inc", text="")

        # --- Total pose count ---
        box = layout.box()
        try:
            total = get_total_pose_count(props)
        except Exception:
            total = 0
        count_row = box.row()
        count_row.label(text=f"Total poses to search: {total:,}")

        # --- Output settings ---
        box = layout.box()
        box.label(text="Output:")
        row = box.row(align=True)
        row.prop(props, "export_to_csv")
        sub = row.row()
        sub.enabled = props.export_to_csv
        sub.prop(props, "only_export_valid_poses", text="Only export valid poses")
        if props.export_to_csv:
            box.prop(props, "export_path")

        box.prop(props, "visualize_collisions")
        if props.visualize_collisions:
            info_row = box.row()
            info_row.label(text="Check NLA Editor to see animations", icon='INFO')

        box.prop(props, "debug_mode")
        if props.debug_mode:
            debug_row = box.row()
            debug_row.label(text="Debug: Shows ALL poses (colliding + valid)", icon='INFO')
            debug_row = box.row()
            debug_row.prop(props, "turn_off_collisions")

        # --- Performance options ---
        box = layout.box()
        box.label(text="Performance Options:", icon='PREFERENCES')
        box.prop(props, "batch_size")
        box.prop(props, "use_convex_hull_optimization")
        box.prop(props, "penetration_sample_count")

        # Proxy mesh
        box.prop(props, "use_proxy_collision")
        if props.use_proxy_collision:
            box.prop(props, "proxy_decimate_ratio")

        # Headless workers
        box.separator()
        row = box.row(align=True)
        row.label(text="Headless Workers:")
        row.prop(props, "headless_worker_count", text="Workers")
        box.prop(props, "headless_chunk_size")
        box.prop(props, "headless_workers_only", text="Workers-only mode")
        if props.headless_workers_only:
            row = box.row()
            row.label(text="Workers-only mode: the run will abort on worker failure.")

        # --- Calculate button (single unified button) ---
        row = layout.row(align=True)
        row.scale_y = 1.5
        row.operator("collision.confirm_calculate", icon='PLAY')

        # --- Minimum X distance finder ---
        box = layout.box()
        box.label(text="Find Minimum X Distance:", icon='ARROW_LEFTRIGHT')
        row = box.row(align=True)
        row.prop(props, "min_x_distance_increment", text="Step")
        row = box.row(align=True)
        row.scale_y = 1.2
        row.operator("collision.find_min_x_distance", icon='VIEWZOOM')

        if props.min_x_distance_found:
            result_row = box.row()
            result_row.label(text=f"Found distance: {props.min_x_distance_result:.6f}", icon='CHECKMARK')
