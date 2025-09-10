import bpy
from bpy.types import Panel
# Import bl_info from __init__.py
from . import bl_info

class COLLISION_PT_panel(Panel):
    """Creates a panel in the 3D View sidebar"""
    bl_label = "Range of Motion Finder"  # We'll override this in draw
    bl_idname = "COLLISION_PT_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Collision"
    
    def draw(self, context):
        layout = self.layout
        props = context.scene.collision_props

        # Show version at the top
        version = bl_info.get("version", ("?",))
        version_str = ".".join(str(v) for v in version)
        layout.label(text=f"Range of Motion Finder v{version_str}", icon='PLUGIN')
        
        # Show rotation logic dropdown at the top
        layout.prop(props, "rotation_mode_enum", text="Rotation Logic")
        
        # Check if calculation is in progress
        if props.is_calculating:
            # Show progress information
            box = layout.box()
            box.label(text="Calculating collisions...", icon='INFO')
            
            # Progress bar
            progress_row = box.row()
            progress_row.prop(props, "calculation_progress", text="Progress")
            
            # Time remaining
            if props.time_remaining:
                box.label(text=props.time_remaining)
            
            # Cancel button
            cancel_row = box.row()
            cancel_row.operator("collision.cancel", icon='X')
            return  # Don't show the rest of the UI while calculating
          # Object selection
        box = layout.box()
        box.label(text="Objects:")
        box.prop(props, "proximal_object")
        box.prop(props, "distal_object")
        box.prop(props, "ACSf_object")
        # Show bone dropdown if ACSf_object is an armature
        if props.ACSf_object and props.ACSf_object.type == 'ARMATURE':
            box.prop(props, "ACSf_bone")
        box.prop(props, "ACSm_object")
        # Show bone dropdown if ACSm_object is an armature
        if props.ACSm_object and props.ACSm_object.type == 'ARMATURE':
            box.prop(props, "ACSm_bone")
        
        # Rotation parameters
        box = layout.box()
        box.label(text="Rotation (degrees):")
        
     
        # Add column headers for rotation
        header_row = box.row(align=True)
        header_row.label(text="")  # Empty space for axis label
        header_row.label(text="Min")
        header_row.label(text="Max")
        header_row.label(text="Step")
        
        row = box.row(align=True)
        row.label(text="X:")
        row.prop(props, "rot_x_min", text="")
        row.prop(props, "rot_x_max", text="")
        row.prop(props, "rot_x_inc", text="")
        
        row = box.row(align=True)
        row.label(text="Y:")
        row.prop(props, "rot_y_min", text="")
        row.prop(props, "rot_y_max", text="")
        row.prop(props, "rot_y_inc", text="")
        
        row = box.row(align=True)
        row.label(text="Z:")
        row.prop(props, "rot_z_min", text="")
        row.prop(props, "rot_z_max", text="")
        row.prop(props, "rot_z_inc", text="")
        
        # Translation parameters
        box = layout.box()
        box.label(text="Translation:")

        # Add column headers for translation
        header_row = box.row(align=True)
        header_row.label(text="")  # Empty space for axis label
        header_row.label(text="Min")
        header_row.label(text="Max")
        header_row.label(text="Step")
        
        row = box.row(align=True)
        row.label(text="X:")
        row.prop(props, "trans_x_min", text="")
        row.prop(props, "trans_x_max", text="")
        row.prop(props, "trans_x_inc", text="")
        
        row = box.row(align=True)
        row.label(text="Y:")
        row.prop(props, "trans_y_min", text="")
        row.prop(props, "trans_y_max", text="")
        row.prop(props, "trans_y_inc", text="")
        
        row = box.row(align=True)
        row.label(text="Z:")
        row.prop(props, "trans_z_min", text="")
        row.prop(props, "trans_z_max", text="")
        row.prop(props, "trans_z_inc", text="")
        
        # Output settings
        box = layout.box()
        box.label(text="Output:")
        
        # CSV Export options
        box.prop(props, "export_to_csv")
        if props.export_to_csv:
            box.prop(props, "export_path")
        
        # Animation layer (visualization) option - always show
        box.prop(props, "visualize_collisions")
        if props.visualize_collisions:
            info_row = box.row()
            info_row.label(text="Check NLA Editor to see animations", icon='INFO')
        
        # Debug mode option
        box.prop(props, "debug_mode")
        if props.debug_mode:
            debug_row = box.row()
            debug_row.label(text="Debug: Shows ALL poses (colliding + valid)", icon='INFO')
            debug_row = box.row()
            debug_row.prop(props, "turn_off_collisions")
        
        # Performance options - keep this as a SEPARATE box
        box = layout.box()
        box.label(text="Performance Options:", icon='PREFERENCES')
        
        # Batch size
        box.prop(props, "batch_size")
        
        # Add checkbox for convex hull optimization
        box.prop(props, "use_convex_hull_optimization")
        
        # Calculate button - place this OUTSIDE any box
        row = layout.row(align=True)
        row.scale_y = 1.5
        row.operator("collision.confirm_calculate", icon='PLAY')
        
        # High-Performance Optimized calculate button
        row = layout.row(align=True)
        row.scale_y = 1.2
        row.operator("collision.calculate_parallel", icon='MOD_ARRAY', text="High-Performance Optimized (experimental)")
        
        # Add info about optimized processing
        info_box = layout.box()
        info_box.scale_y = 0.8
        info_box.label(text="💡 Optimized processing uses efficient batching", icon='INFO')
        info_box.label(text="    with the same collision logic as original")
        info_box.label(text="    (should match original performance)")