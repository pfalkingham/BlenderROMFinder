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
        props = getattr(context.scene, 'collision_props', None)
        # Debug: print all property names on props
        if props:
            print('DEBUG: collision_props properties:', dir(props))

        # Show version at the top
        version = bl_info.get("version", ("?",))
        version_str = ".".join(str(v) for v in version)
        layout.label(text=f"Range of Motion Finder v{version_str}", icon='PLUGIN')
        
        # Check if calculation is in progress
        if props and getattr(props, 'is_calculating', False):
            # Show progress information
            box = layout.box()
            box.label(text="Calculating collisions...", icon='INFO')
            
            # Progress bar (only if property exists)
            progress = getattr(props, 'calculation_progress', None)
            if progress is not None:
                progress_row = box.row()
                progress_row.prop(props, "calculation_progress", text="Progress")
            
            # Time remaining
            time_rem = getattr(props, 'time_remaining', None)
            if time_rem:
                box.label(text=str(time_rem))
            
            # Cancel button
            cancel_row = box.row()
            cancel_row.operator("collision.cancel", icon='X')
            return  # Don't show the rest of the UI while calculating
        
        # Object selection
        box = layout.box()
        box.label(text="Objects:")
        if props:
            box.prop(props, "proximal_object")
            box.prop(props, "distal_object")
            box.prop(props, "rotational_object")
            # Show bone dropdown if rotational_object is an armature
            if getattr(props, 'rotational_object', None) and props.rotational_object.type == 'ARMATURE':
                box.prop(props, "rotational_bone")
        
            # Axis and bone selection
            box.prop(props, "ACSf", text="Fixed (proximal) axis")
            if getattr(props, 'ACSf', None) and props.ACSf.type == 'ARMATURE':
                box.prop(props, "rotational_bone", text="Fixed (proximal) bone")
            box.prop(props, "ACSm", text="Mobile (distal) axis")
            if getattr(props, 'ACSm', None) and props.ACSm.type == 'ARMATURE':
                box.prop(props, "rotational_bone_2", text="Mobile (distal) bone")
        
        # Rotation parameters
        box = layout.box()
        box.label(text="Rotation (degrees):")
        
        # Add dropdown for rotation order
        if props:
            box.prop(props, "rot_order", text="Order")
        
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
        
        if props:
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
        if props:
            box.prop(props, "export_to_csv")
            if getattr(props, 'export_to_csv', False):
                box.prop(props, "export_path")
        
            # Animation layer (visualization) option - always show
            box.prop(props, "visualize_collisions")
            if getattr(props, 'visualize_collisions', False):
                info_row = box.row()
                info_row.label(text="Check NLA Editor to see animations", icon='INFO')
        
        # Performance options - keep this as a SEPARATE box
        box = layout.box()
        box.label(text="Performance Options:", icon='PREFERENCES')
        
        # Batch size
        if props:
            box.prop(props, "batch_size")
        
            # Add checkbox for convex hull optimization
            box.prop(props, "use_convex_hull_optimization")
        
        # Calculate button - place this OUTSIDE any box
        row = layout.row(align=True)
        row.scale_y = 1.5
        row.operator("collision.calculate", icon='PLAY')