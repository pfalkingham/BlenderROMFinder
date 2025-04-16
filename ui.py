import bpy
from bpy.types import Panel

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
        
        # Check if calculation is in progress
        if props.is_calculating:
            # Show progress information
            box = layout.box()
            box.label(text="Calculating collisions...", icon='INFO')
            
            # Progress bar
            progress_row = box.row()
            progress_row.prop(props, "calculation_progress", text="Progress")
            
            # Cancel button
            cancel_row = box.row()
            cancel_row.operator("collision.cancel", icon='X')
            return  # Don't show the rest of the UI while calculating
        
        # Object selection
        box = layout.box()
        box.label(text="Objects:")
        box.prop(props, "proximal_object")
        box.prop(props, "distal_object")
        box.prop(props, "rotational_object")
        
        # Rotation parameters
        box = layout.box()
        box.label(text="Rotation (degrees):")
        
        # Add dropdown for rotation order
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
        
        # Attribute storage options
        box.prop(props, "store_as_attributes")
        
        if props.store_as_attributes:
            row = box.row()
            row.prop(props, "attribute_name_prefix")
            row = box.row()
            row.prop(props, "visualize_collisions")
            
            # Add info text about NLA editor if visualization is enabled
            if props.visualize_collisions:
                info_row = box.row()
                info_row.label(text="Check NLA Editor to see animations", icon='INFO')
        
        # Performance options - keep this as a SEPARATE box
        box = layout.box()
        box.label(text="Performance Options:", icon='PREFERENCES')
        
        # Batch size
        box.prop(props, "batch_size")
        
        # Calculate button - place this OUTSIDE any box
        row = layout.row(align=True)
        row.scale_y = 1.5
        row.operator("collision.calculate", icon='PLAY')