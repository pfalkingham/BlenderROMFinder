import bpy
from bpy.types import Panel

class COLLISION_PT_panel(Panel):
    """Creates a panel in the 3D View sidebar"""
    bl_label = "Collision Range Finder"
    bl_idname = "COLLISION_PT_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Collision"
    
    def draw(self, context):
        layout = self.layout
        props = context.scene.collision_props
        
        # Object selection
        box = layout.box()
        box.label(text="Objects:")
        box.prop(props, "proximal_object")
        box.prop(props, "distal_object")
        box.prop(props, "rotational_object")
        
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
        
        # Attribute storage options
        box.prop(props, "store_as_attributes")
        
        if props.store_as_attributes:
            row = box.row()
            row.prop(props, "attribute_name_prefix")
            row = box.row()
            row.prop(props, "visualize_collisions")
        
        # Calculate button
        layout.operator("collision.calculate", icon='PLAY')