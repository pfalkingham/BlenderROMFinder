import bpy
from bpy.types import PropertyGroup
from bpy.props import (
    FloatProperty, 
    BoolProperty, 
    IntProperty, 
    StringProperty, 
    PointerProperty, 
    FloatVectorProperty,
    EnumProperty
)

class CollisionProperties(PropertyGroup):
    # Object selection
    proximal_object: PointerProperty(
        name="Proximal Object",
        description="Select the proximal (fixed) object",
        type=bpy.types.Object
    )
    
    distal_object: PointerProperty(
        name="Distal Object",
        description="Select the distal (moving) object",
        type=bpy.types.Object
    )
    
    rotational_object: PointerProperty(
        name="Rotational Object",
        description="Select the object to use as the rotation center",
        type=bpy.types.Object
    )
    
    # Rotation parameters
    rot_x_min: FloatProperty(
        name="X Min",
        description="Minimum X rotation in degrees",
        default=-30.0,
        min=-180.0,
        max=180.0
    )
    
    rot_x_max: FloatProperty(
        name="X Max",
        description="Maximum X rotation in degrees",
        default=30.0,
        min=-180.0,
        max=180.0
    )
    
    rot_x_inc: FloatProperty(
        name="X Inc",
        description="X rotation increment in degrees",
        default=5.0,
        min=0.1,
        max=90.0
    )
    
    rot_y_min: FloatProperty(
        name="Y Min",
        description="Minimum Y rotation in degrees",
        default=-30.0,
        min=-180.0,
        max=180.0
    )
    
    rot_y_max: FloatProperty(
        name="Y Max",
        description="Maximum Y rotation in degrees",
        default=30.0,
        min=-180.0,
        max=180.0
    )
    
    rot_y_inc: FloatProperty(
        name="Y Inc",
        description="Y rotation increment in degrees",
        default=5.0,
        min=0.1,
        max=90.0
    )
    
    rot_z_min: FloatProperty(
        name="Z Min",
        description="Minimum Z rotation in degrees",
        default=-30.0,
        min=-180.0,
        max=180.0
    )
    
    rot_z_max: FloatProperty(
        name="Z Max",
        description="Maximum Z rotation in degrees",
        default=30.0,
        min=-180.0,
        max=180.0
    )
    
    rot_z_inc: FloatProperty(
        name="Z Inc",
        description="Z rotation increment in degrees",
        default=5.0,
        min=0.1,
        max=90.0
    )
    
    # Translation parameters
    trans_x_min: FloatProperty(
        name="X Min",
        description="Minimum X translation",
        default=0.0,
        min=-100.0,
        max=100.0
    )
    
    trans_x_max: FloatProperty(
        name="X Max",
        description="Maximum X translation",
        default=0.0,
        min=-100.0,
        max=100.0
    )
    
    trans_x_inc: FloatProperty(
        name="X Inc",
        description="X translation increment",
        default=0.1,
        min=0.01,
        max=10.0
    )
    
    trans_y_min: FloatProperty(
        name="Y Min",
        description="Minimum Y translation",
        default=0.0,
        min=-100.0,
        max=100.0
    )
    
    trans_y_max: FloatProperty(
        name="Y Max",
        description="Maximum Y translation",
        default=0.0,
        min=-100.0,
        max=100.0
    )
    
    trans_y_inc: FloatProperty(
        name="Y Inc",
        description="Y translation increment",
        default=0.1,
        min=0.01,
        max=10.0
    )
    
    trans_z_min: FloatProperty(
        name="Z Min",
        description="Minimum Z translation",
        default=0.0,
        min=-100.0,
        max=100.0
    )
    
    trans_z_max: FloatProperty(
        name="Z Max",
        description="Maximum Z translation",
        default=0.0,
        min=-100.0,
        max=100.0
    )
    
    trans_z_inc: FloatProperty(
        name="Z Inc",
        description="Z translation increment",
        default=0.1,
        min=0.01,
        max=10.0
    )
    
    export_to_csv: BoolProperty(
        name="Export to CSV",
        description="Enable to export collision data to a CSV file",
        default=True
    )
    
    export_path: StringProperty(
        name="Export Path",
        description="Path to export collision data CSV",
        default="//collision_data.csv",
        subtype='FILE_PATH'
    )
    
    store_as_attributes: BoolProperty(
        name="Store as Attributes",
        description="Store collision data as attributes on the distal object",
        default=True
    )
    
    visualize_collisions: BoolProperty(
        name="Visualize Collisions [keyframe non-colliding poses]",
        description="Create a vertex group with collision points",
        default=True
    )
    
    attribute_name_prefix: StringProperty(
        name="Attribute Prefix",
        description="Prefix for the attribute names",
        default="collision_"
    )