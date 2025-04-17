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

    def update_rotational_bone(self, context):
        # Called when rotational_object changes, to update the bone list
        if self.rotational_object and self.rotational_object.type == 'ARMATURE':
            armature = self.rotational_object.data
            return [(bone.name, bone.name, "") for bone in armature.bones]
        return []

    rotational_bone: EnumProperty(
        name="Rotational Bone",
        description="Select the bone to use as the rotation center (if armature)",
        items=lambda self, context: self.update_rotational_bone(context),
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
    
    rot_order: EnumProperty(
        name="Rotation Order",
        description="Order in which rotations are applied",
        items=[
            ("XYZ", "XYZ", ""),
            ("XZY", "XZY", ""),
            ("YXZ", "YXZ", ""),
            ("YZX", "YZX", ""),
            ("ZXY", "ZXY", ""),
            ("ZYX", "ZYX", "")
        ],
        default="ZYX"
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
    
    batch_size: IntProperty(
        name="Batch Size",
        description="Number of iterations to process per update (higher values = faster but less responsive UI)",
        default=100,
        min=10,
        max=5000
    )
    
    # Progress tracking
    is_calculating: BoolProperty(
        name="Is Calculating",
        description="Whether a calculation is currently in progress",
        default=False
    )
    
    calculation_progress: FloatProperty(
        name="Calculation Progress",
        description="Progress of the current calculation (0-100%)",
        default=0.0,
        min=0.0,
        max=100.0,
        subtype='PERCENTAGE'
    )
    
    time_remaining: StringProperty(
        name="Time Remaining",
        description="Estimated time remaining for the calculation",
        default=""
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
        name="Show Animation Layer",
        description="Create an NLA animation layer with non-colliding poses and make it visible",
        default=True
    )
    
    attribute_name_prefix: StringProperty(
        name="Attribute Prefix",
        description="Prefix for the attribute names",
        default="collision_"
    )