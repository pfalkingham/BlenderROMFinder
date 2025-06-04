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
    
    # ACS objects (anatomical coordinate system)
    ACSf_object: PointerProperty(
        name="ACS Fixed",
        description="Select the fixed anatomical coordinate system (determines flexion/extension Z axis)",
        type=bpy.types.Object
    )
    
    ACSm_object: PointerProperty(
        name="ACS Mobile",
        description="Select the mobile anatomical coordinate system (determines long axis rotation X axis)",
        type=bpy.types.Object
    )

    def update_ACSf_bone(self, context):
        # Called when ACSf_object changes, to update the bone list
        if self.ACSf_object and self.ACSf_object.type == 'ARMATURE':
            armature = self.ACSf_object.data
            return [(bone.name, bone.name, "") for bone in armature.bones]
        return []

    ACSf_bone: EnumProperty(
        name="ACS Fixed Bone",
        description="Select the fixed bone to use for the ACS fixed coordinate system",
        items=lambda self, context: self.update_ACSf_bone(context),
    )
    
    def update_ACSm_bone(self, context):
        # Called when ACSm_object changes, to update the bone list
        if self.ACSm_object and self.ACSm_object.type == 'ARMATURE':
            armature = self.ACSm_object.data
            return [(bone.name, bone.name, "") for bone in armature.bones]
        return []

    ACSm_bone: EnumProperty(
        name="ACS Mobile Bone",
        description="Select the mobile bone to use for the ACS mobile coordinate system",
        items=lambda self, context: self.update_ACSm_bone(context),
    )
    
    # Rotation parameters
    rot_x_min: FloatProperty(
        name="X Min",
        description="Minimum X rotation in degrees",
        default=-30.0,
        min=-360.0,
        max=360.0
    )
    
    rot_x_max: FloatProperty(
        name="X Max",
        description="Maximum X rotation in degrees",
        default=30.0,
        min=-360.0,
        max=360.0
    )
    
    rot_x_inc: FloatProperty(
        name="X Inc",
        description="X rotation increment in degrees",
        default=5.0,
        min=0.01,
        max=90.0
    )
    
    rot_y_min: FloatProperty(
        name="Y Min",
        description="Minimum Y rotation in degrees",
        default=-30.0,
        min=-360.0,
        max=360.0
    )
    
    rot_y_max: FloatProperty(
        name="Y Max",
        description="Maximum Y rotation in degrees",
        default=30.0,
        min=-360.0,
        max=360.0
    )
    
    rot_y_inc: FloatProperty(
        name="Y Inc",
        description="Y rotation increment in degrees",
        default=5.0,
        min=0.01,
        max=90.0
    )
    
    rot_z_min: FloatProperty(
        name="Z Min",
        description="Minimum Z rotation in degrees",
        default=-30.0,
        min=-360.0,
        max=360.0
    )
    
    rot_z_max: FloatProperty(
        name="Z Max",
        description="Maximum Z rotation in degrees",
        default=30.0,
        min=-360.0,
        max=360.0
    )
    
    rot_z_inc: FloatProperty(
        name="Z Inc",
        description="Z rotation increment in degrees",
        default=5.0,
        min=0.01,
        max=90.0
    )
    

    rotation_mode_enum: EnumProperty(
        name="Rotation Logic",
        description="...", # Choose rotation AND translation logic
        items=[
            ('ISB_STANDARD', "ISB Standard", "ISB Rotations (floating y-axis) + ACSm Local Post-Rot Translations"),
            ('INTUITIVE', "Intuitive", "Intuitive Rotations (as ISB but using ACSm Y-axis)+ ACSm Local Post-Rot Translations"), # We need to define "Intuitive Rotations" clearly
            ('MG_HINGE', "M&G Hinge", "M&G Rotations (as intuitive) + M&G Prism Translations")
        ],
        default='MG_HINGE'
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
        min=0.001,
        max=10.0,
        precision=4
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
        min=0.001,
        max=10.0,
        precision=4
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
        min=0.001,
        max=10.0,
        precision=4
    )
    
    use_convex_hull_optimization: BoolProperty(
        name="Use Convex Hull Pre-Check",
        description="Enable to use a faster convex hull pre-check. WARNING: May give incorrect non-collision results if one object can be fully contained within the other. Disable for full accuracy in such cases.",
        default=False  # Off by default
    )
    
    batch_size: IntProperty(
        name="Batch Size",
        description="Number of iterations to process per update (higher values = faster but less responsive UI)",
        default=10,
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
    
    visualize_collisions: BoolProperty(
        name="Show Animation Layer",
        description="Create an NLA animation layer with non-colliding poses and make it visible",
        default=True
    )
    
    debug_mode: BoolProperty(
        name="Debug Mode",
        description="When enabled, visualizes ALL poses (both colliding and non-colliding) with different keyframes for debugging purposes",
        default=False
    )