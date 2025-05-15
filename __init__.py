bl_info = {
    "name": "Range of Motion Finder",
    "author": "Peter Falkingham, Andréas Jannel, Ben Griffin (Liverpool John Moores University)",
    "version": (1, 9, 5),
    "blender": (4, 4, 0),
    "location": "View3D > Sidebar > ROM",
    "description": "Find poses where two objects do not collide based on rotations and translations",
    "warning": "",
    "doc_url": "",
    "category": "3D View",
}

import bpy
from bpy.props import PointerProperty

# Import components from modules
from .properties import CollisionProperties
from .operators import COLLISION_OT_calculate, COLLISION_OT_cancel
from .ui import COLLISION_PT_panel

# Registration
classes = (
    CollisionProperties,
    COLLISION_OT_calculate,
    COLLISION_OT_cancel,
    COLLISION_PT_panel,
)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    # Remove old property if it exists (prevents registration bugs)
    if hasattr(bpy.types.Scene, 'collision_props'):
        del bpy.types.Scene.collision_props
    bpy.types.Scene.collision_props = PointerProperty(type=CollisionProperties)

def unregister():
    # Remove property before unregistering classes
    if hasattr(bpy.types.Scene, 'collision_props'):
        del bpy.types.Scene.collision_props
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)

if __name__ == "__main__":
    register()