bl_info = {
    "name": "Range of Motion Finder",
    "author": "Peter Falkingham, Andréas Jannel, Ben Griffin (Liverpool John Moores University), and gratuitous AI/LLMs, especially Google AI Studio",
    "version": (5, 0, 1),
    "blender": (5, 0, 0),
    "location": "View3D > Sidebar > ROM",
    "description": "Find poses where two objects do not collide based on rotations and translations",
    "warning": "",
    "doc_url": "",
    "category": "3D View",
}

import bpy
from bpy.props import PointerProperty

from .properties import CollisionProperties
from .operators import (
    COLLISION_OT_confirm_calculation,
    COLLISION_OT_calculate,
    COLLISION_OT_cancel,
    COLLISION_OT_find_min_x_distance,
)
from .ui import COLLISION_PT_panel

classes = (
    CollisionProperties,
    COLLISION_OT_confirm_calculation,
    COLLISION_OT_calculate,
    COLLISION_OT_cancel,
    COLLISION_OT_find_min_x_distance,
    COLLISION_PT_panel,
)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.collision_props = PointerProperty(type=CollisionProperties)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.collision_props

if __name__ == "__main__":
    register()
