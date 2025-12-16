import bpy
from parallel_processor_v2 import COLLISION_OT_calculate_parallel


def test_save_prompt(monkeypatch):
    op = COLLISION_OT_calculate_parallel()

    # Ensure blend is marked unsaved
    monkeypatch.setattr(bpy.data, 'filepath', '')

    called = {'saved': False}

    def fake_save_as_mainfile(*args, **kwargs):
        called['saved'] = True
        return {'CANCELLED'}

    # Replace the save operator with our fake
    monkeypatch.setattr(bpy.ops.wm, 'save_as_mainfile', fake_save_as_mainfile)

    # Ensure props state is reset
    props = bpy.context.scene.collision_props
    props.is_calculating = False

    res = op.execute(bpy.context)

    assert called['saved'] is True
    assert res == {'CANCELLED'}
    assert props.is_calculating is False
