"""
Worker script for headless processing.
Emits progress messages and final result JSON to stdout using a small protocol:
- ROMF_PROGRESS <json>
- ROMF_RESULT_START\n<single-line JSON>\nROMF_RESULT_END
- ROMF_ERROR <json>

This script is executed with Blender in --background mode.
"""

import sys
import argparse
import json
import time
import os
import traceback

import bpy

# Ensure parent of addon is on sys.path so package imports work (we want to import as BlenderROMFinder.parallel_processor_v2)
addon_dir = os.path.dirname(__file__)
addon_parent = os.path.dirname(addon_dir)
if addon_parent not in sys.path:
    sys.path.insert(0, addon_parent)

import importlib
try:
    ppmod = importlib.import_module('BlenderROMFinder.parallel_processor_v2')
    OptimizedROMProcessor = ppmod.OptimizedROMProcessor
except Exception as e:
    # Fallback: try local import and give a helpful error
    try:
        from parallel_processor_v2 import OptimizedROMProcessor
    except Exception as e2:
        print("ROMF_ERROR " + json.dumps({"msg": "Failed to import parallel_processor_v2", "err1": str(e), "err2": str(e2)}), flush=True)
        raise


def parse_args_manual():
    """Simple, robust manual arg parsing that avoids argparse raising SystemExit

    Returns a dict with keys: start_index, end_index, emit_chunk, worker_id, props_json_b64, props_json_file
    """
    argv = sys.argv[1:]
    args = {
        'start_index': None,
        'end_index': None,
        'emit_chunk': 100,
        'worker_id': 0,
        'props_json_b64': None,
        'props_json_file': None
    }

    def get_val(flag):
        try:
            i = argv.index(flag)
            return argv[i+1]
        except ValueError:
            return None
        except IndexError:
            return None

    si = get_val('--start-index')
    ei = get_val('--end-index')
    if si is not None:
        try:
            args['start_index'] = int(si)
        except Exception:
            args['start_index'] = None
    if ei is not None:
        try:
            args['end_index'] = int(ei)
        except Exception:
            args['end_index'] = None

    ec = get_val('--emit-chunk')
    if ec is not None:
        try:
            args['emit_chunk'] = int(ec)
        except Exception:
            pass

    wid = get_val('--worker-id')
    if wid is not None:
        try:
            args['worker_id'] = int(wid)
        except Exception:
            pass

    pb = get_val('--props-json-b64')
    if pb is not None:
        args['props_json_b64'] = pb

    pf = get_val('--props-json-file')
    if pf is not None:
        args['props_json_file'] = pf

    return args



def main():
    # Debug: print argv the worker received
    try:
        print("ROMF_DEBUG ARGV " + json.dumps(sys.argv), flush=True)
    except Exception:
        pass

    args = parse_args_manual()

    # Validate required arguments
    if args['start_index'] is None or args['end_index'] is None:
        print("ROMF_ERROR " + json.dumps({"msg": "Missing required args --start-index/--end-index", "argv": sys.argv}), flush=True)
        sys.exit(2)

    try:
        # Print a startup banner so parent can see worker started
        print(f"ROMF_DEBUG Worker {args['worker_id']} starting: range {args['start_index']}-{args['end_index']}", flush=True)

        # If props passed via JSON (file or b64), decode and create a temporary props object
        props_dict = None
        if args['props_json_file']:
            try:
                with open(args['props_json_file'], 'r', encoding='utf-8') as f:
                    props_dict = json.load(f)
            except Exception as e:
                print("ROMF_ERROR " + json.dumps({"msg": "Failed to read props json file", "err": str(e)}), flush=True)
                raise
        elif args['props_json_b64']:
            import base64
            props_json = base64.b64decode(args['props_json_b64'].encode('ascii')).decode('utf-8')
            props_dict = json.loads(props_json)

        if props_dict:
            # Build a lightweight props object with attributes used by initialize
            from types import SimpleNamespace
            props = SimpleNamespace()

            # Resolve object references by name
            def obj(name):
                return bpy.data.objects.get(name) if name else None

            props.proximal_object = obj(props_dict.get('proximal_object'))
            props.distal_object = obj(props_dict.get('distal_object'))
            props.ACSf_object = obj(props_dict.get('ACSf_object'))
            props.ACSm_object = obj(props_dict.get('ACSm_object'))
            props.ACSm_bone = props_dict.get('ACSm_bone')

            # Copy numeric properties
            props.rotation_mode_enum = props_dict.get('rotation_mode_enum')
            props.rot_x_min = props_dict.get('rot_x_min')
            props.rot_x_max = props_dict.get('rot_x_max')
            props.rot_x_inc = props_dict.get('rot_x_inc')
            props.rot_y_min = props_dict.get('rot_y_min')
            props.rot_y_max = props_dict.get('rot_y_max')
            props.rot_y_inc = props_dict.get('rot_y_inc')
            props.rot_z_min = props_dict.get('rot_z_min')
            props.rot_z_max = props_dict.get('rot_z_max')
            props.rot_z_inc = props_dict.get('rot_z_inc')
            props.trans_x_min = props_dict.get('trans_x_min')
            props.trans_x_max = props_dict.get('trans_x_max')
            props.trans_x_inc = props_dict.get('trans_x_inc')
            props.trans_y_min = props_dict.get('trans_y_min')
            props.trans_y_max = props_dict.get('trans_y_max')
            props.trans_y_inc = props_dict.get('trans_y_inc')
            props.trans_z_min = props_dict.get('trans_z_min')
            props.trans_z_max = props_dict.get('trans_z_max')
            props.trans_z_inc = props_dict.get('trans_z_inc')
            props.use_convex_hull_optimization = props_dict.get('use_convex_hull')
            props.use_aabb_precheck = props_dict.get('use_aabb_precheck')
            props.aabb_margin = props_dict.get('aabb_margin')
            props.use_proxy_collision = props_dict.get('use_proxy_collision')
            props.proxy_decimate_ratio = props_dict.get('proxy_decimate_ratio')
            props.only_export_valid_poses = props_dict.get('only_export_valid_poses')

            # Ensure defaults for optional props referenced in processing
            # These mirror the defaults from the UI/PropertyGroup
            if not hasattr(props, 'debug_mode'):
                props.debug_mode = False
            if not hasattr(props, 'turn_off_collisions'):
                props.turn_off_collisions = False
            if not hasattr(props, 'only_export_valid_poses'):
                props.only_export_valid_poses = False
            if not hasattr(props, 'use_aabb_precheck'):
                props.use_aabb_precheck = True
            if not hasattr(props, 'aabb_margin'):
                props.aabb_margin = 0.0
            if not hasattr(props, 'use_proxy_collision'):
                props.use_proxy_collision = False
            if not hasattr(props, 'proxy_decimate_ratio'):
                props.proxy_decimate_ratio = 1.0
            if not hasattr(props, 'use_convex_hull_optimization'):
                props.use_convex_hull_optimization = False
            if not hasattr(props, 'batch_size'):
                props.batch_size = 10
            if not hasattr(props, 'visualize_collisions'):
                props.visualize_collisions = False
        else:
            # Fall back to scene props if not provided
            props = bpy.context.scene.collision_props

        proc = OptimizedROMProcessor()
        proc.initialize(props)

        total = proc.total_poses
        start = args['start_index']
        end = args['end_index']
        emit_chunk = max(1, args['emit_chunk'])

        processed = 0
        csv_rows = []
        valids = []

        i = start
        while i <= end:
            sub_end = min(end, i + emit_chunk - 1)
            # Use processor to compute a small sub-range
            part = proc.process_pose_range(i, sub_end)
            csv_rows.extend(part.get('csv_rows', []))
            valids.extend(part.get('valids', []))

            processed += (sub_end - i + 1)
            # Emit progress
            try:
                print("ROMF_PROGRESS " + json.dumps({"processed": processed, "total": (end - start + 1), "worker_id": args['worker_id']}), flush=True)
            except Exception:
                pass

            i = sub_end + 1

        # Emit result payload
        payload = {"csv_rows": csv_rows, "valids": valids, "worker_id": args['worker_id']}
        print("ROMF_RESULT_START", flush=True)
        print(json.dumps(payload), flush=True)
        print("ROMF_RESULT_END", flush=True)

    except Exception as exc:
        err = {"msg": str(exc), "trace": traceback.format_exc()}
        print("ROMF_ERROR " + json.dumps(err), flush=True)
        sys.exit(1)


if __name__ == '__main__':
    main()
