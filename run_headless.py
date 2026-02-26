"""
run_headless.py  —  BlenderROMFinder headless runner
=====================================================
Runs the High-Performance collision/ROM search without opening the Blender GUI.
All parameters (meshes, rotation ranges, export path …) are read from the
scene's saved collision_props, exactly as configured in the UI.

REQUIREMENTS
------------
*  The .blend file must already be saved with your scene set up.
   (The addon reads object references by name from the open file.)

USAGE
-----
Basic (uses every setting from the .blend file as-is):

    blender --background  "C:/path/to/scene.blend"  \
            --python      "C:/path/to/run_headless.py"

Override individual options after ' -- ':

    blender --background scene.blend --python run_headless.py -- [options]

Options (all optional):
  --export-path  <path>    Override the CSV export path stored in the scene.
                           Supports Blender's // relative prefix.
                           Example: --export-path //results/output.csv
  --workers      <int>     Number of parallel headless Blender worker processes
                           to spawn (default: headless_worker_count from scene,
                           which defaults to the CPU core count).
  --worker-exec  <path>    Path to the Blender executable used for workers.
                           Defaults to the current Blender binary.
  --batch-size   <int>     Poses per batch in single-threaded fallback mode.
  --single-threaded        Force single-threaded mode even if workers could run.
  --no-export              Skip CSV export even if export_to_csv is enabled.
  --no-save                Do NOT re-save the .blend file when finished.
  --verbose                Print a progress line for every 1 % change and
                           forward all worker log lines.

EXAMPLES
--------
PowerShell:
    & "C:/Program Files/Blender Foundation/Blender 5.0/blender.exe" `
        --background "C:/Work/shoulder_study.blend" `
        --python     "C:/Work/BlenderROMFinder/run_headless.py"

    # Override export path and skip re-saving the file:
    & "C:/Program Files/Blender Foundation/Blender 5.0/blender.exe" `
        --background "C:/Work/shoulder_study.blend" `
        --python     "C:/Work/BlenderROMFinder/run_headless.py" `
        -- --export-path "C:/Work/results/run1.csv" --no-save

CMD:
    "C:\Program Files\Blender Foundation\Blender 5.0\blender.exe" ^
        --background "C:\Work\shoulder_study.blend" ^
        --python     "C:\Work\BlenderROMFinder\run_headless.py"
"""

import sys
import os
import time

import bpy

# ---------------------------------------------------------------------------
# Make sure the addon package is importable (handles the case where Blender's
# sys.path doesn't include the addons folder yet)
# ---------------------------------------------------------------------------
_addon_dir  = os.path.dirname(os.path.abspath(__file__))   # …/BlenderROMFinder
_addon_parent = os.path.dirname(_addon_dir)                 # …/addons
if _addon_parent not in sys.path:
    sys.path.insert(0, _addon_parent)


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
# Our known flags — used to scan sys.argv regardless of where '--' appears.
_KNOWN_FLAGS = {'--no-export', '--no-save', '--verbose', '--single-threaded'}
_KNOWN_OPTS  = {'--export-path', '--batch-size', '--workers', '--worker-exec'}

def _build_args():
    """Return a list of our script's arguments.

    Blender convention:  everything after the bare '--' token is meant for
    the Python script.  If the caller forgot '--', Blender will complain
    about unknown arguments but our flags will still end up in sys.argv,
    so we fall back to scanning the full argv for any of our known tokens.
    """
    argv = sys.argv
    try:
        after_sep = argv[argv.index('--') + 1:]
        return after_sep
    except ValueError:
        # '--' separator not present — scan full argv for our tokens and warn
        our_tokens = _KNOWN_FLAGS | _KNOWN_OPTS
        found = [a for a in argv if a in our_tokens]
        if found:
            print(
                "[ROMFinder] WARNING: found script options in argv but no '--' separator. "
                "Blender will print 'unknown argument' errors for them.\n"
                "  Correct form:  blender --background scene.blend --python run_headless.py "
                "-- --export-path ...\n"
                "  (note the bare '--' before script options)",
                flush=True
            )
        # Return everything after '--python <script>' so we don't misparse
        # Blender's own arguments.
        try:
            py_idx = argv.index('--python')
            # skip '--python' and the script filename itself
            return argv[py_idx + 2:]
        except (ValueError, IndexError):
            return []

_args = _build_args()

def _flag(name):
    """Return True if a bare flag is present (e.g. --no-save)."""
    return name in _args

def _opt(name, default=None):
    """Return the value following a named option, or *default*."""
    try:
        idx = _args.index(name)
        val = _args[idx + 1]
        # Guard against accidentally treating the next flag as a value
        if val.startswith('--'):
            print(f"[ROMFinder] WARNING: option {name} has no value (got '{val}'); ignoring.",
                  flush=True)
            return default
        return val
    except (ValueError, IndexError):
        return default


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    # -- Import the processor from the addon package -------------------------
    try:
        from BlenderROMFinder.parallel_processor_v2 import OptimizedROMProcessor
    except ImportError as exc:
        print(f"[ROMFinder] ERROR: Could not import OptimizedROMProcessor: {exc}", flush=True)
        sys.exit(1)

    # -- Scene props ---------------------------------------------------------
    scene = bpy.context.scene
    if not hasattr(scene, 'collision_props'):
        print("[ROMFinder] ERROR: Scene has no 'collision_props'. "
              "Is the BlenderROMFinder addon enabled in this Blender build?", flush=True)
        sys.exit(1)

    props = scene.collision_props

    # Validate essential object references
    if not props.proximal_object or not props.distal_object:
        print("[ROMFinder] ERROR: proximal_object or distal_object not set in the scene. "
              "Open the file in Blender and configure them in the panel first.", flush=True)
        sys.exit(1)

    # -- Apply CLI overrides -------------------------------------------------
    if _opt('--export-path'):
        props.export_path = _opt('--export-path')
        print(f"[ROMFinder] Export path overridden → {props.export_path}", flush=True)

    if _opt('--batch-size'):
        try:
            props.batch_size = int(_opt('--batch-size'))
            print(f"[ROMFinder] Batch size overridden → {props.batch_size}", flush=True)
        except ValueError:
            print(f"[ROMFinder] WARNING: --batch-size value is not an integer; ignoring.", flush=True)

    skip_export = _flag('--no-export')
    skip_save   = _flag('--no-save')
    verbose     = _flag('--verbose')
    progress_interval = 1 if verbose else 5   # print every N percent

    # -- Resolve worker count / exec ----------------------------------------
    worker_count_override = _opt('--workers')
    worker_exec_override  = _opt('--worker-exec')
    single_threaded       = _flag('--single-threaded')

    if worker_count_override:
        try:
            worker_count = max(1, int(worker_count_override))
        except ValueError:
            print("[ROMFinder] WARNING: --workers value is not an integer; using scene default.", flush=True)
            worker_count = None
    else:
        worker_count = None  # resolved after processor init

    blender_exec = worker_exec_override or (props.headless_worker_exec or None) or None

    # -- Print a summary of the run configuration ----------------------------
    print("=" * 60, flush=True)
    print("[ROMFinder] Headless High-Performance Run", flush=True)
    print(f"  Blend file  : {bpy.data.filepath}", flush=True)
    print(f"  Proximal    : {props.proximal_object.name}", flush=True)
    print(f"  Distal      : {props.distal_object.name}", flush=True)
    print(f"  Rot mode    : {props.rotation_mode_enum}", flush=True)
    print(f"  Rot X       : {props.rot_x_min}° → {props.rot_x_max}°  step {props.rot_x_inc}°", flush=True)
    print(f"  Rot Y       : {props.rot_y_min}° → {props.rot_y_max}°  step {props.rot_y_inc}°", flush=True)
    print(f"  Rot Z       : {props.rot_z_min}° → {props.rot_z_max}°  step {props.rot_z_inc}°", flush=True)
    print(f"  Trans X     : {props.trans_x_min} → {props.trans_x_max}  step {props.trans_x_inc}", flush=True)
    print(f"  Trans Y     : {props.trans_y_min} → {props.trans_y_max}  step {props.trans_y_inc}", flush=True)
    print(f"  Trans Z     : {props.trans_z_min} → {props.trans_z_max}  step {props.trans_z_inc}", flush=True)
    print(f"  Export CSV  : {not skip_export}  →  {bpy.path.abspath(props.export_path)}", flush=True)
    print(f"  Valid-only  : {props.only_export_valid_poses}", flush=True)
    print(f"  Save .blend : {not skip_save}", flush=True)
    print("=" * 60, flush=True)

    # -- Initialise processor ------------------------------------------------
    import base64
    import queue as _queue
    from pathlib import Path

    # Import worker-spawning helpers from the addon
    try:
        import BlenderROMFinder.parallel_processor_v2 as _pp2
        start_headless_workers_async = _pp2.start_headless_workers_async
        # worker_headless.py lives next to parallel_processor_v2.py in the addon folder
        worker_script = Path(_pp2.__file__).with_name('worker_headless.py')
    except Exception as exc:
        print(f"[ROMFinder] WARNING: could not import worker helpers ({exc}); will run single-threaded.",
              flush=True)
        start_headless_workers_async = None
        worker_script = None

    proc = OptimizedROMProcessor()
    try:
        proc.initialize(props)
    except Exception as exc:
        print(f"[ROMFinder] ERROR during initialization: {exc}", flush=True)
        import traceback; traceback.print_exc()
        sys.exit(1)

    proc.start_processing()

    total      = proc.total_poses
    batch_size = max(1, getattr(props, 'batch_size', 500))

    if worker_count is None:
        worker_count = max(1, min(getattr(props, 'headless_worker_count',
                                          __import__('os').cpu_count() or 1),
                                  total))

    blend_path = bpy.data.filepath
    can_use_workers = (
        not single_threaded
        and start_headless_workers_async is not None
        and worker_script is not None
        and worker_script.exists()
        and blend_path
        and total > 0
    )

    print(f"[ROMFinder] {total:,} poses to evaluate", flush=True)
    if can_use_workers:
        print(f"[ROMFinder] Spawning {worker_count} headless worker(s) ("
              f"worker_headless.py at {worker_script})", flush=True)
    else:
        reasons = []
        if single_threaded:           reasons.append('--single-threaded flag')
        if not blend_path:            reasons.append('file not saved')
        if not can_use_workers and not single_threaded:
            if worker_script and not worker_script.exists():
                reasons.append(f'worker script not found at {worker_script}')
        if reasons:
            print(f"[ROMFinder] Running single-threaded ({'; '.join(reasons)})", flush=True)
        else:
            print("[ROMFinder] Running single-threaded", flush=True)

    if total == 0:
        print("[ROMFinder] WARNING: 0 poses computed — check rotation/translation ranges.", flush=True)

    t_start         = time.time()
    last_pct         = -(progress_interval + 1)   # ensure first tick always prints
    last_print_time  = t_start                     # for time-based heartbeat

    # -------------------------------------------------------------------------
    # PARALLEL path — spawn N worker Blenders, poll their queues, merge results
    # -------------------------------------------------------------------------
    if can_use_workers:
        # Serialise props the same way the operator does
        props_dict = {
            'proximal_object': props.proximal_object.name if props.proximal_object else None,
            'distal_object':   props.distal_object.name   if props.distal_object   else None,
            'ACSf_object':     props.ACSf_object.name     if props.ACSf_object     else None,
            'ACSm_object':     props.ACSm_object.name     if props.ACSm_object     else None,
            'ACSm_bone':       props.ACSm_bone,
            'rotation_mode_enum': props.rotation_mode_enum,
            'rot_x_min': props.rot_x_min, 'rot_x_max': props.rot_x_max, 'rot_x_inc': props.rot_x_inc,
            'rot_y_min': props.rot_y_min, 'rot_y_max': props.rot_y_max, 'rot_y_inc': props.rot_y_inc,
            'rot_z_min': props.rot_z_min, 'rot_z_max': props.rot_z_max, 'rot_z_inc': props.rot_z_inc,
            'trans_x_min': props.trans_x_min, 'trans_x_max': props.trans_x_max, 'trans_x_inc': props.trans_x_inc,
            'trans_y_min': props.trans_y_min, 'trans_y_max': props.trans_y_max, 'trans_y_inc': props.trans_y_inc,
            'trans_z_min': props.trans_z_min, 'trans_z_max': props.trans_z_max, 'trans_z_inc': props.trans_z_inc,
            'use_convex_hull':        bool(props.use_convex_hull_optimization),
            'use_aabb_precheck':      bool(props.use_aabb_precheck),
            'aabb_margin':            float(props.aabb_margin),
            'use_proxy_collision':    bool(props.use_proxy_collision),
            'proxy_decimate_ratio':   float(props.proxy_decimate_ratio),
            'only_export_valid_poses': bool(props.only_export_valid_poses),
        }
        import json as _json
        props_b64 = base64.b64encode(_json.dumps(props_dict).encode('utf-8')).decode('ascii')

        try:
            workers = start_headless_workers_async(
                blend_path,
                worker_script,
                total,
                worker_count=worker_count,
                chunk_size=getattr(props, 'headless_chunk_size', 500),
                blender_exec=blender_exec,
                timeout=getattr(props, 'headless_worker_timeout', 1800),
                props_b64=props_b64,
            )
        except Exception as exc:
            print(f"[ROMFinder] Failed to launch workers ({exc}); falling back to single-threaded.",
                  flush=True)
            workers = []

        for w in workers:
            status = 'started' if not w.get('done') else 'failed immediately'
            pid = w['proc'].pid if w.get('proc') else '?'
            print(f"[ROMFinder]  Worker {w['worker_id']}  PID {pid}  "
                  f"poses {w['start']}-{w['end']}  ({status})", flush=True)

        if workers:
            # Synchronous polling loop (no Blender modal needed)
            timeout_s = getattr(props, 'headless_worker_timeout', 1800)
            while True:
                any_active = False
                total_processed = 0

                for w in workers:
                    # Drain this worker's message queue
                    try:
                        while True:
                            msg_type, payload, wid = w['queue'].get_nowait()
                            w['last_output_time'] = time.time()
                            if msg_type == 'progress':
                                w['last_progress'] = int(payload.get('processed', 0))
                                w['total'] = int(payload.get('total', w.get('total', 0)))
                            elif msg_type == 'result':
                                proc.merge_worker_payload(payload)
                                w['done'] = True
                            elif msg_type == 'error':
                                print(f"[ROMFinder] Worker {wid} error: {payload}", flush=True)
                                w['done'] = True
                            elif msg_type == 'log':
                                if verbose:
                                    print(f"[ROMFinder Worker {wid}] {payload}", flush=True)
                    except Exception:
                        pass  # queue.Empty or other; keep going

                    # Check process exit
                    rc = w['proc'].poll()
                    if rc is not None and rc != 0 and not w.get('done'):
                        print(f"[ROMFinder] Worker {w['worker_id']} exited with code {rc}",
                              flush=True)
                        w['done'] = True

                    # Watchdog
                    if (not w.get('done')
                            and rc is None
                            and time.time() - w.get('last_output_time', time.time()) > timeout_s):
                        print(f"[ROMFinder] Worker {w['worker_id']} timed out; terminating.",
                              flush=True)
                        try:
                            w['proc'].terminate()
                        except Exception:
                            pass
                        w['done'] = True

                    if not w.get('done') and w['proc'].poll() is None:
                        any_active = True

                    total_processed += w.get('last_progress', 0)

                # Progress report — on pct milestone, time heartbeat, or completion
                pct     = int(total_processed / total * 100) if total > 0 else 100
                now     = time.time()
                heartbeat = (now - last_print_time) >= 10
                if pct >= last_pct + progress_interval or heartbeat or not any_active:
                    elapsed = now - t_start
                    done    = total_processed
                    if done > 0 and elapsed > 0:
                        eta = max(0.0, (total - done) / (done / elapsed))
                        m, s = divmod(int(eta), 60)
                        eta_str = f"ETA {m:02d}:{s:02d}"
                    else:
                        eta_str = "ETA --:--"  # workers still starting up
                    active_count = sum(1 for w in workers if not w.get('done'))
                    print(f"[ROMFinder]  {pct:3d}%  ({done:,}/{total:,} poses)  "
                          f"{active_count}/{len(workers)} worker(s) active  "
                          f"{elapsed:.0f}s elapsed  {eta_str}", flush=True)
                    last_pct        = pct
                    last_print_time = now

                if not any_active:
                    # Clean up temp props files
                    for w in workers:
                        try:
                            pf = w.get('props_file')
                            if pf and __import__('os').path.exists(pf):
                                __import__('os').unlink(pf)
                        except Exception:
                            pass
                    break

                time.sleep(0.25)  # 4 Hz poll — low overhead

            # If workers produced nothing (all failed), fall back below
            if proc.processed_poses == 0:
                print("[ROMFinder] All workers failed; falling back to single-threaded.",
                      flush=True)
                can_use_workers = False  # triggers the fallback block
        else:
            can_use_workers = False  # no workers started; fall through

    # -------------------------------------------------------------------------
    # SINGLE-THREADED fallback
    # -------------------------------------------------------------------------
    if not can_use_workers:
        while True:
            still_going = proc.process_batch(batch_size)

            pct = int(proc.get_progress())
            if pct >= last_pct + progress_interval or not still_going:
                elapsed = time.time() - t_start
                done    = proc.processed_poses
                if done > 0 and elapsed > 0:
                    rate = done / elapsed
                    eta  = max(0.0, (total - done) / rate)
                    m, s = divmod(int(eta), 60)
                    eta_str = f"ETA {m:02d}:{s:02d}"
                else:
                    eta_str = "ETA --:--"
                print(f"[ROMFinder]  {pct:3d}%  ({done:,}/{total:,} poses)  {eta_str}", flush=True)
                last_pct = pct

            if not still_going:
                break

    elapsed = time.time() - t_start
    m, s    = divmod(int(elapsed), 60)
    rate    = proc.processed_poses / elapsed if elapsed > 0 else 0
    print(f"[ROMFinder] Search complete: {len(proc.valid_poses):,} valid poses "
          f"out of {total:,}  in {m:02d}:{s:02d}  ({rate:.0f} poses/sec)", flush=True)

    # -- Export results ------------------------------------------------------
    if skip_export:
        print("[ROMFinder] Skipping CSV export (--no-export).", flush=True)
    else:
        try:
            exported = proc.export_results(props)
            if exported:
                print(f"[ROMFinder] CSV saved → {bpy.path.abspath(props.export_path)}", flush=True)
            else:
                print("[ROMFinder] WARNING: export_results returned False "
                      "(no data, export disabled in props, or path empty).", flush=True)
        except Exception as exc:
            print(f"[ROMFinder] ERROR during CSV export: {exc}", flush=True)
            import traceback; traceback.print_exc()

    # -- Create keyframes ---------------------------------------------------
    # The modal operator creates keyframes after export_results(), but in headless
    # mode there is no modal loop, so we drive the batches synchronously here.
    # Mirror the same condition the modal operator uses: props.visualize_collisions.
    if props.visualize_collisions and proc.valid_poses:
        print(f"[ROMFinder] Creating keyframes for {len(proc.valid_poses):,} valid poses...",
              flush=True)
        try:
            if proc.start_keyframe_creation(props):
                kf_start = time.time()
                kf_batches = 0
                while proc.process_keyframe_batch(props):
                    kf_batches += 1
                # process_keyframe_batch returns False on the *final* batch after calling
                # apply_collected_keyframes_5_0() and finish_keyframe_creation() itself.
                kf_elapsed = time.time() - kf_start
                print(f"[ROMFinder] Keyframes written in {kf_elapsed:.1f}s "
                      f"({kf_batches + 1} batch(es)).", flush=True)
            else:
                print("[ROMFinder] WARNING: Could not start keyframe creation.", flush=True)
        except Exception as exc:
            print(f"[ROMFinder] ERROR during keyframe creation: {exc}", flush=True)
            import traceback; traceback.print_exc()
    elif props.visualize_collisions:
        print("[ROMFinder] No valid poses found; skipping keyframe creation.", flush=True)
    else:
        print("[ROMFinder] visualize_collisions is off; skipping keyframe creation "
              "(enable it in the panel to get keyframed poses in the saved file).", flush=True)

    # -- Save .blend ---------------------------------------------------------
    if skip_save:
        print("[ROMFinder] Skipping .blend save (--no-save).", flush=True)
    else:
        try:
            bpy.ops.wm.save_mainfile()
            print(f"[ROMFinder] .blend saved → {bpy.data.filepath}", flush=True)
        except Exception as exc:
            print(f"[ROMFinder] ERROR saving .blend: {exc}", flush=True)

    print("[ROMFinder] Done.", flush=True)


if __name__ == '__main__':
    main()
