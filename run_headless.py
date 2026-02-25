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
  --batch-size   <int>     Poses evaluated per processing cycle (default: 500).
                           Larger batches are faster but produce less frequent
                           progress prints.
  --no-export              Skip CSV export even if export_to_csv is enabled.
  --no-save                Do NOT re-save the .blend file when finished.
  --verbose                Print a progress line for every 1 % change instead
                           of every 5 %.

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
_KNOWN_FLAGS = {'--no-export', '--no-save', '--verbose'}
_KNOWN_OPTS  = {'--export-path', '--batch-size'}

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
    print(f"[ROMFinder] {total:,} poses to evaluate  (batch size {batch_size})", flush=True)

    if total == 0:
        print("[ROMFinder] WARNING: 0 poses computed — check rotation/translation ranges.", flush=True)

    # -- Processing loop -----------------------------------------------------
    t_start   = time.time()
    last_pct  = -1

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
