# Range of Motion (ROM) Finder for Blender

A Blender addon that systematically finds all valid (non-colliding) 6-DOF poses between two objects across user-defined ranges of JCS rotations and translations. Useful for biomechanics, virtual palaeontology, robotics, and animation.

**Version:** 3.7.4 · **Blender:** 4.4+ · **Authors:** Peter Falkingham, Andréas Jannel, Ben Griffin, Rebecca Lowes (LJMU)  
**Video tutorial:** [YouTube](https://youtu.be/sQL41YbC_TY) *(may not reflect latest features)*


---

## Installation

1. Place the `BlenderROMFinder` folder in your Blender addons directory.
2. `Edit > Preferences > Add-ons > Install…` → select the folder/zip.
3. Enable *Range of Motion Finder*.
4. The **Collision** tab appears in the 3D View sidebar (`N`).

---

## Scene Setup

| Role | Description |
|---|---|
| **Proximal Object** | Static/base object (e.g. femur) |
| **Distal Object** | Moving object tested for collisions (e.g. tibia) |
| **ACS Fixed (ACSf)** | Defines the Z-axis for Flexion/Extension |
| **ACS Mobile (ACSm)** | Defines the X-axis for Long-Axis Rotation; Distal Object should follow it |

If either ACS object is an armature, a bone dropdown appears. Start with ACSm's local axes aligned to ACSf's for predictable results; use Delta Transforms for visual offsets.

---

## Rotation Modes

Choose one from the **Rotation Logic** dropdown:

| Mode | Rotation | Translation | Best for |
|---|---|---|---|
| **ISB Standard** | Z-Y'-X'' with floating Y' axis (Grood & Suntay) | Along final ACSm axes | Academic/clinical studies |
| **Simplified** | Z-Y'-X'' using ACSm's Y axis — reduces visual twist | Along final ACSm axes | Teaching, visualisation |
| **M&G Hinge** | Same as Simplified | "Prism" axes rotate with FE only | Hinge joints (M&G 2021) |

> **Visual twist note:** ISB Standard can show apparent roll at zero Long-Axis Rotation — this is correct mathematically. Use Simplified to avoid it.

---

## Parameters

Set Min / Max / Step for each of the six DOFs:

- **Rotations (degrees):** X = Long-Axis Rotation, Y = Ad/Abduction, Z = Flexion/Extension  
- **Translations (Blender units):** X, Y, Z (meaning depends on mode)

A live **Total poses to search** count is shown beneath the translation fields.

---

## Output

- **Export to CSV** — enable and set a path (`//` = relative to .blend). Columns: `rot_x/y/z`, `trans_x/y/z`, `Valid_pose` (1 = no collision, 0 = collision). **Only export valid poses** restricts output to non-colliding rows.
- **Show Animation Layer** — creates keyframes for valid poses viewable in the NLA Editor.
- **Debug Mode** — visualises all poses including collisions; also exposes **Turn Off Collisions** to bypass collision checking entirely.

---

## Running a Calculation

### Standard
Click **Confirm and Calculate Collisions**. Progress and ETA update in the panel. Press **Cancel** or `ESC` to stop early; any collected rows are still exported.

### High-Performance (parallel)
Click **High-Performance**. Spawns one headless Blender worker process per CPU core by default, each computing a slice of the pose space in parallel. Results are streamed back and merged automatically.

**Requirement:** the `.blend` file must be **saved** — workers load the scene from disk.

**Performance settings** (in the *Performance Options* box):

| Setting | Default | Description |
|---|---|---|
| Workers | CPU count | Number of parallel Blender processes |
| Chunk Size | 10 | Poses per progress-report chunk |
| Worker Timeout | 1800 s | Kill unresponsive workers after this long |
| Blender Executable | *(current binary)* | Override the Blender used for workers |
| Workers-only mode | On | Abort rather than fall back to single-thread if all workers fail |

**Convex Hull Pre-Check** — fast first-pass rejection; speeds up detection significantly. Disable if one object can fit entirely inside the other (can cause false negatives).

---

## Headless / Command-Line Mode

Run the High-Performance search entirely from a terminal using `run_headless.py`:

```powershell
& "C:\Program Files\Blender Foundation\Blender 5.0\blender.exe" `
    --background "C:\path\to\scene.blend" `
    --python     "C:\path\to\BlenderROMFinder\run_headless.py"
```

All settings are read from the saved scene. The same parallel workers are spawned as with the in-GUI button.

### Overriding settings

Append `--` (required) followed by options:

```powershell
& "...\blender.exe" --background scene.blend --python run_headless.py `
    -- --export-path "C:\results\run1.csv" --workers 8 --no-save --verbose
```

| Option | Description |
|---|---|
| `--export-path <path>` | Override CSV export path (supports `//` prefix) |
| `--workers <n>` | Number of parallel worker processes |
| `--worker-exec <path>` | Blender executable to use for workers |
| `--batch-size <n>` | Batch size for single-threaded fallback |
| `--single-threaded` | Disable workers; run on one core |
| `--no-export` | Skip CSV export |
| `--no-save` | Don't re-save the .blend on completion |
| `--verbose` | Progress every 1 % + forward all worker log lines |

### Troubleshooting headless runs

- **Workers fail immediately** — copy the `Starting worker N: <command>` line from the output and run it directly in a terminal to see the full error.
- **No progress output** — ensure the `.blend` is saved and the addon is enabled in that Blender build.
- **0 valid poses** — verify object/ACS assignments in the panel first; test with `--single-threaded --verbose`.
- **Workers silently hang** — reduce Worker Timeout in the panel or pass `--workers 1` to isolate the issue.

---

## Technical Details

### Collision detection
BVH tree intersection testing, with an optional convex hull pre-check for fast rejection. Each worker runs a full Blender instance, applies poses via `view_layer.update()`, and streams results over stdout using a lightweight protocol (`ROMF_PROGRESS`, `ROMF_RESULT_START/END`, `ROMF_ERROR`).

### Module overview

| File | Role |
|---|---|
| `__init__.py` | Registration and metadata |
| `properties.py` | Property definitions |
| `operators.py` | Standard modal operator |
| `parallel_processor_v2.py` | High-performance processor + worker-spawning helpers |
| `worker_headless.py` | Script run inside each worker Blender process |
| `run_headless.py` | Command-line entry point |
| `poseCalculations.py` | JCS pose matrix calculations |
| `ui.py` | Panel layout |

---

## Scientific References

- Grood & Suntay (1983). *J Biomech Eng* 105(2):136–144.
- Wu et al. (2002). *J Biomechanics* 35(4):543–548.
- Manafzadeh & Gatesy (2021). *J Anatomy* 239(6):1516–1524.

---

*Results should be validated against known standards. Save your work frequently during intensive calculations.*
