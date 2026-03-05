# Range of Motion (ROM) Finder for Blender

A Blender addon that systematically finds all valid (non-colliding) 6-DOF poses between two objects across user-defined ranges of JCS rotations and translations. Useful for biomechanics, virtual palaeontology, robotics, and animation.

**Version:** 5.0.4 · **Blender:** 5.0+ · **Authors:** Peter Falkingham, with help and testing from Andréas Jannel, Ben Griffin, Rebecca Lowes (LJMU)  
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

Set Min / Max / increment or steps for each of the six DOFs:

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

Click **Confirm and Calculate Collisions**. The addon validates the scene setup, then spawns headless Blender worker processes (one per CPU core by default) to compute the pose space in parallel. Results are streamed back and merged automatically. Progress and ETA update in the panel. Press **Cancel** or `ESC` to stop early; any collected rows are still exported.

If the `.blend` file is not yet saved (workers cannot load an unsaved scene) and *Workers-only mode* is disabled, the calculation falls back to in-process single-threaded execution.

**Performance settings** (in the *Performance Options* box):

| Setting | Default | Description |
|---|---|---|
| Batch Size | 10 | Poses processed per UI update tick in single-threaded fallback |
| Convex Hull Pre-Check | On | Tier-1 fast rejection; disable if one mesh can fit entirely inside the other |
| Penetration Samples | 10 000 | Distal surface vertices sampled for the Tier-2 penetration check |
| Use Proxy Mesh | Off | Use a decimated copy of the distal mesh for BVH rebuilds (faster per pose) |
| Proxy Ratio | 0.25 | Face ratio for the proxy mesh (1.0 = full detail) |
| Workers | CPU count | Number of parallel headless Blender processes |
| Chunk Size | 10 | Poses per worker progress-report chunk |
| Workers-only mode | On | Abort rather than fall back to single-thread if workers fail |

> **Worker Timeout** and **Blender Executable** can be overridden on the command line (see Headless mode below) but are not exposed in the panel.

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


**Batch processing:**

To run through all blender files in a directory, and output csvs with according names, place run_headless.py in the directory and run:

```
$blender = "C:\Program Files\Blender Foundation\Blender 5.0\blender.exe"
 
foreach ($blend in Get-ChildItem -Filter *.blend) {
 
    $sceneName = $blend.BaseName
    $outputCsv = "//$sceneName.csv"
 
    & $blender `
        --background $blend.FullName `
        --python .\run_headless.py `
        -- --export-path $outputCsv
}
```

### Troubleshooting headless runs

- **Workers fail immediately** — copy the `Starting worker N: <command>` line from the output and run it directly in a terminal to see the full error.
- **No progress output** — ensure the `.blend` is saved and the addon is enabled in that Blender build.
- **0 valid poses** — verify object/ACS assignments in the panel first; test with `--single-threaded --verbose`.
- **Workers silently hang** — reduce Worker Timeout in the panel or pass `--workers 1` to isolate the issue.

---

## Technical Details

### Collision detection
Poses are tested with a 3-tier cascade — each tier either resolves the result early or falls through to the next:

| Tier | Method | Result |
|---|---|---|
| **1 — Convex hull** | Convex hulls of proximal and distal meshes are tested for overlap using BVH trees. The proximal hull is computed once; the distal hull is transformed per pose. If the hulls do not overlap the meshes cannot overlap. | → **Valid** (early exit) |
| **2 — Penetration sampling** | A configurable number of evenly-sampled distal surface vertices are tested against the proximal mesh via ray-parity (odd crossing count = inside). If at least `penetration_definite_threshold` samples are inside, a collision is certain. | → **Invalid** (early exit) or ambiguous |
| **3 — Full BVH overlap** | A BVH tree is built for the distal mesh at the current pose and tested against the static proximal BVH tree. The proximal BVH is computed once at initialisation; the distal BVH is rebuilt per pose that reaches this tier. | → **Valid** or **Invalid** (definitive) |

The proximal BVH (and proximal convex hull BVH) are computed once before iteration begins. Workers each run a full Blender instance, apply poses via `view_layer.update()`, and stream results over stdout using a lightweight protocol (`ROMF_PROGRESS`, `ROMF_RESULT_START/END`, `ROMF_ERROR`).

### Object hierarchy
The distal mesh is parented under ACSm, which is parented under ACSf, which is parented under the proximal mesh. The proximal mesh is static; poses are applied by modifying the transforms of ACSf and ACSm.

### Module overview

| File | Role |
|---|---|
| `__init__.py` | Registration and metadata |
| `properties.py` | Property definitions |
| `operators.py` | Blender operators (calculation, cancel, confirm dialog, min-distance finder) |
| `processor.py` | ROM processing engine: pose iteration, worker spawning, CSV export, keyframe creation |
| `collision.py` | Self-contained 3-tier collision detector (`CollisionDetector` class) |
| `pose_math.py` | JCS pose matrix calculations (ISB Standard, Simplified, M&G Hinge) |
| `worker_headless.py` | Script run inside each worker Blender process |
| `run_headless.py` | Command-line entry point |
| `ui.py` | Panel layout |

---

## Scientific References

- Grood & Suntay (1983). *J Biomech Eng* 105(2):136–144.
- Wu et al. (2002). *J Biomechanics* 35(4):543–548.
- Manafzadeh & Gatesy (2021). *J Anatomy* 239(6):1516–1524.

---

*Results should be validated against known standards. Save your work frequently during intensive calculations.*

