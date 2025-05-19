# Range of Motion (ROM) Finder for Blender

This Blender addon is designed to find and record all possible 6-DOF (Degrees of Freedom) poses where two selected objects (a "proximal" and a "distal" object) do not collide. It systematically iterates through user-defined ranges of JCS (Joint Coordinate System) rotations and various translational offsets, performing collision checks at each step. This tool is particularly useful for biomechanical studies, virtual paleontology, and robotics.

**Video Tutorial:** [Watch on YouTube](https://youtu.be/sQL41YbC_TY) *(Note: Video may not reflect the latest UI or features)*

## Features

-   Calculates collision-free 6-DOF poses based on user-defined JCS rotation and translation ranges.
-   Supports rotation and translation of a mobile coordinate system (`ACSm`) relative to a fixed coordinate system (`ACSf`), which can be objects or specific bones within armatures.
-   **JCS Rotation Logic Modes:**
    -   **ISB Standard:** Implements a Joint Coordinate System (e.g., Z-Y'-X'' sequence for Flexion/Extension, Adduction/Abduction, Long-Axis Rotation) where the Adduction/Abduction axis (Y') is a "floating" axis, perpendicular to both the FE axis and the mobile segment's long axis after FE. This aligns with common biomechanics standards (e.g., Grood & Suntay, 1983; Wu et al., 2002).
    -   **Intuitive (Fixed Y for AD/AB):** Adduction/Abduction occurs around the fixed local Y-axis of the `ACSf` object/bone. This can be more visually intuitive for certain constrained joints.
-   **Translation Application Modes:**
    -   **Simple (ACSf Local):** Translations are applied along the fixed local axes of the `ACSf` object/bone. The JCS rotation then occurs from this translated position.
    -   **ACSm Local (Post-Rotation):** Translations are applied along the axes of the `ACSm` object/bone *after* all JCS rotations for that pose have been applied. Useful for representing joint "play" at a specific orientation.
    -   **M&G Prism (Hinge):** For hinge-like joints, translations are applied along the axes of a virtual prism (inspired by Manafzadeh & Gatesy, 2021) that rotates with the primary Flexion/Extension of the joint, ensuring anatomical consistency of translational terms (e.g., "distraction," "A-P glide") across the FE range.
-   Outputs results (JCS angles, translations, collision status) to a CSV file.
-   Optionally creates an animation layer (NLA strip) visualizing all non-colliding poses.
-   Configurable performance settings, including batch size and convex hull pre-check.
-   Progress bar and estimated time remaining during calculations.
-   Ability to cancel calculations.

## Installation

1.  Download the addon (ensure all files like `__init__.py`, `operators.py`, `properties.py`, `ui.py` are in a folder, e.g., `BlenderROMFinder`).
2.  In Blender, go to `Edit > Preferences > Add-ons`.
3.  Click `Install...` and navigate to the `.zip` file of the addon folder.
4.  Enable the addon by checking the box next to its name ("Range of Motion Finder").

## How to Use

The addon panel appears in the 3D View sidebar (press `N` if hidden) under the "Collision" tab.

### 1. Setup Objects & Coordinate Systems

-   **Proximal Object:** The static or base object against which collisions are checked.
-   **Distal Object:** The moving object whose mesh is checked for collision with the Proximal Object.
-   **ACS Fixed (ACSf):** An object (often an Empty or a bone) representing the fixed/proximal anatomical coordinate system. JCS Flexion/Extension (typically `rot_z`) will occur around this object's local Z-axis.
-   **ACS Mobile (ACSm):** An object (often an Empty or a bone, typically parented to `ACSf`) representing the mobile/distal anatomical coordinate system. JCS Long-Axis Rotation (typically `rot_x`) will occur around this object's local X-axis after other rotations. The `Distal Object` should be parented or constrained to `ACSm` to follow its movements.
-   **ACSf/ACSm Bone:** If `ACSf` or `ACSm` is an Armature, select the specific bone to define the coordinate system.

*Initial Alignment:* For predictable JCS behavior, it's recommended that `ACSm` starts with its local axes aligned with `ACSf`'s local axes (i.e., `ACSm`'s `matrix_local` relative to `ACSf` is an identity matrix, or only has a translational component if their origins are offset). Visual offsets/rotations should be in their Delta Transforms.

### 2. Select Logic Modes

-   **Rotation Logic:** Choose between "ISB Standard" or "Intuitive (Fixed Y)".
-   **Translation Mode:** Choose between "Simple (ACSf Local)", "ACSm Local (Post-Rotation)", or "M&G Prism (Hinge)".

### 3. Rotation Parameters (JCS Component Angles)

Define the range and increment for the three JCS rotations. These are applied sequentially.
-   **rot_z (Flexion/Extension):** Min/Max/Step in degrees.
-   **rot_y (Adduction/Abduction):** Min/Max/Step in degrees.
-   **rot_x (Long-Axis Rotation):** Min/Max/Step in degrees.
-   **Order (Euler for Keyframing):** The Euler order used if keyframing the `ACSm` object's final `rotation_euler`. For best results in Blender's UI, match this to `ACSm`'s own rotation mode. *This does NOT define the JCS sequence itself, which is hardcoded as FE -> AD/AB -> LAR.*

### 4. Translation Parameters

Define the range and increment for translational offsets. Their meaning depends on the selected "Translation Mode".
-   **trans_x, trans_y, trans_z:** Min/Max/Step in Blender units.

### 5. Output Settings

-   **Export to CSV:** Enable and specify path. The CSV will contain the input JCS `rot_x,y,z` and `trans_x,y,z` values, and a "Valid_pose" column (1 for no collision, 0 for collision).
-   **Show Animation Layer:** Creates keyframes for the `ACSm` object/bone for every non-colliding pose. Viewable in the NLA Editor.

### 6. Performance Options

-   **Batch Size:** Number of poses per UI update.
-   **Use Convex Hull Pre-Check:** Faster initial check, but disable if `Distal Object` can fit entirely inside `Proximal Object` without hulls intersecting.

### 7. Running the Calculation

-   Click **Calculate Collisions**.
-   Monitor progress. Click **Cancel Calculation** to stop.

## Important Notes

-   **JCS Sequence:** The addon internally applies JCS rotations in the order: 1st Flexion/Extension (`rot_z`), 2nd Adduction/Abduction (`rot_y`), 3rd Long-Axis Rotation (`rot_x`).
-   **"Visual Twist":** The "ISB Standard" rotation logic can produce a "visual twist" (an apparent roll) when FE and AD/AB are combined, even if input LAR is zero. This is a mathematical property of such sequential rotations (Gimbal effect/coupling). The "Intuitive (Fixed Y)" mode reduces this for AD/AB.
-   **M&G Prism Mode:** For the "M&G Prism (Hinge)" translation mode, make sure to properly set axes for distraction/compression, AP and ML movement.
-   **Delta Transforms:** It's recommended to use Delta Transforms (Ctrl+A > Apply to Deltas) for the initial visual setup of `ACSf` and `ACSm` if their main transform values need to be at identity/zero for the script's logic (especially for `ACSm`'s initial local matrix relative to `ACSf`).
-   **Object Origins:** The origin of `ACSm` (or head of `ACSm_bone`) is crucial as it defines the point around which JCS rotations are conceptually applied and from which local translations (in "ACSm Local" mode) originate.
-   **Experimental:** This addon implements complex biomechanical concepts. Always save your work and test with simple setups first.

## Relevant Biomechanics References

-   Grood, E. S., & Suntay, W. J. (1983). A joint coordinate system for the clinical description of three-dimensional motions: application to the knee. *Journal of Biomechanical Engineering*, 105(2), 136-144.
-   Wu, G., Siegler, S., Allard, P., Kirtley, C., Leardini, A., Rosenbaum, D., ... & Stokes, I. (2002). ISB recommendation on definitions of joint coordinate system of various joints for the reporting of human joint motion—part I: ankle, hip, and spine. *Journal of Biomechanics*, 35(4), 543-548.
-   Manafzadeh, A. R., & Gatesy, S. M. (2021). Paleobiological reconstructions of articular function require all six degrees of freedom. *Journal of Anatomy*, 239(6), 1516-1524. (For the prism translation concept).

## Known Issues / Future Ideas

-   Explore SDF (Signed Distance Fields) for collision detection.

Contributions and feedback are welcome!
