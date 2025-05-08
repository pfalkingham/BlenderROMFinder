# Range of Motion (ROM) Finder for Blender

This Blender addon is designed to find and record all possible poses where two selected objects (a "proximal" and a "distal" object) do not collide. It iterates through a defined range of rotations and translations, performing collision checks at each step.

**Video Tutorial:** [Watch on YouTube](https://youtu.be/sQL41YbC_TY) (Note: Video may not reflect the latest UI or features)

## Features

*   Calculates collision-free poses based on user-defined rotation and translation ranges.
*   Supports rotation around an object\'s origin or a specified bone in an armature.
*   Outputs results to a CSV file.
*   Optionally stores results as custom attributes on the rotational object.
*   Optionally creates an animation layer (NLA strip) visualizing all non-colliding poses.
*   Configurable performance settings.

## Installation

1.  Download the addon (ensure all files like `__init__.py`, `operators.py`, `properties.py`, `ui.py` are in a folder, e.g., `BlenderROMFinder`).
2.  In Blender, go to `Edit > Preferences > Add-ons`.
3.  Click `Install...` and navigate to the `.zip` file of the addon folder (or the `__init__.py` if installing from loose files, though zipping the folder is usually best).
4.  Enable the addon by checking the box next to its name ("Range of Motion Finder").

## How to Use

The addon panel will appear in the 3D View sidebar (press `N` if hidden) under the "Collision" tab (or "ROM" depending on version).

### 1. Object Selection

*   **Proximal Object:** Select the object that is considered static or the base. Collisions will be checked against this object.
*   **Distal Object:** Select the object that will be moved and rotated.
*   **Rotational Object:**
    *   If the `Distal Object` itself has the correct pivot point for rotation (e.g., you\'ve moved its origin to the desired joint center and applied transformations), set the `Distal Object` as the `Rotational Object`.
    *   If the `Distal Object` is part of an armature and you want to rotate a specific bone, set the Armature object as the `Rotational Object`.
*   **Rotational Bone:** If an Armature is selected as the `Rotational Object`, this dropdown will list its bones. Select the bone that will be rotated. The rotation will occur around this bone\'s head.

### 2. Rotation Parameters

Define the range and increment for rotations around the X, Y, and Z axes of the `Rotational Object` (or `Rotational Bone`).
*   **Min/Max (degrees):** The minimum and maximum rotation angles for each axis.
*   **Step (degrees):** The increment for each rotational step.
*   **Order:** The Euler rotation order (e.g., XYZ, ZYX). This defines the sequence in which rotations are applied.
    *   **Important for Keyframes:** For the keyframed animation to display "clean" rotation values (e.g., 10, 20, 30 degrees) in Blender\'s UI, ensure the `Rotation Mode` of your `Rotational Object` (in Object Properties) or `Rotational Bone` (in Pose Mode > Bone Properties > Rotation Mode) matches the `Order` selected in this addon.

### 3. Translation Parameters

Define the range and increment for translations along the X, Y, and Z axes of the `Rotational Object` (or `Rotational Bone`).
*   **Min/Max:** The minimum and maximum translation offsets.
*   **Step:** The increment for each translational step.
    *   Note: Translations are applied relative to the initial position of the rotational object/bone, in its local coordinate system after rotation.

### 4. Output Settings

*   **Export to CSV:**
    *   Enable to save the results to a CSV file.
    *   **Export Path:** Specify the location and name for the CSV file. The CSV will contain the delta rotations, absolute world rotations, delta translations, and a "Valid\_pose" column (1 for no collision, 0 for collision).
*   **Store as Attributes:**
    *   Enable to store the collision data as custom attributes on the `Rotational Object`.
    *   **Attribute Prefix:** A prefix for the names of the custom attributes (e.g., `collision_data`, `collision_count`).
*   **Show Animation Layer:**
    *   Enable to create keyframes for every non-colliding pose. These keyframes are applied to the `Rotational Object` or `Rotational Bone`.
    *   An NLA (Non-Linear Animation) strip will be created to make it easy to view and manage the animation.

### 5. Performance Options

*   **Batch Size:** The number of poses to calculate before updating the UI. Higher values can speed up the overall process but make the UI less responsive during calculation.
*   **Use Convex Hull Pre-Check:**
    *   Default: **Disabled**.
    *   Enable to use a faster convex hull intersection test as an initial check. If the convex hulls of the objects don\'t overlap, the pose is quickly marked as non-colliding.
    *   **WARNING:** This optimization can give incorrect non-collision results (false negatives) if one object can be fully contained within the other without their hulls intersecting. If you suspect this (e.g., a small object moving inside a larger, hollow object), **disable this option** for full accuracy using a direct mesh-to-mesh check.

### 6. Running the Calculation

*   Click **Calculate Collisions** to start the process.
*   A progress bar and estimated time remaining will appear.
*   Click **Cancel Calculation** to stop the process prematurely.

## Important Notes & Troubleshooting

*   **"NOT HEAVILY TESTED":** This addon is under development. Please save your work before running extensive calculations.
*   **Object Origins/Pivots:** The accuracy of rotation depends heavily on the correct placement of the `Rotational Object`\'s origin or the `Rotational Bone`\'s head.
*   **Mesh Complexity:** Very high-poly meshes will take longer to process.
*   **Rotation Mode Matching:** As mentioned, for clean keyframe values in the UI, match the object/bone `Rotation Mode` in Blender to the addon\'s `Order` setting.
*   **Convex Hull for Contained Objects:** If one object can fit entirely inside another, disable the "Use Convex Hull Pre-Check" for accurate collision detection.

## Known Issues / Future Ideas
*   SDF (Signed Distance Fields) could be explored for more robust and potentially faster collision, especially for containment scenarios (pending Blender API developments).

Contributions and feedback are welcome!
