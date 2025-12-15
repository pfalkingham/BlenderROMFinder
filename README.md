# Range of Motion (ROM) Finder for Blender

A Blender addon that systematically finds and records all possible 6-DOF (Degrees of Freedom) poses where two selected objects (a "proximal" and a "distal" object) do not collide. It iterates through user-defined ranges of Joint Coordinate System (JCS) rotations and translational offsets, performing collision checks at each step. This tool is particularly valuable for biomechanical studies, virtual paleontology, robotics, and animation.

**Version:** 3.7.4  
**Blender Version:** 4.4.0 or higher  
**Authors:** Peter Falkingham, Andréas Jannel, Ben Griffin, Rebecca Lowes (Liverpool John Moores University)  
**Video Tutorial:** [Watch on YouTube](https://youtu.be/sQL41YbC_TY) *(Note: Video may not reflect the latest features)*

## Key Features

### Core Functionality
- **6-DOF Analysis:** Calculates collision-free poses across all six degrees of freedom (3 rotations + 3 translations)
- **Object & Bone Support:** Works with both objects and specific bones within armatures as coordinate systems
- **Real-time Progress:** Modal operation with progress tracking, estimated time remaining, and cancellation support
- **High Performance:** BVH tree collision detection with optional convex hull pre-check optimization
- **Experimental Parallel Processor:** New high-performance optimized calculation mode for faster processing

### Three Integrated Calculation Modes

The addon features a unified system with three distinct modes that each combine specific rotation and translation methods:

1. **ISB Standard Mode**
   - **Rotation:** Implements ISB-compliant Joint Coordinate System (Z-Y'-X'' sequence) where the Adduction/Abduction axis (Y') is a "floating" axis perpendicular to both the FE axis and the mobile segment's long axis after FE
   - **Translation:** Applied along the rotated ACSm's local axes after all rotations are complete
   - **Use Case:** Standard biomechanical analysis following ISB guidelines (Grood & Suntay, 1983; Wu et al., 2002)

2. **Simplified Mode**  
   - **Rotation:** Similar to ISB but uses the ACSm's Y-axis (after FE rotation) for Adduction/Abduction instead of the floating axis
   - **Translation:** Applied along the rotated ACSm's local axes after all rotations are complete
   - **Use Case:** More visually intuitive for certain constrained joints, reduces "visual twist" artifacts

3. **M&G Hinge Mode**
   - **Rotation:** Uses the ACSm's Y-axis method (same as Simplified)
   - **Translation:** Special "prism" method where translation directions rotate with Flexion/Extension, maintaining anatomical consistency of terms like "distraction" and "A-P glide" across the FE range
   - **Use Case:** Hinge-like joints where translation meanings should remain anatomically consistent (inspired by Manafzadeh & Gatesy, 2021)

### Output & Visualization
- **CSV Export:** Complete dataset with rotation angles, translations, and collision status
- **Animation Layer:** Automatic keyframe creation for non-colliding poses viewable in NLA Editor
- **Debug Mode:** Option to visualize ALL poses (both colliding and non-colliding) for analysis
  - **Turn Off Collisions:** Sub-option to disable collision checking entirely when in debug mode

## Installation

1. Download or clone the addon files (ensure all files are in a folder named `BlenderROMFinder`):
   - `__init__.py`
   - `operators.py`
   - `properties.py`
   - `ui.py`
   - `poseCalculations.py`
   - `parallel_processor_v2.py`

2. In Blender, go to `Edit > Preferences > Add-ons`

3. Click `Install...` and navigate to the `.zip` file containing the addon folder (or select the folder if installing from directory)

4. Enable the addon by checking the box next to "Range of Motion Finder"

5. The panel will appear in the 3D View sidebar under the **"Collision"** tab (press `N` to show sidebar if hidden)

## Quick Start Guide

### 1. Object Setup
Set up your scene with the required objects:

- **Proximal Object:** The static/base object (e.g., femur in a knee joint)
- **Distal Object:** The moving object to test for collisions (e.g., tibia) 
- **ACS Fixed (ACSf):** Object or bone representing the fixed coordinate system (determines the Z-axis for Flexion/Extension)
- **ACS Mobile (ACSm):** Object or bone representing the mobile coordinate system (determines the X-axis for Long-Axis Rotation). The Distal Object should follow this coordinate system
- **Bone Selection:** If using armatures, select specific bones from the dropdown menus that appear when an armature is selected

**Setup Tip:** For predictable results, align ACSm's local axes with ACSf's local axes initially (use Delta Transforms for visual offsets).

### 2. Choose Calculation Mode
Select one of three modes from the **"Rotation Logic"** dropdown at the top of the panel:

- **ISB Standard:** Full ISB compliance with floating Y' axis
- **Simplified:** Reduces visual artifacts, easier to understand  
- **M&G Hinge:** Best for hinge joints where translation meanings should stay consistent

### 3. Define Parameter Ranges
Set ranges and increments for the six degrees of freedom:

**Rotations (degrees):**
- **X (Long-Axis Rotation):** Internal/external rotation around the long axis
- **Y (Adduction/Abduction):** Side-to-side movement  
- **Z (Flexion/Extension):** Forward/backward bending

**Translations (Blender units):**
- **X, Y, Z:** Linear movements (interpretation depends on selected mode)

Use the Min, Max, and Step fields for each axis to define the search space.

- **Total poses to search:** The panel now shows a live count under the Translation box labeled "Total poses to search: XX". This is computed as the product of the number of steps for each of the six axes (counts use the same inclusive stepping logic as the search) and updates immediately when you change any Min/Max/Step.

### 4. Configure Output
- **Export to CSV:** Enable and set file path for data export (uses `//` for paths relative to .blend file). Specify if you want all poses or only valid poses exported.
- **Only export valid poses:** A checkbox next to `Export to CSV` will restrict the CSV to only include valid (non-colliding) poses when enabled. When disabled, all poses (colliding and valid) are exported.
- **Partial exports on cancel:** If you cancel a run mid-calculation, the addon will still write any collected CSV rows (partial export) so you don't lose progress.
- **Show Animation Layer:** Creates keyframes for valid poses (view in NLA Editor)
- **Debug Mode:** Shows ALL poses including collisions (for troubleshooting)
  - When enabled, also shows option to "Turn Off Collisions" to see all poses without collision checking

### 5. Performance Settings  
- **Batch Size:** Number of iterations per update (10-5000). Higher values = faster but less responsive UI (10-100 recommended)
- **Convex Hull Pre-Check:** Enable for speed boost. **WARNING:** May give incorrect results if one object can be fully contained within the other. Disable for full accuracy in such cases.

### 6. Run Analysis
Choose one of two calculation methods:

#### Standard Calculation
1. Click **"Confirm and Calculate Collisions"**
2. Confirm the calculation in the dialog
3. Monitor progress in the UI
4. Press **Cancel** or `ESC` to stop if needed

#### High-Performance Optimized (Experimental)
1. Click **"High-Performance Optimized (experimental)"** button
2. Uses efficient batching with the same collision logic as the original
3. Should provide similar or better performance

Check results in the CSV file and/or NLA Editor when complete.

## Technical Details

### Joint Coordinate System (JCS) Implementation
The addon implements a consistent Z-Y'-X'' rotation sequence:

1. **Flexion/Extension (Z):** Around ACSf's local Z-axis
2. **Adduction/Abduction (Y'):** Axis definition varies by mode:
   - **ISB Standard:** Floating axis perpendicular to Z and current X
   - **Simplified/M&G:** ACSm's Y-axis after FE rotation
3. **Long-Axis Rotation (X):** Around ACSm's final X-axis

### Translation Methods
- **ISB Standard & Simplified:** Translations applied along ACSm's final rotated axes in world space
- **M&G Hinge:** Translations along "prism" axes that rotate only with FE, preserving anatomical meaning

The pose calculation logic has been modularized in `poseCalculations.py` for maintainability.

### Performance Optimization
- **BVH Trees:** Fast spatial collision detection using Blender's BVH tree system
- **Convex Hull Pre-check:** Optional fast elimination of obviously non-colliding poses
- **Batch Processing:** Configurable batch sizes maintain UI responsiveness
- **Modal Operation:** Non-blocking execution with progress tracking and cancellation
- **Parallel Processor:** Experimental optimized processing mode (`parallel_processor_v2.py`) for improved performance

### Output Format
CSV files contain these columns:
- `rot_x`, `rot_y`, `rot_z`: Input rotation angles (degrees)  
- `trans_x`, `trans_y`, `trans_z`: Input translation distances (Blender units)
- `Valid_pose`: 1 = no collision, 0 = collision detected

Note: When "Only export valid poses" is enabled, only rows with `Valid_pose == 1` are written to the CSV.

## Important Notes & Best Practices

### Setup Recommendations
- **Coordinate System Alignment:** Start with ACSm's local axes aligned with ACSf's axes for predictable results
- **Delta Transforms:** Use `Ctrl+A > Apply to Deltas` for visual offsets while keeping main transforms at identity
- **Object Origins:** ACSm's origin defines the center of rotation - position carefully
- **Parent Relationships:** Ensure Distal Object follows ACSm's movements through parenting or constraints
- **Parent Scaling:** The addon now properly handles parent object scaling in world-space translations

### Understanding "Visual Twist"
The ISB Standard mode can produce apparent "roll" even with zero Long-Axis Rotation input. This is mathematically correct behavior due to gimbal effects when combining FE and AD/AB rotations. Use Simplified mode to minimize this effect if problematic.

### Troubleshooting
- **Convex Hull Warning:** Disable convex hull optimization if one object can fit entirely inside another
- **Performance Issues:** Reduce batch size or simplify mesh geometry for complex models. Try the experimental parallel processor.
- **Unexpected Results:** Enable Debug Mode to visualize all poses and verify coordinate system setup
- **Bone Selection Issues:** Ensure you select the armature object first, then the bone dropdown will populate
- **Save Often:** Complex biomechanical calculations can be processor-intensive

### When to Use Each Mode
- **ISB Standard:** Academic/clinical studies requiring ISB compliance
- **Simplified:** Teaching, visualization, or when visual artifacts are problematic  
- **M&G Hinge:** Hinge joints where translation terminology must remain anatomically consistent

## Scientific References

This addon implements established biomechanical concepts from these key sources:

- **Grood, E. S., & Suntay, W. J. (1983).** A joint coordinate system for the clinical description of three-dimensional motions: application to the knee. *Journal of Biomechanical Engineering*, 105(2), 136-144.

- **Wu, G., et al. (2002).** ISB recommendation on definitions of joint coordinate system of various joints for the reporting of human joint motion—part I: ankle, hip, and spine. *Journal of Biomechanics*, 35(4), 543-548.

- **Manafzadeh, A. R., & Gatesy, S. M. (2021).** Paleobiological reconstructions of articular function require all six degrees of freedom. *Journal of Anatomy*, 239(6), 1516-1524.

## Development & Support

**Current Version:** 3.7.1  
**Minimum Blender Version:** 4.4.0  
**Authors:** Peter Falkingham, Andréas Jannel, Ben Griffin (Liverpool John Moores University)  
**Acknowledgments:** Development assisted by various AI/LLMs, especially Google AI Studio

### Architecture
The addon is organized into several modules:
- `__init__.py` - Registration and addon metadata
- `properties.py` - Blender property definitions
- `operators.py` - real time operator 
- `parallel_processor_v2.py` - optimized processor (faster, but can lock up Blender UI temporarily)
- `poseCalculations.py` - JCS pose calculation logic
- `ui.py` - User interface panel

### Known Issues & Future Enhancements
- Exploring SDF (Signed Distance Fields) for enhanced collision detection
- Potential integration with biomechanical analysis pipelines
- Continued optimization of parallel processing

### Contributing
Contributions and feedback are welcome! This addon implements complex biomechanical and mathematical concepts. Always validate results against known standards and save your work frequently.

## License & Citation

If you use this addon in your research, please cite the relevant scientific papers listed above and acknowledge the tool in your methods section.

---

**Note:** This addon implements complex biomechanical and mathematical concepts. Results should be validated against known standards. Always save your work frequently when performing intensive calculations.