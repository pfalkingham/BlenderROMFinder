# Range of Motion (ROM) Finder for Blender

This Blender addon systematically finds and records all possible 6-DOF (Degrees of Freedom) poses where two selected objects (a "proximal" and a "distal" object) do not collide. It iterates through user-defined ranges of Joint Coordinate System (JCS) rotations and translational offsets, performing collision checks at each step. This tool is particularly valuable for biomechanical studies, virtual paleontology, robotics, and animation.

**Version:** 3.0.2  
**Video Tutorial:** [Watch on YouTube](https://youtu.be/sQL41YbC_TY) *(Note: Video may not reflect the latest features)*

## Key Features

### Core Functionality
-   **6-DOF Analysis:** Calculates collision-free poses across all six degrees of freedom (3 rotations + 3 translations)
-   **Object & Bone Support:** Works with both objects and specific bones within armatures as coordinate systems
-   **Real-time Progress:** Modal operation with progress tracking, estimated time remaining, and cancellation
-   **High Performance:** BVH tree collision detection with optional convex hull pre-check optimization

### Three Integrated Calculation Modes

The addon now features a unified system with three distinct modes that each combine specific rotation and translation methods:

1. **ISB Standard Mode**
   - **Rotation:** Implements ISB-compliant Joint Coordinate System (Z-Y'-X'' sequence) where the Adduction/Abduction axis (Y') is a "floating" axis perpendicular to both the FE axis and the mobile segment's long axis after FE
   - **Translation:** Applied along the rotated ACSm's local axes after all rotations are complete
   - **Use Case:** Standard biomechanical analysis following ISB guidelines (Grood & Suntay, 1983; Wu et al., 2002)

2. **Simplified Mode**  
   - **Rotation:** Similar to ISB but uses the ACSm's Y-axis (after FE rotation) for Adduction/Abduction instead of the floating axis
   - **Translation:** Applied along the rotated ACSm's local axes after all rotations are complete
   - **Use Case:** More visually intuitive for certain constrained joints, reduces "visual twist" artifacts

3. **M&G Hinge Mode**
   - **Rotation:** Uses the ACSm's Y-axis method (same as Intuitive)
   - **Translation:** Special "prism" method where translation directions rotate with Flexion/Extension, maintaining anatomical consistency of terms like "distraction" and "A-P glide" across the FE range
   - **Use Case:** Hinge-like joints where translation meanings should remain anatomically consistent (inspired by Manafzadeh & Gatesy, 2021)

### Output & Visualization
-   **CSV Export:** Complete dataset with rotation angles, translations, and collision status
-   **Animation Layer:** Automatic keyframe creation for non-colliding poses viewable in NLA Editor
-   **Debug Mode:** Option to visualize ALL poses (both colliding and non-colliding) for analysis

## Installation

1. Download the addon files (ensure all files: `__init__.py`, `operators.py`, `properties.py`, `ui.py` are in a folder named `BlenderROMFinder`)
2. In Blender, go to `Edit > Preferences > Add-ons`
3. Click `Install...` and navigate to the `.zip` file of the addon folder
4. Enable the addon by checking the box next to "Range of Motion Finder"
5. The panel will appear in the 3D View sidebar under the "Collision" tab (press `N` to show sidebar if hidden)

## Quick Start Guide

### 1. Object Setup
Set up your scene with the required objects:

- **Proximal Object:** The static/base object (e.g., femur in a knee joint)
- **Distal Object:** The moving object to test for collisions (e.g., tibia) 
- **ACS Fixed (ACSf):** Object or bone representing the fixed coordinate system (determines the Z-axis for Flexion/Extension)
- **ACS Mobile (ACSm):** Object or bone representing the mobile coordinate system (determines the X-axis for Long-Axis Rotation). The Distal Object should follow this coordinate system
- **Bone Selection:** If using armatures, select specific bones from the dropdown menus

**Setup Tip:** For predictable results, align ACSm's local axes with ACSf's local axes initially (use Delta Transforms for visual offsets).

### 2. Choose Calculation Mode
Select one of three modes from the "Rotation Logic" dropdown:

- **ISB Standard:** Full ISB compliance with floating Y' axis
- **Intuitive:** Reduces visual artifacts, easier to understand  
- **M&G Hinge:** Best for hinge joints where translation meanings should stay consistent

### 3. Define Parameter Ranges
Set ranges and increments for the six degrees of freedom:

**Rotations (degrees):**
- **X (Long-Axis Rotation):** Internal/external rotation around the long axis
- **Y (Adduction/Abduction):** Side-to-side movement  
- **Z (Flexion/Extension):** Forward/backward bending

**Translations (Blender units):**
- **X, Y, Z:** Linear movements (meaning depends on selected mode)

### 4. Configure Output
- **Export to CSV:** Enable and set file path for data export
- **Show Animation Layer:** Creates keyframes for valid poses (view in NLA Editor)
- **Debug Mode:** Shows ALL poses including collisions (for troubleshooting)

### 5. Performance Settings  
- **Batch Size:** Higher values = faster but less responsive UI (10-100 recommended)
- **Convex Hull Pre-Check:** Enable for speed boost (disable if one object fits inside the other)

### 6. Run Analysis
1. Click **Calculate Collisions**
2. Monitor progress in the UI
3. Press **Cancel** or `ESC` to stop if needed
4. Check results in CSV file and/or NLA Editor when complete

## Technical Details

### Joint Coordinate System (JCS) Implementation
The addon implements a consistent Z-Y'-X'' rotation sequence:
1. **Flexion/Extension (Z):** Around ACSf's local Z-axis
2. **Adduction/Abduction (Y'):** Axis definition varies by mode:
   - ISB Standard: Floating axis perpendicular to Z and current X
   - Intuitive/M&G: ACSm's Y-axis after FE rotation
3. **Long-Axis Rotation (X):** Around ACSm's final X-axis

### Translation Methods
- **ISB Standard & Intuitive:** Translations applied along ACSm's final rotated axes
- **M&G Hinge:** Translations along "prism" axes that rotate only with FE, preserving anatomical meaning

### Performance Optimization
- **BVH Trees:** Fast spatial collision detection using Blender's BVH tree system
- **Convex Hull Pre-check:** Optional fast elimination of obviously non-colliding poses
- **Batch Processing:** Configurable batch sizes maintain UI responsiveness
- **Modal Operation:** Non-blocking execution with progress tracking and cancellation

### Output Format
CSV files contain these columns:
- `rot_x`, `rot_y`, `rot_z`: Input rotation angles (degrees)  
- `trans_x`, `trans_y`, `trans_z`: Input translation distances (Blender units)
- `Valid_pose`: 1 = no collision, 0 = collision detected

## Important Notes & Best Practices

### Setup Recommendations
- **Coordinate System Alignment:** Start with ACSm's local axes aligned with ACSf's axes for predictable results
- **Delta Transforms:** Use `Ctrl+A > Apply to Deltas` for visual offsets while keeping main transforms at identity
- **Object Origins:** ACSm's origin defines the center of rotation - position carefully
- **Parent Relationships:** Ensure Distal Object follows ACSm's movements through parenting or constraints

### Understanding "Visual Twist"
The ISB Standard mode can produce apparent "roll" even with zero Long-Axis Rotation input. This is mathematically correct behavior due to gimbal effects when combining FE and AD/AB rotations. Use Intuitive mode to minimize this effect if problematic.

### Troubleshooting
- **Convex Hull Warning:** Disable convex hull optimization if one object can fit entirely inside another
- **Performance Issues:** Reduce batch size or simplify mesh geometry for complex models
- **Unexpected Results:** Enable Debug Mode to visualize all poses and verify coordinate system setup
- **Save Often:** Complex biomechanical calculations can be processor-intensive

### When to Use Each Mode
- **ISB Standard:** Academic/clinical studies requiring ISB compliance
- **Intuitive:** Teaching, visualization, or when visual artifacts are problematic  
- **M&G Hinge:** Hinge joints where translation terminology must remain anatomically consistent

## Scientific References

This addon implements established biomechanical concepts from these key sources:

- **Grood, E. S., & Suntay, W. J. (1983).** A joint coordinate system for the clinical description of three-dimensional motions: application to the knee. *Journal of Biomechanical Engineering*, 105(2), 136-144.

- **Wu, G., et al. (2002).** ISB recommendation on definitions of joint coordinate system of various joints for the reporting of human joint motion—part I: ankle, hip, and spine. *Journal of Biomechanics*, 35(4), 543-548.

- **Manafzadeh, A. R., & Gatesy, S. M. (2021).** Paleobiological reconstructions of articular function require all six degrees of freedom. *Journal of Anatomy*, 239(6), 1516-1524.

## Development & Support

**Version:** 3.0.1  
**Authors:** Peter Falkingham, Andréas Jannel, Ben Griffin (Liverpool John Moores University)  
**Additional Thanks:** Various AI/LLM assistance including Google AI Studio

### Known Issues & Future Enhancements
- Exploring SDF (Signed Distance Fields) for enhanced collision detection
- Potential integration with biomechanical analysis pipelines

**Note:** This addon implements complex biomechanical and mathematical concepts. Always validate results against known standards and save your work frequently. Contributions and feedback are welcome!
