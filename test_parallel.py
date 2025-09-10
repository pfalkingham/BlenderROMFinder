# Test script for the new parallel ROM processor
# This file can be used to test the parallel processing functionality

import bpy
import time
from mathutils import Matrix, Vector

def setup_test_scene():
    """Set up a simple test scene with two cubes"""
    # Clear existing mesh objects
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False, confirm=False)
    
    # Add proximal cube
    bpy.ops.mesh.primitive_cube_add(location=(0, 0, 0))
    prox_cube = bpy.context.active_object
    prox_cube.name = "Proximal_Cube"
    
    # Add distal cube (slightly offset)
    bpy.ops.mesh.primitive_cube_add(location=(3, 0, 0))
    dist_cube = bpy.context.active_object
    dist_cube.name = "Distal_Cube"
    
    # Add ACSf (coordinate system fixed)
    bpy.ops.object.empty_add(type='ARROWS', location=(0, 0, 0))
    acsf = bpy.context.active_object
    acsf.name = "ACSf"
    
    # Add ACSm (coordinate system mobile)
    bpy.ops.object.empty_add(type='ARROWS', location=(3, 0, 0))
    acsm = bpy.context.active_object
    acsm.name = "ACSm"
    
    # Parent distal cube to ACSm
    dist_cube.parent = acsm
    
    return prox_cube, dist_cube, acsf, acsm

def configure_rom_properties(prox_cube, dist_cube, acsf, acsm):
    """Configure ROM finder properties for testing"""
    props = bpy.context.scene.collision_props
    
    # Set objects
    props.proximal_object = prox_cube
    props.distal_object = dist_cube
    props.ACSf_object = acsf
    props.ACSm_object = acsm
    
    # Set small ranges for quick testing
    props.rot_x_min = -10.0
    props.rot_x_max = 10.0
    props.rot_x_inc = 5.0
    
    props.rot_y_min = -10.0
    props.rot_y_max = 10.0
    props.rot_y_inc = 5.0
    
    props.rot_z_min = -10.0
    props.rot_z_max = 10.0
    props.rot_z_inc = 5.0
    
    # No translation for simple test
    props.trans_x_min = 0.0
    props.trans_x_max = 0.0
    props.trans_x_inc = 0.1
    
    props.trans_y_min = 0.0
    props.trans_y_max = 0.0
    props.trans_y_inc = 0.1
    
    props.trans_z_min = 0.0
    props.trans_z_max = 0.0
    props.trans_z_inc = 0.1
    
    # Set output options
    props.export_to_csv = True
    props.export_path = "//test_parallel_results.csv"
    props.visualize_collisions = True
    
    print("✅ ROM properties configured for testing")
    print(f"📊 Expected combinations: 5×5×5×1×1×1 = 125 poses")

def run_test():
    """Run the full test"""
    print("🧪 Starting ROM Finder Parallel Processing Test")
    
    # Setup scene
    print("🏗️  Setting up test scene...")
    prox_cube, dist_cube, acsf, acsm = setup_test_scene()
    
    # Configure properties
    print("⚙️  Configuring ROM properties...")
    configure_rom_properties(prox_cube, dist_cube, acsf, acsm)
    
    print("🚀 Test setup complete!")
    print("💡 You can now test the parallel processor:")
    print("   1. Go to the Collision panel")
    print("   2. Click 'High-Performance Parallel' button")
    print("   3. Check the console for performance metrics")
    print("   4. Results will be saved to test_parallel_results.csv")

if __name__ == "__main__":
    run_test()
