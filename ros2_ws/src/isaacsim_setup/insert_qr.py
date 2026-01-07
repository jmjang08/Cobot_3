import os
import random
import omni.usd
from pxr import Sdf, Usd
import omni.kit.commands

def apply_random_qr_texture(shader_prim_path):
    # 1. Set the directory path containing your PNG files
    folder_path = "/home/rokey/Cobot_3/ros2_ws/src/simulation/maps/qr_images"
    
    if not os.path.exists(folder_path):
        print(f"Error: Directory not found: {folder_path}")
        return

    # 2. Filter for PNG files only
    png_files = [f for f in os.listdir(folder_path) if f.endswith('.png')]
    if not png_files:
        print("Error: No PNG files found in the folder.")
        return

    # 3. Pick a random file and create the asset path
    selected_file = random.choice(png_files)
    full_path = os.path.join(folder_path, selected_file)
    new_value = Sdf.AssetPath(full_path)

    # 4. Access the current stage and the specific Shader prim
    stage = omni.usd.get_context().get_stage()
    shader_prim = stage.GetPrimAtPath(shader_prim_path)
    
    if not shader_prim.IsValid():
        print(f"Error: Shader prim not found at path: {shader_prim_path}")
        return

    # 5. Get the current value for the 'prev' argument (required for Undo/Redo)
    attribute_name = "inputs:diffuse_texture"
    attr = shader_prim.GetAttribute(attribute_name)
    
    if not attr.IsValid():
        print(f"Error: Attribute '{attribute_name}' not found. Check your material type.")
        return

    prev_value = attr.Get()

    # 6. Execute the property change via Kit command
    property_path = f"{shader_prim_path}.{attribute_name}"
    
    success = omni.kit.commands.execute(
        "ChangeProperty",
        prop_path=property_path,
        value=new_value,
        prev=prev_value
    )

    if success:
        print(f"Success: Applied '{selected_file}' to {shader_prim_path}")
    else:
        print("Error: Command execution failed.")

# --- MAIN EXECUTION ---
target_shader_path = "/World/Environment/Box/mat_QR/Shader"

apply_random_qr_texture(target_shader_path)