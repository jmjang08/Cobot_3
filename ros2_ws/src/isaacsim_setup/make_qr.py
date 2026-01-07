import qrcode
import os

current_dir = os.path.dirname(os.path.abspath(__file__))

save_dir = os.path.normpath(os.path.join(current_dir, "../../simulation/maps/qr_images"))

if not os.path.exists(save_dir):
    os.makedirs(save_dir)
    print(f"Created directory: {save_dir}")

for i in range(1301, 1309):
    # 4. Generate QR code
    text = f"Room {i}"
    img = qrcode.make(text)
    
    # 5. Define file name and full path
    file_name = f"qr_{i}.png"
    file_path = os.path.join(save_dir, file_name)
    
    # 6. Save the image
    img.save(file_path)
    print(f"Saved: {file_path}")
