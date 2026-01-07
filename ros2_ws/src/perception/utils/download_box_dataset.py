from roboflow import Roboflow

rf = Roboflow(api_key="YOUR_API_KEY_HERE")

workspace = "package-detection-iyziv"
project   = "cardboard-box-u35qd-eheqh"
version   = 1

proj = rf.workspace(workspace).project(project)
dataset = proj.version(version).download("yolov8", location="data/cardboard_box")

print("✅ Downloaded dataset in YOLOv8 format:")
print(dataset.location)  # data/cardboard_box
