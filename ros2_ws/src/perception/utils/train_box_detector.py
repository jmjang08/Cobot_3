from ultralytics import YOLO

# 1) base 모델
model = YOLO("yolov8n.pt")

# 2) 학습
model.train(
    data="data/cardboard_box/data.yaml",
    epochs=100,
    imgsz=640,
    batch=16,
    project="runs_tiago_box",
    name="yolov8n-cardboard",
)
