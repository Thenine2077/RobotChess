from ultralytics import YOLO
from pathlib import Path

# --- 1. กำหนดพาธน้ำหนัก -------------------------------------------------------
weights_path = Path(r"C:/Chess/RobotChess/runs/detect/train5/weights/best.pt")

# --- 2. โหลดโมเดล --------------------------------------------------------------
model = YOLO("C:/Chess/RobotChess/runs/detect/train5/weights/best.pt")         # ถ้าอยากเทสต์เร็ว ๆ ใช้ YOLO("yolov8n.pt") ก็ได้

# --- 3. รันกล้อง (Webcam = index 0) -----------------------------------------
for result in model(source=0, stream=True, show=True):
    # result เป็น ultralytics.engine.results.Results
    # คุณสามารถเข้าถึง result.boxes, result.masks, result.names ฯลฯ
    pass
