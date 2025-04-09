import cv2
import numpy as np
import os
from ultralytics import YOLO

def yolo_detect(image, model, conf_threshold=0.5):
    """
    ตรวจจับวัตถุในภาพด้วย YOLO โดยใช้ model ที่โหลดจากไฟล์ best.pt
    คืนค่า list ของ detection แต่ละตัวเป็น dict ที่มี keys:
      - 'bbox': [x, y, w, h]
      - 'label': ชื่อหมาก (เช่น 'white_knight', 'black_bishop', ...)
    """
    results = model(image, conf=conf_threshold)[0]  # ผลลัพธ์สำหรับภาพเดียว
    detections = []
    if results.boxes is not None:
        boxes = results.boxes.xyxy.cpu().numpy()  # รูปแบบ [x1, y1, x2, y2]
        class_ids = results.boxes.cls.cpu().numpy()
        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box
            x = int(x1)
            y = int(y1)
            w = int(x2 - x1)
            h = int(y2 - y1)
            label = model.names[int(class_ids[i])]
            detections.append({'bbox': [x, y, w, h], 'label': label})
    return detections

def map_to_square(center, board_width, board_height):
    """
    แปลงตำแหน่ง center (ของ bounding box) ให้เป็นชื่อช่องบนกระดาน 8x8 (A1-H8)
    โดยสมมติว่าภาพมีขนาด board_width x board_height
    """
    cell_width = board_width // 8
    cell_height = board_height // 8
    x, y = center
    col = int(x // cell_width)
    row = int(y // cell_height)
    col_labels = ['A','B','C','D','E','F','G','H']
    row_labels = [8,7,6,5,4,3,2,1]
    col = min(max(col, 0), 7)
    row = min(max(row, 0), 7)
    return f"{col_labels[col]}{row_labels[row]}"

def create_board_state(detections, board_width, board_height):
    """
    สร้าง board state โดย mapping ตำแหน่ง center ของ bounding box detection
    ลงในช่องบนกระดาน (เช่น 'B2': 'white_knight')
    """
    state = {}
    for det in detections:
        x, y, w, h = det['bbox']
        center = (x + w/2, y + h/2)
        square = map_to_square(center, board_width, board_height)
        state[square] = det['label']
    return state

def main():
    # ระบุ path ของภาพ warped before/after
    path_before = 'D:/Project RobotChess/RobotChess/game_chess/GUI/black_Bishop-4.jpg'
    path_after = 'D:/Project RobotChess/RobotChess/game_chess/GUI/black_Bishop-10.jpg'
    
    if not os.path.exists(path_before) or not os.path.exists(path_after):
        print("ไม่พบไฟล์ภาพกระดาน กรุณาตรวจสอบ path")
        return
    
    warped_before = cv2.imread(path_before)
    warped_after = cv2.imread(path_after)
    
    if warped_before is None or warped_after is None:
        print("เกิดปัญหาในการโหลดภาพ warped")
        return
    
    if warped_before.shape != warped_after.shape:
        print("ขนาดภาพ before/after ไม่ตรงกัน!")
        return
    
    board_height, board_width = warped_before.shape[:2]
    
    # โหลด YOLO model จากไฟล์ best.pt
    model_path = "D:/Project RobotChess/RobotChess/game_chess/runs/detect/train3/weights/best.pt"
    if not os.path.exists(model_path):
        print("ไม่พบไฟล์ model best.pt กรุณาตรวจสอบ path")
        return
    
    model = YOLO(model_path)
    
    # ตรวจจับหมากในภาพ warped before/after
    detections_before = yolo_detect(warped_before, model, conf_threshold=0.5)
    detections_after = yolo_detect(warped_after, model, conf_threshold=0.5)
    
    # สำหรับ Debug: วาด bounding box บนภาพ
    for det in detections_before:
        x, y, w, h = det['bbox']
        cv2.rectangle(warped_before, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(warped_before, det['label'], (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
    for det in detections_after:
        x, y, w, h = det['bbox']
        cv2.rectangle(warped_after, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(warped_after, det['label'], (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
    
    # สร้าง board state mapping detection ลงในช่องบนกระดาน
    state_before = create_board_state(detections_before, board_width, board_height)
    state_after = create_board_state(detections_after, board_width, board_height)
    
    print("Board state BEFORE:", state_before)
    print("Board state AFTER :", state_after)
    
    # เปรียบเทียบ board state เพื่อหาช่องที่มีการเปลี่ยนแปลง
    changed_from = None
    changed_to = None
    moved_piece = None
    
    all_squares = set(state_before.keys()).union(set(state_after.keys()))
    for square in all_squares:
        piece_before = state_before.get(square)
        piece_after = state_after.get(square)
        if piece_before != piece_after:
            # ถ้าในช่องนั้นมีหมากใน before แต่ใน after ไม่มี => candidate ช่องต้นทาง
            if piece_before is not None and piece_after is None:
                changed_from = square
                moved_piece = piece_before
            # ถ้าในช่องนั้นไม่มีหมากใน before แต่มีใน after => candidate ช่องปลายทาง
            elif piece_before is None and piece_after is not None:
                changed_to = square
    
    if changed_from and changed_to:
        print(f"{moved_piece} ถูกย้ายจาก {changed_from} ไป {changed_to}")
    else:
        print("ไม่พบการเปลี่ยนแปลงชัดเจนใน board state")
    
    # แสดงภาพสำหรับตรวจสอบ
    cv2.imshow("Warped Before", warped_before)
    cv2.imshow("Warped After", warped_after)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
