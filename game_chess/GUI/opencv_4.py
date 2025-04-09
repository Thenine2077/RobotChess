import cv2
import numpy as np
from ultralytics import YOLO

def order_points(pts):
    """
    เรียงลำดับจุดให้เป็น [top-left, top-right, bottom-right, bottom-left]
    """
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]  # top-left
    rect[2] = pts[np.argmax(s)]  # bottom-right
    
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]  # top-right
    rect[3] = pts[np.argmax(diff)]  # bottom-left
    
    return rect

def find_board_contour(image):
    """
    หา contour ที่น่าจะเป็นกระดาน โดยใช้ Canny edge + approxPolyDP
    คืนค่า contour 4 จุด (quadrilateral) ที่มีพื้นที่ใหญ่สุด
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    
    board_contour = None
    for cnt in contours:
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        if len(approx) == 4:
            board_contour = approx
            break
    return board_contour, edges

def warp_board(image, board_contour, board_width=800, board_height=800):
    """
    Warp ภาพกระดานให้เป็น top-down view ขนาด board_width x board_height
    โดยมุม [top-left, top-right, bottom-right, bottom-left] → [(0,0), (w,0), (w,h), (0,h)]
    """
    pts = board_contour.reshape(4, 2)
    rect = order_points(pts)  # [tl, tr, br, bl]
    pts_dst = np.array([
        [0, 0],
        [board_width - 1, 0],
        [board_width - 1, board_height - 1],
        [0, board_height - 1]
    ], dtype="float32")
    
    M = cv2.getPerspectiveTransform(rect, pts_dst)
    warped = cv2.warpPerspective(image, M, (board_width, board_height))
    return warped

def map_center_to_square(cx, cy, board_w, board_h):
    """
    แม็ปจุดกึ่งกลาง (cx, cy) ไปยังชื่อช่อง (A1..H8)
    ปัจจุบันสมมติว่า top-left = A8, bottom-right = H1 (เพราะเรา warp ให้ [0,0] = top-left)
    """
    cell_w = board_w // 8
    cell_h = board_h // 8
    
    # col_index นับจากซ้ายไปขวา
    col_index = int(cx // cell_w)
    # row_index นับจากบนลงล่าง
    row_index = int(cy // cell_h)
    
    # clamp ไม่ให้ออกนอก 0..7
    col_index = max(0, min(col_index, 7))
    row_index = max(0, min(row_index, 7))
    
    # ถ้าต้องการให้ top-left เป็น A8 => col_labels = A..H, row_labels = 8..1
    col_labels = ['A','B','C','D','E','F','G','H']
    row_labels = [8,7,6,5,4,3,2,1]
    
    # col = col_labels[col_index]
    # row = row_labels[row_index]
    # return f"{col}{row}"
    
    # หรือถ้าอยากได้ bottom-left = A1 ให้สลับวิธี warp หรือสลับ row_index
    # (ตัวอย่างด้านบนเป็น top-left = A8)
    
    return f"{col_labels[col_index]}{row_labels[row_index]}"

def main():
    # =========================================
    # 1) โหลดภาพ และหา contour ของกระดาน
    # =========================================
    img_path = "D:/Project RobotChess/RobotChess/game_chess/GUI/whiteBishop1.jpg"  # ภาพที่มีหมากขาวบังเส้นบางส่วน
    image = cv2.imread(img_path)
    if image is None:
        print("ไม่พบไฟล์ภาพ:", img_path)
        return
    
    board_contour, edges = find_board_contour(image)
    if board_contour is None:
        print("ไม่พบ contour กระดาน")
        return
    
    # =========================================
    # 2) Warp ภาพกระดานให้เป็น top-down
    # =========================================
    warped = warp_board(image, board_contour, 800, 800)
    
    # =========================================
    # 3) ตรวจจับหมากในภาพ warped (ด้วย YOLO)
    # =========================================
    model_path = "D:/Project RobotChess/RobotChess/game_chess/runs/detect/train3/weights/best.pt"  # ปรับ path ตามโมเดลของคุณ
    model = YOLO(model_path)
    results = model(warped, conf=0.5)
    boxes = results[0].boxes  # list ของ bounding boxes
    
    # =========================================
    # 4) แม็ปหมากลงในช่อง A1..H8
    # =========================================
    board_w = warped.shape[1]
    board_h = warped.shape[0]
    
    if boxes is not None:
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            class_id = int(box.cls[0])
            label = model.names[class_id]
            w_box = x2 - x1
            h_box = y2 - y1
            cx = x1 + w_box/2
            cy = y1 + h_box/2
            
            # หา “ช่อง” ที่ center ตกอยู่
            square = map_center_to_square(cx, cy, board_w, board_h)
            print(f"พบ {label} อยู่ที่ช่อง {square}")
            
            # วาด bounding box + ชื่อช่องลงบน warped
            cv2.rectangle(warped, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
            cv2.putText(warped, f"{label} ({square})", (int(x1), int(y1)-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
    
    # =========================================
    # 5) วาดเส้นแบ่ง 8x8 ลงบน warped เพื่อดูตำแหน่ง
    # =========================================
    cell_w = board_w // 8
    cell_h = board_h // 8
    for i in range(8):
        for j in range(8):
            x_min = j * cell_w
            y_min = i * cell_h
            x_max = (j+1)*cell_w
            y_max = (i+1)*cell_h
            cv2.rectangle(warped, (x_min, y_min), (x_max, y_max), (0,0,255), 2)
    
    # แสดงผล
    cv2.imshow("Detected Board Contour", image)
    cv2.imshow("Edges", edges)
    cv2.imshow("Warped Board", warped)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
