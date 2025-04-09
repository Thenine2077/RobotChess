import cv2
import numpy as np
from ultralytics import YOLO

# ===============================
# ฟังก์ชันช่วย: เรียงลำดับจุดให้เป็น [top-left, top-right, bottom-right, bottom-left]
# ===============================
def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    # ผลรวมพิกัด: น้อยสุด = top-left, มากสุด = bottom-right
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    
    # ผลต่างพิกัด: น้อยสุด = top-right, มากสุด = bottom-left
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect

# ===============================
# หา contour ของกระดานด้วย Canny edge detection และ approxPolyDP
# ===============================
def find_board_contour(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # เรียง contour ตามพื้นที่จากมากไปน้อย
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    
    board_contour = None
    for cnt in contours:
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        if len(approx) == 4:
            board_contour = approx
            break
    return board_contour

# ===============================
# Warp ภาพกระดานให้เป็นมุมมอง Top-Down (800×800)
# ===============================
def warp_board(image, board_contour, board_width=800, board_height=800):
    pts = board_contour.reshape(4, 2)
    rect = order_points(pts)
    pts_dst = np.array([
        [0, 0],
        [board_width - 1, 0],
        [board_width - 1, board_height - 1],
        [0, board_height - 1]
    ], dtype="float32")
    
    M = cv2.getPerspectiveTransform(rect, pts_dst)
    warped = cv2.warpPerspective(image, M, (board_width, board_height))
    return warped

# ===============================
# ตรวจจับหมากด้วย YOLO บนภาพ warped แล้วสร้าง board state
# ===============================
def get_board_state_from_warped(warped, model):
    results = model(warped, conf=0.5)
    boxes = results[0].boxes
    board_state = {}  # ตัวอย่าง: {'A8': 'black_knight', 'B7': 'white_bishop', ...}
    
    board_w = warped.shape[1]
    board_h = warped.shape[0]
    cell_w = board_w // 8
    cell_h = board_h // 8
    
    # เตรียม dictionary สำหรับทุกช่อง (top-left = A8, bottom-right = H1)
    cols = ['A','B','C','D','E','F','G','H']
    rows = [8,7,6,5,4,3,2,1]
    for i in range(8):
        for j in range(8):
            square = f"{cols[j]}{rows[i]}"
            board_state[square] = None  # กำหนดเริ่มต้นเป็นว่าง
    
    # ฟังก์ชันช่วย: แม็ปจุดกึ่งกลางไปยังช่อง
    def map_center(cx, cy):
        col_index = int(cx // cell_w)
        row_index = int(cy // cell_h)
        col_index = max(0, min(col_index, 7))
        row_index = max(0, min(row_index, 7))
        return f"{cols[col_index]}{rows[row_index]}"
    
    if boxes is not None:
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            class_id = int(box.cls[0])
            label = model.names[class_id]
            w_box = x2 - x1
            h_box = y2 - y1
            cx = x1 + w_box/2
            cy = y1 + h_box/2
            square = map_center(cx, cy)
            board_state[square] = label
    return board_state

# ===============================
# เปรียบเทียบ board state ก่อนและหลังเพื่อหาการเคลื่อนที่
# ===============================
def compare_board_states(state_before, state_after):
    origin = None
    destination = None
    moved_piece = None
    # ตรวจสอบทุกช่องใน board state
    for square in state_before.keys():
        piece_before = state_before[square]
        piece_after = state_after[square]
        if piece_before != piece_after:
            # ช่องที่เคยมีหมากแต่ในภายหลังว่าง => candidate ต้นทาง
            if piece_before is not None and (piece_after is None or piece_before != piece_after):
                origin = square
                moved_piece = piece_before
            # ช่องที่เคยว่างแต่ในภายหลังมีหมาก => candidate ปลายทาง
            if piece_after is not None and (piece_before is None or piece_before != piece_after):
                destination = square
    return origin, destination, moved_piece

# ===============================
# Main Function: ทำ Before-After และตรวจจับการเคลื่อนที่
# ===============================
def main():
    # ระบุ path ของภาพ before และ after
    img_path_before = "D:/Project RobotChess/RobotChess/game_chess/GUI/pawn2.jpg"
    img_path_after  = "D:/Project RobotChess/RobotChess/game_chess/GUI/pawn1.jpg"  # ปรับให้เป็นไฟล์ภาพ “หลัง”
    model_path = "D:/Project RobotChess/RobotChess/game_chess/runs/detect/train3/weights/best.pt"

    img_before = cv2.imread(img_path_before)
    img_after  = cv2.imread(img_path_after)
    
    if img_before is None or img_after is None:
        print("ไม่สามารถโหลดภาพ before/after ได้ กรุณาตรวจสอบ path")
        return
    
    # หา contour ของกระดานในภาพ before (สมมติว่าทั้งสองภาพมุมและลักษณะคล้ายกัน)
    board_contour = find_board_contour(img_before)
    if board_contour is None:
        print("ไม่พบ contour ของกระดานในภาพ before")
        return
    
    # Warp ภาพ both before และ after ให้เป็น top-down view (800×800)
    warped_before = warp_board(img_before, board_contour, 800, 800)
    warped_after  = warp_board(img_after, board_contour, 800, 800)
    
    # โหลด YOLO model
    model = YOLO(model_path)
    
    # สร้าง board state จากภาพ warped ก่อนและหลัง
    state_before = get_board_state_from_warped(warped_before, model)
    state_after = get_board_state_from_warped(warped_after, model)
    
    # แสดง board state (สำหรับ debug)
    print("Board State BEFORE:")
    for sq in sorted(state_before.keys()):
        print(f"{sq}: {state_before[sq]}")
    
    print("\nBoard State AFTER:")
    for sq in sorted(state_after.keys()):
        print(f"{sq}: {state_after[sq]}")
    
    # เปรียบเทียบ board state เพื่อหาการเปลี่ยนแปลง (การเคลื่อนที่)
    origin, destination, moved_piece = compare_board_states(state_before, state_after)
    if origin and destination and moved_piece:
        print(f"\n{moved_piece} ถูกขยับจาก {origin} ไป {destination}")
    else:
        print("\nไม่พบการเปลี่ยนแปลงที่ชัดเจนใน board state")
    
    # (Optional) วาดกริด 8x8 ลงบนภาพ warped_after เพื่อดูตำแหน่งช่อง
    cell_w = warped_after.shape[1] // 8
    cell_h = warped_after.shape[0] // 8
    cols = ['A','B','C','D','E','F','G','H']
    rows = [8,7,6,5,4,3,2,1]
    for i in range(8):
        for j in range(8):
            x_min = j * cell_w
            y_min = i * cell_h
            x_max = (j+1)*cell_w
            y_max = (i+1)*cell_h
            cv2.rectangle(warped_after, (x_min, y_min), (x_max, y_max), (0,0,255), 2)
            square_label = f"{cols[j]}{rows[i]}"
            cv2.putText(warped_after, square_label, (x_min+5, y_min+20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2)
    
    cv2.imshow("Warped Before", warped_before)
    cv2.imshow("Warped After", warped_after)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
