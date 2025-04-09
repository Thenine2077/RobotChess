import cv2
import numpy as np
from ultralytics import YOLO

# ===============================
# กำหนด Path ของไฟล์ภาพและโมเดล
# ===============================
img_path_before = "D:/Project RobotChess/RobotChess/game_chess/GUI/whiteBishop1.jpg"
img_path_after  = "D:/Project RobotChess/RobotChess/game_chess/GUI/whiteBishop2.jpg"  # ปรับให้เป็นไฟล์ภาพ “หลัง”
model_path = "D:/Project RobotChess/RobotChess/game_chess/runs/detect/train3/weights/best.pt"

# ===============================
# ฟังก์ชันสำหรับ Warp กระดานให้เป็น Top-Down
# ===============================
def warp_chessboard(image, board_width=800, board_height=800):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    inner_board_size = (7, 7)  # inner corners สำหรับกระดาน 8x8
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    ret, corners = cv2.findChessboardCorners(gray, inner_board_size, None)
    if not ret:
        print("ไม่พบกระดานหมากรุกในภาพ")
        return None
    
    corners_sub = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    inner = corners_sub.reshape(inner_board_size[1], inner_board_size[0], 2)
    
    # Extrapolate หา outer corners
    # กำหนดลำดับให้ได้: bottom-left, bottom-right, top-right, top-left
    bl = inner[6, 0] - (inner[6, 1] - inner[6, 0]) + (inner[6, 0] - inner[5, 0])  # bottom-left
    br = inner[6, 6] + (inner[6, 6] - inner[6, 5]) + (inner[6, 6] - inner[5, 6])  # bottom-right
    tr = inner[0, 6] + (inner[0, 6] - inner[0, 5]) - (inner[1, 6] - inner[0, 6])  # top-right
    tl = inner[0, 0] - (inner[0, 1] - inner[0, 0]) - (inner[1, 0] - inner[0, 0])  # top-left

    pts_src = np.array([bl, br, tr, tl], dtype="float32")
    # กำหนด pts_dst ให้ bottom-left อยู่ที่ (0, board_height) และ top-left ที่ (0,0)
    pts_dst = np.array([
        [0, board_height],             # bottom-left
        [board_width, board_height],   # bottom-right
        [board_width, 0],              # top-right
        [0, 0]                         # top-left
    ], dtype="float32")
    
    M = cv2.getPerspectiveTransform(pts_src, pts_dst)
    warped = cv2.warpPerspective(image, M, (board_width, board_height))
    return warped

# ===============================
# ฟังก์ชันสำหรับสร้าง Board State จากภาพ warped ด้วย YOLO
# ===============================
def get_board_state_from_warped(warped, model):
    results = model(warped, conf=0.5)
    boxes = results[0].boxes
    board_state = {}
    board_w = warped.shape[1]
    board_h = warped.shape[0]
    cell_w = board_w // 8
    cell_h = board_h // 8
    
    # เตรียม dictionary สำหรับทุกช่อง (A1 ถึง H8)
    cols = ['A','B','C','D','E','F','G','H']
    rows = [8,7,6,5,4,3,2,1]
    for i in range(8):
        for j in range(8):
            square = f"{cols[j]}{rows[i]}"
            board_state[square] = None
    
    # ฟังก์ชันช่วยแม็ปจุดกึ่งกลางไปยังช่อง
    def map_center(cx, cy):
        col_index = int(cx // cell_w)
        row_index = int(cy // cell_h)
        col_index = max(0, min(col_index, 7))
        # ในภาพ warped แถวบนสุดคือ 8, ล่างสุดคือ 1
        row_number = 8 - row_index
        return f"{cols[col_index]}{row_number}"
    
    if boxes is not None:
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            class_id = int(box.cls[0])
            label = model.names[class_id]
            w_box = x2 - x1
            h_box = y2 - y1
            cx = x1 + w_box / 2
            cy = y1 + h_box / 2
            square = map_center(cx, cy)
            board_state[square] = label
    return board_state

# ===============================
# ฟังก์ชันสำหรับเปรียบเทียบ Board State ก่อนและหลัง
# ===============================
def compare_board_states(state_before, state_after):
    origin = None
    destination = None
    moved_piece = None
    
    # เปรียบเทียบทุกช่องใน board state (A1 ถึง H8)
    for square in state_before.keys():
        piece_before = state_before[square]
        piece_after = state_after[square]
        if piece_before != piece_after:
            # ช่องที่มีหมากในก่อนแต่ไม่มีในหลัง (หรือเปลี่ยน) => candidate ต้นทาง
            if piece_before is not None and (piece_after is None or piece_before != piece_after):
                origin = square
                moved_piece = piece_before
            # ช่องที่ไม่มีหมากในก่อนแต่มีในหลัง (หรือเปลี่ยน) => candidate ปลายทาง
            if piece_after is not None and (piece_before is None or piece_before != piece_after):
                destination = square
    return origin, destination, moved_piece

# ===============================
# Main Function
# ===============================
def main():
    # โหลดภาพก่อนและหลัง
    img_before = cv2.imread(img_path_before)
    img_after = cv2.imread(img_path_after)
    
    if img_before is None or img_after is None:
        print("ไม่สามารถโหลดภาพ before/after ได้ กรุณาตรวจสอบ path")
        return

    # Warp ภาพให้เป็น top-down view
    warped_before = warp_chessboard(img_before)
    warped_after = warp_chessboard(img_after)
    
    if warped_before is None or warped_after is None:
        print("เกิดปัญหาในการ warp ภาพ")
        return
    
    # โหลด YOLO model จากไฟล์ best.pt
    model = YOLO(model_path)
    
    # สร้าง board state จากภาพ warped ด้วย YOLO
    board_state_before = get_board_state_from_warped(warped_before, model)
    board_state_after = get_board_state_from_warped(warped_after, model)
    
    print("Board State BEFORE:")
    for sq in sorted(board_state_before.keys()):
        print(f"{sq}: {board_state_before[sq]}")
        
    print("\nBoard State AFTER:")
    for sq in sorted(board_state_after.keys()):
        print(f"{sq}: {board_state_after[sq]}")
    
    # เปรียบเทียบ board state เพื่อหาการเปลี่ยนแปลง
    origin, destination, moved_piece = compare_board_states(board_state_before, board_state_after)
    
    if origin and destination:
        print(f"\n{moved_piece} ถูกขยับจาก {origin} ไป {destination}")
    else:
        print("\nไม่พบการเปลี่ยนแปลงที่ชัดเจนใน board state")
    
    # สำหรับ Debug: แสดงภาพ warped หลังจากวาด grid ช่อง
    # วาดเส้นกริด 8x8 และ label ช่องลงบนภาพ warped_after
    cell_w = warped_after.shape[1] // 8
    cell_h = warped_after.shape[0] // 8
    cols = ['A','B','C','D','E','F','G','H']
    rows = [8,7,6,5,4,3,2,1]
    for i in range(8):
        for j in range(8):
            x_min = j * cell_w
            x_max = (j + 1) * cell_w
            y_min = i * cell_h
            y_max = (i + 1) * cell_h
            cv2.rectangle(warped_after, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)
            square_label = f"{cols[j]}{rows[i]}"
            cv2.putText(warped_after, square_label, (x_min + 5, y_min + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    
    cv2.imshow("Warped Before", warped_before)
    cv2.imshow("Warped After", warped_after)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
