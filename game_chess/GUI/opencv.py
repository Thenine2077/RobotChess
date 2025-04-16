import cv2
import numpy as np

# กำหนดจำนวน inner corners ที่ใช้ค้นหาจากกระดาน (7x7 สำหรับกระดาน 8x8 ช่อง)
inner_board_size = (7, 7)  # (cols, rows)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# โหลดภาพ (ปรับชื่อไฟล์ตามที่คุณใช้งาน)
img = cv2.imread('C:\Chess\RobotChess\game_chess\GUI\black_Bishop-10.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# หาจุดมุม inner corners ของกระดาน
ret, corners = cv2.findChessboardCorners(gray, inner_board_size, None)

if ret:
    # ปรับความแม่นยำของมุม
    corners_sub = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    
    # ปรับรูปแบบให้เป็น array ขนาด (rows, cols, 2)
    inner = corners_sub.reshape(inner_board_size[1], inner_board_size[0], 2)
    
    # Extrapolate หา outer corners (จุดมุม 9x9)
    # คำนวณมุมบนซ้าย (top-left)
    tl = inner[0, 0] - (inner[0, 1] - inner[0, 0]) - (inner[1, 0] - inner[0, 0])
    # มุมบนขวา (top-right)
    tr = inner[0, 6] + (inner[0, 6] - inner[0, 5]) - (inner[1, 6] - inner[0, 6])
    # มุมล่างขวา (bottom-right)
    br = inner[6, 6] + (inner[6, 6] - inner[6, 5]) + (inner[6, 6] - inner[5, 6])
    # มุมล่างซ้าย (bottom-left)
    bl = inner[6, 0] - (inner[6, 1] - inner[6, 0]) + (inner[6, 0] - inner[5, 0])
    
    # กำหนดจุด 4 มุมของกระดาน (outer corners)
    pts_src = np.array([tl, tr, br, bl], dtype="float32")
    
    # กำหนดขนาด output ของกระดาน (ควรหารด้วย 8 ลงตัว เช่น 800x800)
    board_width = 800
    board_height = 800
    pts_dst = np.array([
        [0, 0],
        [board_width, 0],
        [board_width, board_height],
        [0, board_height]
    ], dtype="float32")
    
    # คำนวณ perspective transform matrix
    M = cv2.getPerspectiveTransform(pts_src, pts_dst)
    
    # Warp ภาพให้ได้มุมมอง top-down ของกระดาน
    warped = cv2.warpPerspective(img, M, (board_width, board_height))
    
    # คำนวณขนาดของแต่ละช่อง
    cell_width = board_width // 8
    cell_height = board_height // 8
    
    # กำหนดชื่อคอลัมน์และแถว (มาตรฐานหมากรุก: คอลัมน์ A-H, แถว 8-1)
    col_labels = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H']
    row_labels = [8, 7, 6, 5, 4, 3, 2, 1]
    
    # วนลูปแบ่งและแสดงข้อมูลของแต่ละช่อง (64 ช่อง)
    for i in range(8):
        for j in range(8):
            x_min = j * cell_width
            x_max = (j + 1) * cell_width
            y_min = i * cell_height
            y_max = (i + 1) * cell_height
            # วาดกรอบสี่เหลี่ยมของช่อง
            cv2.rectangle(warped, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)
            label = f"{col_labels[j]}{row_labels[i]}"
            # ใส่ label ในแต่ละช่อง
            cv2.putText(warped, label, (x_min + 5, y_min + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            print(f"{label} -> xmin:{x_min}, xmax:{x_max}, ymin:{y_min}, ymax:{y_max}")
    
    # แสดงภาพผลลัพธ์ที่ถูก warp แล้วพร้อม grid 8x8
    cv2.imshow("Warped Chessboard", warped)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("ไม่พบกระดานหมากรุกในภาพ")
