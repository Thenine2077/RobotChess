import cv2
import numpy as np

# สมมติว่าเราได้ภาพกระดาน top-down ก่อนและหลัง (ขนาด 800x800)
warped_before = cv2.imread('D:/Project RobotChess/RobotChess/game_chess/GUI/black_Bishop-4.jpg')
warped_after  = cv2.imread('D:/Project RobotChess/RobotChess/game_chess/GUI/black_Bishop-10.jpg')
if warped_before.shape != warped_after.shape:
    print("Error: ขนาดภาพ before/after ไม่ตรงกัน!")
    exit()

board_width  = warped_before.shape[1]  # 800
board_height = warped_before.shape[0]  # 800
cell_width   = board_width // 8        # 100
cell_height  = board_height // 8       # 100

col_labels = ['A','B','C','D','E','F','G','H']
row_labels = [8,7,6,5,4,3,2,1]

# ฟังก์ชันช่วยตรวจจับความแตกต่างในแต่ละช่อง
def is_cell_different(crop_before, crop_after, pixel_diff_thresh=30, mean_diff_thresh=20):
    # แปลงเป็น Gray
    gray_before = cv2.cvtColor(crop_before, cv2.COLOR_BGR2GRAY)
    gray_after  = cv2.cvtColor(crop_after, cv2.COLOR_BGR2GRAY)
    
    # คำนวณ absolute difference
    diff = cv2.absdiff(gray_before, gray_after)
    
    # ทำ threshold เพื่อตัด noise
    _, diff_thresh = cv2.threshold(diff, pixel_diff_thresh, 255, cv2.THRESH_BINARY)
    
    # ลด noiseด้วย morphological operations
    kernel = np.ones((3,3), np.uint8)
    diff_clean = cv2.morphologyEx(diff_thresh, cv2.MORPH_OPEN, kernel)
    
    # คุณอาจแสดง diff_clean เพื่อดูภาพความแตกต่าง (debug)
    # cv2.imshow("Difference", diff_clean)
    # cv2.waitKey(0)
    
    mean_diff = diff_clean.mean()
    # ถ้า mean_diff เกินค่าที่กำหนด ถือว่ามีการเปลี่ยนแปลง
    return mean_diff > mean_diff_thresh

changed_cells = []

for i in range(8):
    for j in range(8):
        x_min = j * cell_width
        x_max = (j+1) * cell_width
        y_min = i * cell_height
        y_max = (i+1) * cell_height
        
        crop_before = warped_before[y_min:y_max, x_min:x_max]
        crop_after  = warped_after[y_min:y_max, x_min:x_max]
        
        if is_cell_different(crop_before, crop_after):
            cell_name = f"{col_labels[j]}{row_labels[i]}"
            changed_cells.append(cell_name)

print("ช่องที่เปลี่ยน:", changed_cells)

if len(changed_cells) == 2:
    from_cell, to_cell = changed_cells
    print(f"หมากรุกขยับจาก {from_cell} ไป {to_cell}")
elif len(changed_cells) > 2:
    print("ตรวจจับหลายช่องเปลี่ยน (อาจมีการกินหรือขยับหลายตัว)")
else:
    print("ไม่พบการเปลี่ยนแปลงที่ชัดเจน")
