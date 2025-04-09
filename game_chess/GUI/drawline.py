import cv2
import numpy as np

def draw_chessboard_grid(image, rows=8, cols=8, color=(0, 0, 255), thickness=2):
    """
    วาดเส้นแบ่ง grid บนภาพให้เป็นตาราง rows x cols (ค่าเริ่มต้น 8x8)
    
    Parameters:
        image: ภาพที่ต้องการวาดเส้นแบ่ง (numpy array)
        rows: จำนวนแถว (default 8)
        cols: จำนวนคอลัมน์ (default 8)
        color: สีของเส้น (default เป็นสีแดง (B,G,R) = (0,0,255))
        thickness: ความหนาของเส้น (default 2)
    
    Returns:
        grid_img: ภาพที่มีเส้นแบ่ง grid แล้ว
    """
    grid_img = image.copy()
    height, width = grid_img.shape[:2]
    cell_width = width // cols
    cell_height = height // rows

    # วาดเส้นแนวตั้ง
    for col in range(1, cols):
        x = col * cell_width
        cv2.line(grid_img, (x, 0), (x, height), color, thickness)
    
    # วาดเส้นแนวนอน
    for row in range(1, rows):
        y = row * cell_height
        cv2.line(grid_img, (0, y), (width, y), color, thickness)
    
    return grid_img

def main():
    # ระบุ path ของภาพ warped (top-down view)
    image_path = 'D:/Project RobotChess/RobotChess/game_chess/GUI/black_Bishop-4.jpg'
    image = cv2.imread(image_path)
    
    if image is None:
        print("ไม่พบไฟล์ภาพ กรุณาตรวจสอบ path:", image_path)
        return

    # วาด grid 8x8 ลงบนภาพ
    grid_image = draw_chessboard_grid(image, rows=8, cols=8, color=(0, 0, 255), thickness=2)
    
    # แสดงภาพผลลัพธ์
    cv2.imshow("Chessboard Grid", grid_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
