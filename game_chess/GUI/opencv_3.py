import cv2
import numpy as np

def order_points(pts):
    """
    เรียงลำดับจุดให้เป็น [top-left, top-right, bottom-right, bottom-left]
    """
    rect = np.zeros((4, 2), dtype="float32")
    # ผลรวมของพิกัด: ค่าน้อยสุดคือ top-left, มากสุดคือ bottom-right
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    
    # ผลต่างของพิกัด: ค่าน้อยสุดคือ top-right, มากสุดคือ bottom-left
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    
    return rect

def find_board_contour(image):
    """
    หา contour ที่น่าจะเป็นกระดานหมากรุก โดยใช้ Canny edge detection และ approxPolyDP
    คืนค่า contour ที่มี 4 จุด (quadrilateral) ที่มีพื้นที่มากที่สุด
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Gaussian Blur เพื่อลด noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # ใช้ Canny edge detection
    edges = cv2.Canny(blurred, 50, 150)
    
    # หา contour
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # เรียง contour ตามพื้นที่ (มากไปน้อย)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    
    board_contour = None
    for cnt in contours:
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        # ถ้า contour มี 4 จุด ให้ถือว่าเป็น candidate
        if len(approx) == 4:
            board_contour = approx
            break
            
    return board_contour, edges

def warp_board(image, board_contour, board_width=800, board_height=800):
    """
    ใช้ perspective transform เพื่อ warp ภาพกระดานให้เป็น top-down view
    """
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

def main():
    # ระบุ path ของภาพกระดาน (ภาพที่มีหมากอาจบังส่วนเส้นตารางได้)
    img_path = "D:/Project RobotChess/RobotChess/game_chess/GUI/whiteBishop1.jpg"
    image = cv2.imread(img_path)
    if image is None:
        print("ไม่พบไฟล์ภาพ:", img_path)
        return
    
    # หา contour ของกระดาน
    board_contour, edges = find_board_contour(image)
    if board_contour is None:
        print("ไม่พบ contour ที่เป็นกระดาน")
        return
    
    # สำหรับ Debug: แสดงภาพ edges และ contour ที่ตรวจจับได้
    contour_image = image.copy()
    cv2.drawContours(contour_image, [board_contour], -1, (0, 255, 0), 3)
    cv2.imshow("Detected Board Contour", contour_image)
    cv2.imshow("Edges", edges)
    cv2.waitKey(0)
    
    # Warp ภาพกระดาน
    warped = warp_board(image, board_contour, board_width=800, board_height=800)
    cv2.imshow("Warped Board", warped)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
