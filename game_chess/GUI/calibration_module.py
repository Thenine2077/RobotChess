import cv2
import numpy as np

class RealTimeCamera:
    """
    Simple wrapper around OpenCV VideoCapture to grab frames from a camera.
    """
    def __init__(self, camera_index=0, width=800, height=800):
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera index {camera_index}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to read frame from camera")
        return frame

    def release(self):
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()


# เก็บพิกัดที่ผู้ใช้คลิก (TL, TR, BR, BL)
clicked_pts = []

def click_event(event, x, y, flags, param):
    """
    Mouse callback: records up to 4 clicks and draws them on the image.
    """
    global clicked_pts
    img = param['image']
    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_pts) < 4:
        clicked_pts.append((x, y))
        cv2.circle(img, (x, y), 5, (0, 255, 0), -1)
        cv2.imshow(param['window'], img)


def manual_calibration(cam):
    """
    เปิดหน้าต่างให้คลิก 4 มุมของกระดาน (TL, TR, BR, BL)
    คืนค่า 3x3 homography matrix H (dtype=float32) สำหรับ warp
    """
    global clicked_pts
    clicked_pts = []
    frame = cam.get_frame().copy()
    window = "Calibration - click 4 corners"
    cv2.namedWindow(window)
    cv2.imshow(window, frame)
    cv2.setMouseCallback(window, click_event, {'image': frame, 'window': window})

    print("Calibration mode: click 4 corners in order TL, TR, BR, BL")
    print("Press 'q' to abort.")

    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyWindow(window)
            print("Calibration aborted.")
            return None
        if len(clicked_pts) == 4:
            cv2.destroyWindow(window)
            break

    dst = np.array([[0,0],[799,0],[799,799],[0,799]], dtype=np.float32)
    src = np.array(clicked_pts, dtype=np.float32)
    H = cv2.getPerspectiveTransform(src, dst)
    print("Calibration complete. Homography matrix H:\n", H)
    return H


def warp_with_H(image, H, size=(800,800)):
    """
    Apply perspective warp to the image using homography H.
    """
    return cv2.warpPerspective(image, H, size)


if __name__ == "__main__":
    import sys

    try:
        cam = RealTimeCamera(camera_index=0, width=800, height=800)
    except Exception as e:
        print("Error opening camera:", e)
        sys.exit(1)

    H = manual_calibration(cam)
    if H is not None:
        print("Testing warp on a new frame...")
        frame = cam.get_frame()
        warped = warp_with_H(frame, H)
        cv2.imshow("Warp Test", warped)
        print("Press any key to exit test.")
        cv2.waitKey(0)
        cv2.destroyWindow("Warp Test")

    cam.release()
