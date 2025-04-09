import pygame
import chess
import os
import time
import sys

# โหลดโมดูล YOLO จาก ultralytics (ใช้สำหรับ auto-detect move) 
try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None  # ถ้าไม่มี YOLO จะไม่สามารถใช้ auto-detect ได้

# โหลด find_best_move จาก game_minimax หากมี
try:
    from game_minimax import find_best_move
except ImportError:
    def find_best_move(board, depth=3):
        for move in board.legal_moves:
            return move
    print("Using mock find_best_move")

pygame.init()
pygame.key.set_repeat(500, 50)

# --------------------------------------------------------------------------------
# CONFIG: ค่าต่างๆ สำหรับ UI
# --------------------------------------------------------------------------------
SCREEN_WIDTH = 900
SCREEN_HEIGHT = 600

BOARD_SIZE = 600
SQUARE_SIZE = BOARD_SIZE // 8

# กระดานอยู่ด้านซ้าย
BOARD_X = 0
BOARD_Y = 0

# Panel อยู่ด้านขวา
PANEL_X = BOARD_X + BOARD_SIZE + 20
PANEL_Y = 20
PANEL_WIDTH = SCREEN_WIDTH - (BOARD_X + BOARD_SIZE + 40)
PANEL_HEIGHT = 500

# สีต่างๆ
BACKGROUND_COLOR = (255, 255, 255)
PANEL_BG_COLOR = (240, 240, 240)
BOARD_WHITE = (240, 217, 181)
BOARD_BROWN = (181, 136, 99)
TEXT_COLOR = (0, 0, 0)
STATUS_COLOR = (255, 0, 0)

# ฟอนต์ (สามารถเปลี่ยนเป็นฟอนต์ที่รองรับภาษาไทยได้)
FONT = pygame.font.Font(None, 24)

# โฟลเดอร์สำหรับภาพหมาก (ปรับให้ตรงกับระบบของคุณ)
ASSET_FOLDER = r"D:\Project RobotChess\RobotChess\game_chess\GUI\assets"
PIECES = {
    "p": "b_p.png",
    "P": "w_p.png",
    "r": "b_r.png",
    "R": "w_r.png",
    "n": "b_n.png",
    "N": "w_n.png",
    "b": "b_b.png",
    "B": "w_b.png",
    "q": "b_q.png",
    "Q": "w_q.png",
    "k": "b_k.png",
    "K": "w_k.png",
}

# --------------------------------------------------------------------------------
# ฟังก์ชันสำหรับโหลดรูปหมาก
# --------------------------------------------------------------------------------
def load_pieces():
    piece_images = {}
    for piece, filename in PIECES.items():
        path = os.path.join(ASSET_FOLDER, filename)
        if os.path.exists(path):
            img = pygame.image.load(path)
            img = pygame.transform.scale(img, (SQUARE_SIZE, SQUARE_SIZE))
            piece_images[piece] = img
        else:
            print("ไม่พบไฟล์:", path)
    return piece_images

# --------------------------------------------------------------------------------
# ฟังก์ชันสำหรับวาดกระดานหมากรุก
# --------------------------------------------------------------------------------
def draw_board(screen, board, piece_images):
    for row in range(8):
        for col in range(8):
            color = BOARD_WHITE if (row + col) % 2 == 0 else BOARD_BROWN
            rect = pygame.Rect(BOARD_X + col * SQUARE_SIZE,
                               BOARD_Y + row * SQUARE_SIZE,
                               SQUARE_SIZE, SQUARE_SIZE)
            pygame.draw.rect(screen, color, rect)
            piece = board.piece_at(chess.square(col, 7 - row))
            if piece:
                screen.blit(piece_images[piece.symbol()], (rect.x, rect.y))
    # วาดตัวเลข (1-8) ซ้ายกระดาน
    for i in range(8):
        num_text = FONT.render(str(8 - i), True, TEXT_COLOR)
        screen.blit(num_text, (BOARD_X - 20, BOARD_Y + i * SQUARE_SIZE + SQUARE_SIZE // 3))
    # วาดตัวอักษร (a-h) ด้านล่าง
    for i in range(8):
        letter_text = FONT.render(chr(ord('a') + i), True, TEXT_COLOR)
        screen.blit(letter_text, (BOARD_X + i * SQUARE_SIZE + SQUARE_SIZE // 2 - 5,
                                  BOARD_Y + BOARD_SIZE + 5))

# --------------------------------------------------------------------------------
# ฟังก์ชัน format_time สำหรับจัดรูปแบบเวลา mm:ss
# --------------------------------------------------------------------------------
def format_time(seconds):
    m = int(seconds // 60)
    s = int(seconds % 60)
    return f"{m:02d}:{s:02d}"

# --------------------------------------------------------------------------------
# ฟังก์ชันสำหรับตรวจจับ move ด้วย Vision (Auto Detect) 
# --------------------------------------------------------------------------------
def detect_move_vision():
    """
    ตรวจจับ move จากภาพ before/after ด้วย OpenCV และ YOLO
    คืนค่าเป็น move UCI (เช่น "d3d2") หรือ None หากตรวจจับไม่พบ
    """
    if YOLO is None:
        print("ไม่พบโมดูล ultralytics YOLO สำหรับ Auto Detect")
        return None

    import cv2
    import numpy as np

    def order_points(pts):
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        return rect

    def find_board_contour(image):
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
        return board_contour

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

    def get_board_state_from_warped(warped, model):
        results = model(warped, conf=0.5)
        boxes = results[0].boxes
        board_state = {}
        board_w = warped.shape[1]
        board_h = warped.shape[0]
        cell_w = board_w // 8
        cell_h = board_h // 8
        cols = ['A','B','C','D','E','F','G','H']
        rows = [8,7,6,5,4,3,2,1]
        for i in range(8):
            for j in range(8):
                square = f"{cols[j]}{rows[i]}"
                board_state[square] = None
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
                cx = x1 + w_box / 2
                cy = y1 + h_box / 2
                square = map_center(cx, cy)
                board_state[square] = label
        return board_state

    def compare_board_states(state_before, state_after):
        origin = None
        destination = None
        moved_piece = None
        for square in state_before.keys():
            piece_before = state_before[square]
            piece_after = state_after[square]
            if piece_before != piece_after:
                if piece_before is not None and (piece_after is None or piece_before != piece_after):
                    origin = square
                    moved_piece = piece_before
                if piece_after is not None and (piece_before is None or piece_before != piece_after):
                    destination = square
        return origin, destination, moved_piece

    # ระบุ path ของภาพ before/after (ปรับให้ตรงกับไฟล์ของคุณ)
    img_path_before = "D:/Project RobotChess/RobotChess/game_chess/GUI/pawn1.jpg"
    img_path_after  = "D:/Project RobotChess/RobotChess/game_chess/GUI/pawn2.jpg"
    import cv2
    img_before = cv2.imread(img_path_before)
    img_after  = cv2.imread(img_path_after)
    
    if img_before is None or img_after is None:
        print("ไม่สามารถโหลดภาพ before/after ได้ กรุณาตรวจสอบ path")
        return None
    
    board_contour = find_board_contour(img_before)
    if board_contour is None:
        print("ไม่พบ contour ของกระดานในภาพ before")
        return None
    
    warped_before = warp_board(img_before, board_contour, 800, 800)
    warped_after  = warp_board(img_after, board_contour, 800, 800)
    
    # ระบุ path ของโมเดล YOLO (ปรับให้ตรงกับไฟล์ model ของคุณ)
    model_path = "D:/Project RobotChess/RobotChess/game_chess/runs/detect/train3/weights/best.pt"
    try:
        model = YOLO(model_path)
    except Exception as e:
        print("โหลด YOLO model ไม่ได้:", e)
        return None
    
    state_before = get_board_state_from_warped(warped_before, model)
    state_after = get_board_state_from_warped(warped_after, model)
    origin, destination, moved_piece = compare_board_states(state_before, state_after)
    if origin and destination and moved_piece:
        move_str = origin.lower() + destination.lower()
        print(f"{moved_piece} ถูกขยับจาก {origin} ไป {destination} (Move: {move_str})")
        return move_str
    else:
        print("ไม่พบการเปลี่ยนแปลงที่ชัดเจนใน board state")
        return None

# --------------------------------------------------------------------------------
# MAIN GAME LOOP: Dual Move Input (Manual และ Auto Detect)
# --------------------------------------------------------------------------------
def play_chess():
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Robot Chess with Dual Move Input")
    clock = pygame.time.Clock()

    board = chess.Board()
    piece_images = load_pieces()

    # เวลาของแต่ละฝั่ง (ตัวอย่าง 5 นาที)
    white_time_left = 5 * 60
    black_time_left = 5 * 60

    # กำหนด state ของเกม:
    # "WHITE_TURN"    - ผู้เล่น (ขาว) กรอก move ด้วยตัวเอง (หรือ Auto Detect)
    # "BLACK_TURN"    - AI (ดำ) คิดประมาณ 3 วินาที
    # "POST_AI_DELAY" - หน่วงเวลาหลัง AI เดิน (3 วิ)
    # "GAME_OVER"     - เกมจบ
    state = "WHITE_TURN"
    ai_think_time = 3000
    post_ai_delay = 3000
    state_start_ticks = pygame.time.get_ticks()

    # ตัวแปรสำหรับช่อง input move
    move_input = ""
    move_made = False  # เช็คว่าผู้เล่นได้เดินหมากแล้วหรือยัง

    running = True
    while running:
        dt = clock.tick(30)
        dt_sec = dt / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # รับ event ใน state "WHITE_TURN"
            if state == "WHITE_TURN":
                if event.type == pygame.TEXTINPUT:
                    move_input += event.text
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_BACKSPACE:
                        move_input = move_input[:-1]
                    elif event.key == pygame.K_RETURN:
                        try:
                            uci_move = chess.Move.from_uci(move_input.strip())
                            if uci_move in board.legal_moves:
                                board.push(uci_move)
                                move_made = True
                            else:
                                print("❌ Invalid move:", move_input)
                        except Exception as e:
                            print("❌ Invalid move format:", move_input)
                        move_input = ""
                if event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = event.pos
                    # ปุ่ม End Turn (เฉพาะเมื่อ move ถูกทำแล้ว)
                    end_turn_rect = pygame.Rect(PANEL_X + 20, PANEL_Y + 220, 80, 30)
                    if end_turn_rect.collidepoint(mx, my) and move_made:
                        state = "BLACK_TURN"
                        state_start_ticks = pygame.time.get_ticks()
                        move_made = False
                    # ปุ่ม Auto Detect
                    auto_detect_rect = pygame.Rect(PANEL_X + 20, PANEL_Y + 260, 100, 30)
                    if auto_detect_rect.collidepoint(mx, my):
                        detected_move = detect_move_vision()
                        if detected_move is not None:
                            move_input = detected_move
                            print("Auto Detected Move:", move_input)
                    # ปุ่ม Exit
                    exit_rect = pygame.Rect(PANEL_X, SCREEN_HEIGHT - 60, PANEL_WIDTH, 40)
                    if exit_rect.collidepoint(mx, my):
                        running = False
            else:
                if event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = event.pos
                    exit_rect = pygame.Rect(PANEL_X, SCREEN_HEIGHT - 60, PANEL_WIDTH, 40)
                    if exit_rect.collidepoint(mx, my):
                        running = False

        # ลดเวลา
        if state == "WHITE_TURN":
            white_time_left -= dt_sec
            if white_time_left <= 0:
                white_time_left = 0
                print("White out of time!")
                state = "GAME_OVER"
        elif state == "BLACK_TURN":
            black_time_left -= dt_sec
            if black_time_left <= 0:
                black_time_left = 0
                print("Black out of time!")
                state = "GAME_OVER"
            elapsed = pygame.time.get_ticks() - state_start_ticks
            if elapsed >= ai_think_time:
                if not board.is_game_over():
                    ai_move = find_best_move(board, depth=3)
                    print(f"AI Move: {ai_move}")
                    board.push(ai_move)
                state = "POST_AI_DELAY"
                state_start_ticks = pygame.time.get_ticks()
        elif state == "POST_AI_DELAY":
            elapsed = pygame.time.get_ticks() - state_start_ticks
            if elapsed >= post_ai_delay:
                state = "WHITE_TURN"
                state_start_ticks = pygame.time.get_ticks()

        # กำหนดข้อความ Status
        if state == "WHITE_TURN":
            status_message = "Your turn: Awaiting move"
        elif state == "BLACK_TURN":
            remain_ai = max(0, int((ai_think_time - (pygame.time.get_ticks() - state_start_ticks)) / 1000))
            status_message = f"AI thinking: {remain_ai} sec"
        elif state == "POST_AI_DELAY":
            delay_remain = max(0, int((post_ai_delay - (pygame.time.get_ticks() - state_start_ticks)) / 1000))
            status_message = f"Wait for Robot move: {delay_remain} sec"
        elif state == "GAME_OVER":
            status_message = "Game Over!"
        else:
            status_message = ""

        # ------ DRAWING ------
        screen.fill(BACKGROUND_COLOR)
        draw_board(screen, board, piece_images)

        # วาด Panel
        panel_rect = pygame.Rect(PANEL_X, PANEL_Y, PANEL_WIDTH, PANEL_HEIGHT)
        pygame.draw.rect(screen, PANEL_BG_COLOR, panel_rect)

        # ส่วนสำหรับ White: Label + Timer
        white_label = FONT.render("White", True, TEXT_COLOR)
        white_time_str = format_time(white_time_left)
        white_time_surf = FONT.render(white_time_str, True, TEXT_COLOR)
        screen.blit(white_label, (PANEL_X + 20, PANEL_Y + 20))
        screen.blit(white_time_surf, (PANEL_X + 20, PANEL_Y + 40))

        # Status Message
        status_surf = FONT.render(status_message, True, STATUS_COLOR)
        screen.blit(status_surf, (PANEL_X + 20, PANEL_Y + 65))

        # ช่อง Input สำหรับ Move
        move_label = FONT.render("Move:", True, TEXT_COLOR)
        screen.blit(move_label, (PANEL_X + 20, PANEL_Y + 90))
        input_box = pygame.Rect(PANEL_X + 20, PANEL_Y + 110, 120, 30)
        pygame.draw.rect(screen, (255, 255, 255), input_box)
        move_text = FONT.render(move_input, True, TEXT_COLOR)
        screen.blit(move_text, (input_box.x + 5, input_box.y + 5))

        # ปุ่ม Auto Detect
        auto_detect_rect = pygame.Rect(PANEL_X + 20, PANEL_Y + 260, 100, 30)
        pygame.draw.rect(screen, (100, 150, 250), auto_detect_rect)
        auto_detect_text = FONT.render("Auto Detect", True, (255,255,255))
        screen.blit(auto_detect_text, (
            auto_detect_rect.x + (auto_detect_rect.width - auto_detect_text.get_width()) // 2,
            auto_detect_rect.y + (auto_detect_rect.height - auto_detect_text.get_height()) // 2
        ))

        # ปุ่ม End Turn
        end_turn_rect = pygame.Rect(PANEL_X + 20, PANEL_Y + 220, 80, 30)
        pygame.draw.rect(screen, (200, 100, 100), end_turn_rect)
        end_turn_text = FONT.render("End Turn", True, (255,255,255))
        screen.blit(end_turn_text, (
            end_turn_rect.x + (end_turn_rect.width - end_turn_text.get_width()) // 2,
            end_turn_rect.y + (end_turn_rect.height - end_turn_text.get_height()) // 2
        ))

        # ส่วนสำหรับ Black: Label + Timer
        black_label = FONT.render("Black", True, TEXT_COLOR)
        black_time_str = format_time(black_time_left)
        black_time_surf = FONT.render(black_time_str, True, TEXT_COLOR)
        screen.blit(black_label, (PANEL_X + 20, PANEL_Y + 270))
        screen.blit(black_time_surf, (PANEL_X + 20, PANEL_Y + 290))

        # ปุ่ม Exit (ด้านล่าง Panel)
        exit_rect = pygame.Rect(PANEL_X, SCREEN_HEIGHT - 60, PANEL_WIDTH, 40)
        pygame.draw.rect(screen, (255, 0, 0), exit_rect)
        exit_text = FONT.render("Exit", True, (255,255,255))
        screen.blit(exit_text, (
            exit_rect.x + (exit_rect.width - exit_text.get_width()) // 2,
            exit_rect.y + (exit_rect.height - exit_text.get_height()) // 2
        ))

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    play_chess()
