import pygame
import chess
import os
import time
import sys

# ถ้ามีโมดูล find_best_move ให้ import ให้ถูก path
try:
    from game_minimax import find_best_move
except ImportError:
    # หากไม่มี ทำ mock function สำหรับทดสอบ
    def find_best_move(board, depth=3):
        for move in board.legal_moves:
            return move
    print("Using mock find_best_move")

pygame.init()
pygame.key.set_repeat(500, 50)  # กำหนดให้ KEYDOWN repeat หากกดค้าง

# --------------------------------------------------------------------------------
# CONFIG: ค่าต่างๆ สำหรับ UI
# --------------------------------------------------------------------------------
SCREEN_WIDTH = 900
SCREEN_HEIGHT = 600

BOARD_SIZE = 600
SQUARE_SIZE = BOARD_SIZE // 8

# กำหนดตำแหน่งของกระดาน (ด้านซ้าย)
BOARD_X = 0
BOARD_Y = 0

# กำหนดตำแหน่งของ Panel (ด้านขวา)
PANEL_X = BOARD_X + BOARD_SIZE + 20
PANEL_Y = 20
PANEL_WIDTH = SCREEN_WIDTH - (BOARD_X + BOARD_SIZE + 40)
PANEL_HEIGHT = 500

# สี
BACKGROUND_COLOR = (255, 255, 255)
PANEL_BG_COLOR = (240, 240, 240)
BOARD_WHITE = (240, 217, 181)
BOARD_BROWN = (181, 136, 99)
TEXT_COLOR = (0, 0, 0)
STATUS_COLOR = (255, 0, 0)

# ฟอนต์ (สามารถใช้ฟอนต์ไทยได้โดยระบุไฟล์ฟอนต์ที่รองรับภาษาไทย)
FONT = pygame.font.Font(None, 24)

# โฟลเดอร์สำหรับภาพหมาก (ปรับ path ให้ตรงกับระบบของคุณ)
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
    # วาดตัวเลข (1-8) ซ้าย
    for i in range(8):
        num_text = FONT.render(str(8 - i), True, TEXT_COLOR)
        screen.blit(num_text, (BOARD_X - 20, BOARD_Y + i * SQUARE_SIZE + SQUARE_SIZE // 3))
    # วาดตัวอักษร (a-h) ด้านล่าง
    for i in range(8):
        letter_text = FONT.render(chr(ord('a') + i), True, TEXT_COLOR)
        screen.blit(letter_text, (BOARD_X + i * SQUARE_SIZE + SQUARE_SIZE // 2 - 5,
                                  BOARD_Y + BOARD_SIZE + 5))

def format_time(seconds):
    m = int(seconds // 60)
    s = int(seconds % 60)
    return f"{m:02d}:{s:02d}"

def play_chess():
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Robot Chess with Timer, Status & UI")
    clock = pygame.time.Clock()

    board = chess.Board()
    piece_images = load_pieces()

    # ตั้งเวลาให้แต่ละฝั่ง (ตัวอย่าง 5 นาทีต่อฝั่ง)
    white_time_left = 5 * 60
    black_time_left = 5 * 60

    # กำหนด state ของเกม:
    # "WHITE_TURN"    - ผู้เล่น (สีขาว) กำลังพิมพ์ move
    # "BLACK_TURN"    - รอบ AI (สีดำ) คิดประมาณ 3 วินาที
    # "POST_AI_DELAY" - หน่วงเวลาหลัง AI เดิน (3 วินาที) เพื่อรอการเคลื่อนไหวของหุ่นยนต์
    # "GAME_OVER"     - เกมจบ
    state = "WHITE_TURN"
    ai_think_time = 3000
    post_ai_delay = 3000
    state_start_ticks = pygame.time.get_ticks()

    move_input = ""    # ข้อความที่ผู้เล่นพิมพ์ในช่อง "Move"
    move_made = False  # เช็คว่าผู้เล่นได้เดินหมากแล้วหรือยัง

    running = True
    while running:
        dt = clock.tick(30)
        dt_sec = dt / 1000.0

        for event in pygame.event.get():
            # (สำหรับ debug: คุณสามารถเปิด print อีเวนต์ได้ หากต้องการดูเหตุการณ์)
            # print("DEBUG EVENT:", event)

            if event.type == pygame.QUIT:
                running = False

            # รับอีเวนต์สำหรับ state WHITE_TURN เท่านั้น
            if state == "WHITE_TURN":
                # ใช้ TEXTINPUT สำหรับรับข้อความทั่วไป
                if event.type == pygame.TEXTINPUT:
                    move_input += event.text

                # KEYDOWN สำหรับควบคุม BACKSPACE และ RETURN
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
                # จัดการคลิกเมาส์ (ปุ่ม End Turn และ Exit)
                if event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = event.pos
                    # ปุ่ม End Turn (เฉพาะเมื่อ move ถูกทำแล้ว)
                    end_turn_rect = pygame.Rect(PANEL_X + 20, PANEL_Y + 120, 80, 30)
                    if end_turn_rect.collidepoint(mx, my) and move_made:
                        state = "BLACK_TURN"
                        state_start_ticks = pygame.time.get_ticks()
                        move_made = False
                    # ปุ่ม Exit
                    exit_rect = pygame.Rect(PANEL_X, SCREEN_HEIGHT - 60, PANEL_WIDTH, 40)
                    if exit_rect.collidepoint(mx, my):
                        running = False

            else:
                # ใน state อื่น (BLACK_TURN, POST_AI_DELAY, GAME_OVER) รับแค่ปุ่ม Exit
                if event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = event.pos
                    exit_rect = pygame.Rect(PANEL_X, SCREEN_HEIGHT - 60, PANEL_WIDTH, 40)
                    if exit_rect.collidepoint(mx, my):
                        running = False

        # ลดเวลาตาม state
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
            # เช็คว่า AI คิดครบ 3 วินาทีหรือยัง
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
        # state "GAME_OVER" คงอยู่

        # กำหนดข้อความ Status ตาม state
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

        # Panel
        panel_rect = pygame.Rect(PANEL_X, PANEL_Y, PANEL_WIDTH, PANEL_HEIGHT)
        pygame.draw.rect(screen, PANEL_BG_COLOR, panel_rect)

        # ข้อมูลฝั่ง White: Label + Timer
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

        # ปุ่ม End Turn
        end_turn_rect = pygame.Rect(PANEL_X + 20, PANEL_Y + 150, 80, 30)
        pygame.draw.rect(screen, (200, 100, 100), end_turn_rect)
        end_turn_text = FONT.render("End Turn", True, (255, 255, 255))
        screen.blit(end_turn_text, (
            end_turn_rect.x + (end_turn_rect.width - end_turn_text.get_width()) // 2,
            end_turn_rect.y + (end_turn_rect.height - end_turn_text.get_height()) // 2
        ))

        # ข้อมูลฝั่ง Black: Label + Timer
        black_label = FONT.render("Black", True, TEXT_COLOR)
        black_time_str = format_time(black_time_left)
        black_time_surf = FONT.render(black_time_str, True, TEXT_COLOR)
        screen.blit(black_label, (PANEL_X + 20, PANEL_Y + 200))
        screen.blit(black_time_surf, (PANEL_X + 20, PANEL_Y + 220))

        # ปุ่ม Exit (ด้านล่าง Panel)
        exit_rect = pygame.Rect(PANEL_X, SCREEN_HEIGHT - 60, PANEL_WIDTH, 40)
        pygame.draw.rect(screen, (255, 0, 0), exit_rect)
        exit_text = FONT.render("Exit", True, (255, 255, 255))
        screen.blit(exit_text, (
            exit_rect.x + (exit_rect.width - exit_text.get_width()) // 2,
            exit_rect.y + (exit_rect.height - exit_text.get_height()) // 2
        ))

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    play_chess()
