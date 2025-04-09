import pygame
import chess
import os
import multiprocessing
import math
import sys
import os

# ใช้ os.path.abspath() เพื่อให้เส้นทางสมบูรณ์
sys.path.append(os.path.abspath(r"D:\Project RobotChess\RobotChess\game_chess\Ai"))

from game_minimax import find_best_move  # นำเข้า Minimax จาก game_minimax.py


# ตั้งค่าขนาดกระดานหมากรุก
BOARD_SIZE = 600
SQUARE_SIZE = BOARD_SIZE // 8
WHITE = (240, 217, 181)
BROWN = (181, 136, 99)
TEXT_COLOR = (0, 0, 0)  # สีตัวอักษร (ดำ)

# ตั้งค่า font
pygame.init()
FONT = pygame.font.Font(None, 24)  # ใช้ font เริ่มต้นของ pygame

# โฟลเดอร์ที่เก็บไฟล์หมาก PNG
ASSET_FOLDER = r"D:\Project RobotChess\RobotChess\game_chess\GUI\assets"

# จัดเก็บภาพหมากรุก
piece_images = {}

# ชื่อไฟล์ของหมากรุก
pieces = {
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
    """ โหลดรูปหมากรุก (PNG) จากโฟลเดอร์ assets """
    for piece, filename in pieces.items():
        full_path = os.path.join(ASSET_FOLDER, filename)
        if os.path.exists(full_path):
            piece_images[piece] = pygame.image.load(full_path)
            piece_images[piece] = pygame.transform.scale(piece_images[piece], (SQUARE_SIZE, SQUARE_SIZE))
        else:
            print(f"⚠️ ไม่พบไฟล์: {full_path}")

def draw_board(screen, board):
    """ วาดกระดานหมากรุก พร้อมตัวอักษรตำแหน่ง a-h และ 1-8 """
    for row in range(8):
        for col in range(8):
            color = WHITE if (row + col) % 2 == 0 else BROWN
            pygame.draw.rect(screen, color, pygame.Rect(col * SQUARE_SIZE + 40, row * SQUARE_SIZE, SQUARE_SIZE, SQUARE_SIZE))

            piece = board.piece_at(chess.square(col, 7 - row))
            if piece:
                screen.blit(piece_images[piece.symbol()], (col * SQUARE_SIZE + 40, row * SQUARE_SIZE))

    # วาดตัวเลข 1-8 ด้านซ้ายของกระดาน
    for i in range(8):
        text = FONT.render(str(8 - i), True, TEXT_COLOR)
        screen.blit(text, (10, i * SQUARE_SIZE + SQUARE_SIZE // 3))

    # วาดตัวอักษร a-h ด้านล่างของกระดาน
    for i, letter in enumerate("abcdefgh"):
        text = FONT.render(letter, True, TEXT_COLOR)
        screen.blit(text, (i * SQUARE_SIZE + 55, BOARD_SIZE + 10))

def play_chess(move_queue):
    """ เกมหมากรุกที่รับค่าการเดินจาก `move_queue` """
    pygame.init()
    screen = pygame.display.set_mode((BOARD_SIZE + 40, BOARD_SIZE + 30))  # ขยายขนาดหน้าจอให้พอดี
    pygame.display.set_caption("Chess Game with Minimax Algorithm")

    clock = pygame.time.Clock()
    board = chess.Board()
    load_pieces()

    running = True
    while running:
        screen.fill((255, 255, 255))  # พื้นหลังสีขาว
        draw_board(screen, board)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        pygame.display.flip()
        clock.tick(30)

        # ตรวจสอบว่าเกมจบหรือยัง
        if board.is_game_over():
            print("Game Over! Result:", board.result())
            break

        # รับค่าจาก Queue (ถ้ามี)
        if board.turn:  # Turn ของผู้เล่น
            if not move_queue.empty():
                user_move = move_queue.get()
                try:
                    move_obj = chess.Move.from_uci(user_move)

                    # ตรวจสอบว่า Pawn เดินไปถึงแถวที่ 7 หรือ 8 แล้วโปรโมท
                    if move_obj.promotion is None and board.piece_at(move_obj.to_square) == chess.PAWN:
                        if move_obj.to_square == 56 or move_obj.to_square == 48:  # แถวที่ 7 หรือ 8
                            # ให้เลือกการโปรโมท
                            promotion_piece = chess.QUEEN  # ตัวเลือกโปรโมทเป็นควีน (สามารถเปลี่ยนเป็น ม้า, บิชอป, เรือ)
                            move_obj = move_obj.promote(promotion_piece)

                    if move_obj in board.legal_moves:
                        board.push(move_obj)
                    else:
                        print("❌ Invalid move:", user_move)
                except ValueError:
                    print("❌ Invalid move format:", user_move)

        # AI เดินหมาก
        else:
            move = find_best_move(board, depth=3)
            print(f"AI Move: {move}")
            board.push(move)

    pygame.quit()

# สร้าง Queue และเริ่มเกม
if __name__ == "__main__":
    move_queue = multiprocessing.Manager().Queue()
    process = multiprocessing.Process(target=play_chess, args=(move_queue,))
    process.start()
    process.join()
