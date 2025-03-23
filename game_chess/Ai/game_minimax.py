import pygame
import chess
import chess.engine
import os
import multiprocessing

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
ASSET_FOLDER = r"D:\RobotChess\RobotChess\Project Robot\assets"

# ตำแหน่งของ Stockfish
STOCKFISH_PATH = r"D:\RobotChess\RobotChess\Project Robot\stockfish-windows-x86-64-avx2\stockfish\stockfish-windows-x86-64-avx2.exe"

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
    pygame.display.set_caption("Chess Game with Stockfish")

    clock = pygame.time.Clock()
    board = chess.Board()
    load_pieces()

    
import math

# Evaluation function for board position
piece_values = {
    chess.PAWN: 1,
    chess.KNIGHT: 3,
    chess.BISHOP: 3,
    chess.ROOK: 5,
    chess.QUEEN: 9,
    chess.KING: 0
}

def evaluate_board(board):
    value = 0
    for piece_type in piece_values:
        value += len(board.pieces(piece_type, chess.WHITE)) * piece_values[piece_type]
        value -= len(board.pieces(piece_type, chess.BLACK)) * piece_values[piece_type]
    return value

def minimax(board, depth, alpha, beta, maximizing_player):
    if depth == 0 or board.is_game_over():
        return evaluate_board(board)

    if maximizing_player:
        max_eval = -math.inf
        for move in board.legal_moves:
            board.push(move)
            eval = minimax(board, depth - 1, alpha, beta, False)
            board.pop()
            max_eval = max(max_eval, eval)
            alpha = max(alpha, eval)
            if beta <= alpha:
                break
        return max_eval
    else:
        min_eval = math.inf
        for move in board.legal_moves:
            board.push(move)
            eval = minimax(board, depth - 1, alpha, beta, True)
            board.pop()
            min_eval = min(min_eval, eval)
            beta = min(beta, eval)
            if beta <= alpha:
                break
        return min_eval

def find_best_move(board, depth=3):
    best_move = None
    best_value = -math.inf
    for move in board.legal_moves:
        board.push(move)
        board_value = minimax(board, depth - 1, -math.inf, math.inf, False)
        board.pop()
        if board_value > best_value:
            best_value = board_value
            best_move = move
    return best_move

    # ใช้ Minimax แทน Stockfish:
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
