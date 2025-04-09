import chess
import math
import multiprocessing

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

# ฟังก์ชันเล่นหมากรุก
def play_chess(move_queue):
    board = chess.Board()  # สร้างกระดานหมากรุกใหม่
    running = True
    while running:
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

# สร้าง Queue และเริ่มเกม
if __name__ == "__main__":
    move_queue = multiprocessing.Manager().Queue()  # สร้าง Queue สำหรับการรับการเดินหมากจากผู้เล่น
    process = multiprocessing.Process(target=play_chess, args=(move_queue,))  # เริ่มกระบวนการเล่นหมากรุก
    process.start()
    process.join()
