import chess
import chess.engine

# สร้างกระดานหมากรุก
board = chess.Board()

# ใช้ Engine ที่ติดตั้ง (เช่น Stockfish)
with chess.engine.SimpleEngine.popen_uci("C:\Project Robot\stockfish-windows-x86-64-avx2\stockfish\stockfish-windows-x86-64-avx2") as engine:
    while not board.is_game_over():
        print(board)  # แสดงสถานะของกระดาน
        
        # ให้ AI ทำการเคลื่อนหมากรุก
        result = engine.play(board, chess.engine.Limit(time=2.0))  # จำกัดเวลา 2 วินาที
        print(f"AI move: {result.move}")
        
        # ทำการเคลื่อนหมากรุกของ AI
        board.push(result.move)
        
        # ถามผู้เล่นให้ทำการเคลื่อนหมากรุก
        print("Your turn!")
        move = input("Enter your move (e.g., e2e4): ")
        
        try:
            board.push_san(move)  # ทำการเคลื่อนหมากรุกของผู้เล่น
        except ValueError:
            print("Invalid move. Please try again.")
            continue

    print("Game Over!")
    print("Result: ", board.result())  # แสดงผลลัพธ์ของเกม
