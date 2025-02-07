import chess
import chess.engine
import tkinter as tk
from tkinter import messagebox

# กำหนดตำแหน่งของ Stockfish (กรณีที่ใช้ Windows, ให้เปลี่ยนเป็นตำแหน่งที่เก็บไฟล์ Stockfish)
stockfish_path = "C:\\Project Robot\\stockfish-windows-x86-64-avx2\\stockfish\\stockfish-windows-x86-64-avx2.exe"

# สร้าง chess.Board() เพื่อเริ่มเกมใหม่
board = chess.Board()

# เชื่อมต่อกับ Stockfish
engine = chess.engine.SimpleEngine.popen_uci(stockfish_path)

# สร้าง GUI
class ChessApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Chess Game with Stockfish")

        self.board_frame = tk.Frame(root)
        self.board_frame.pack()

        self.squares = {}  # จัดเก็บปุ่มกระดานหมากรุก
        for row in range(8):
            for col in range(8):
                square_color = "#DDB88C" if (row + col) % 2 == 0 else "#A66D4F"
                square = tk.Label(self.board_frame, width=4, height=2, bg=square_color, font=("Helvetica", 16))
                square.grid(row=row + 1, column=col + 1)  # เว้นที่สำหรับตัวเลขและตัวอักษร
                self.squares[(row, col)] = square

        # เพิ่มตัวเลขและตัวอักษรเพื่อระบุตำแหน่ง
        for row in range(8):
            row_label = tk.Label(self.board_frame, text=str(8 - row), width=2, height=2, font=("Helvetica", 14))
            row_label.grid(row=row + 1, column=0)

        for col in range(8):
            col_label = tk.Label(self.board_frame, text=chr(97 + col), width=4, height=1, font=("Helvetica", 14))
            col_label.grid(row=9, column=col + 1)

        self.move_entry = tk.Entry(root)
        self.move_entry.pack()

        self.move_button = tk.Button(root, text="Make Move", command=self.make_move)
        self.move_button.pack()

        self.info_label = tk.Label(root, text="Your move (e.g., Be4 for Bishop to e4):")
        self.info_label.pack()

        self.update_board()

    def update_board(self):
        for row in range(8):
            for col in range(8):
                piece = board.piece_at(chess.square(col, 7 - row))  # แปลงจากกระดาน 8x8
                if piece:
                    self.squares[(row, col)].config(text=piece.symbol())
                else:
                    self.squares[(row, col)].config(text="")

    def make_move(self):
        if board.is_game_over():
            messagebox.showinfo("Game Over", f"Game Over! Result: {board.result()}")
            return

        if board.turn:  # True = สีขาว (Player)
            user_move = self.move_entry.get().strip()
            try:
                move = self.parse_move(user_move)
                if move in board.legal_moves:
                    board.push(move)
                    self.update_board()
                    self.move_entry.delete(0, tk.END)  # เคลียร์ข้อความในช่องป้อนคำสั่ง
                    self.stockfish_move()
                else:
                    messagebox.showerror("Invalid Move", "This move is not legal.")
            except ValueError:
                messagebox.showerror("Invalid Format", "Invalid move format. Use format like Be4 for Bishop to e4.")

    def stockfish_move(self):
        if not board.is_game_over():
            result = engine.play(board, chess.engine.Limit(time=2.0))
            board.push(result.move)
            dest_square = result.move.uci()[2:4]
            messagebox.showinfo("Stockfish Move", f"Stockfish moves to: {dest_square}")
            self.update_board()

    def parse_move(self, user_input):
        if len(user_input) < 2:
            raise ValueError("Invalid move format")

        piece_symbol = user_input[0].upper() if user_input[0].isalpha() else "P"
        destination = user_input[-2:]

        for move in board.legal_moves:
            if move.uci()[2:4] == destination:
                piece = board.piece_at(move.from_square)
                if piece and piece.symbol().upper() == piece_symbol:
                    return move

        raise ValueError("Invalid move format or piece selection")

# เริ่มต้นโปรแกรม
root = tk.Tk()
app = ChessApp(root)
root.mainloop()

# ปิดการเชื่อมต่อกับ Stockfish
engine.quit()
