import chess
import chess.engine
import tkinter as tk
from tkinter import messagebox
import command_provider  # เพิ่มการนำเข้าโมดูล command_provider
import subprocess  # เพิ่มการนำเข้า subprocess
import cv2  # สำหรับ OpenCV
import time 
import chess
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
from PIL import Image, ImageTk
import tkinter as tk
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import chess



def load_images():
    # โหลดรูปภาพหมากหมากรุก
    piece_images = {}
    pieces = ['b', 'k', 'n', 'p', 'q', 'r', 'white-b', 'white-k', 'white-n', 'white-p', 'white-q', 'white-r']
    
    for piece in pieces:
        # ใช้ชื่อไฟล์ตามที่ระบุ
        image_path = f"C:\\Project Robot\\image\\{piece}.png"
        image = Image.open(image_path)
        image = image.resize((40, 40), Image.Resampling.LANCZOS)
        piece_images[piece] = ImageTk.PhotoImage(image)
        
    return piece_images




# เริ่มต้น Stockfish
stockfish_path = "C:\\Project Robot\\stockfish-windows-x86-64-avx2\\stockfish\\stockfish-windows-x86-64-avx2.exe"
engine = chess.engine.SimpleEngine.popen_uci(stockfish_path)
board = chess.Board()

def detect_move():
    # จำลองการประมวลผลของ OpenCV
    print("Processing image for move detection...")  
    return "e2e4"  # ส่งคืนผลลัพธ์จำลองของ OpenCV

# สำหรับเรียกใช้ไฟล์และรับคำสั่ง
def get_user_move():
    # สั่งการทำงานของไฟล์ command_simulator.py และรับค่ากลับ
    command_output = subprocess.check_output(["python", "command_simulator.py", "pd4"], text=True)
    return command_output.strip()

# สร้าง chess.Board() เพื่อเริ่มเกมใหม่
board = chess.Board()

# เชื่อมต่อกับ Stockfish
engine = chess.engine.SimpleEngine.popen_uci(stockfish_path)

# สร้าง GUI
class ChessApp:
    def __init__(self, root, piece_images):
        self.root = root
        self.root.title("Chess Game with Stockfish")
        self.piece_images = piece_images

        self.board_frame = tk.Frame(root)
        self.board_frame.pack()

        self.squares = {}
        
        # กำหนดขนาดแถวและคอลัมน์ให้มีน้ำหนักเท่ากัน
        for i in range(8):
            self.board_frame.grid_columnconfigure(i, weight=1, uniform="equal")
            self.board_frame.grid_rowconfigure(i, weight=1, uniform="equal")

        for row in range(8):
            for col in range(8):
                square_color = "#DDB88C" if (row + col) % 2 == 0 else "#A66D4F"
                square = tk.Label(self.board_frame, bg=square_color, height=18, width=18)  # ขนาดช่องที่เหมาะสม
                square.grid(row=row, column=col, sticky="nsew")
                self.squares[(row, col)] = square

        self.update_board()

    def update_board(self):
        board = chess.Board()
        for i in range(64):
            row, col = divmod(i, 8)
            piece = board.piece_at(i)
            if piece:
                piece_symbol = piece.symbol().lower()  # ดึงสัญลักษณ์ของหมากและแปลงให้เป็นตัวพิมพ์เล็ก
                if piece.color == chess.WHITE:  # ถ้าเป็นหมากขาว
                    piece_symbol = f"white-{piece_symbol}"  # ใช้ชื่อไฟล์สำหรับหมากขาว
                self.squares[(row, col)].config(image=self.piece_images[piece_symbol])
            else:
                self.squares[(row, col)].config(image='')

    def make_move(self):
        if board.is_game_over():
            messagebox.showinfo("Game Over", f"Game Over! Result: {board.result()}")
            return

        # สมมติว่าผู้ใช้ทำการเคลื่อนที่
        user_move = "e2e4"  # ตัวอย่างการเคลื่อนที่
        self.info_label.config(text=f"Received move: {user_move}")

        move = chess.Move.from_uci(user_move)
        if move in board.legal_moves:
            board.push(move)
            self.update_board()
        else:
            messagebox.showerror("Invalid Move", "This move is not legal.")


    def stockfish_move(self):
        if not board.is_game_over():
            result = engine.play(board, chess.engine.Limit(time=2.0))
            board.push(result.move)
            self.update_board()
            self.info_label.config(text=f"Stockfish moved: {result.move.uci()}")


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
root = tk.Tk()

piece_images = load_images()
detect_move()
# เริ่มต้นโปรแกรม

# สร้างและแสดง ChessApp
app = ChessApp(root, piece_images)
root.mainloop()
# ปิดการเชื่อมต่อกับ Stockfish
engine.quit()
