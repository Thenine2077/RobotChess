import multiprocessing
import time

def send_moves(move_queue):
    """ ฟังก์ชันสำหรับรับค่าจากผู้ใช้ และส่งไปยัง `game.py` """
    while True:
        move = input("Your move (e.g., e2e4): ").strip()
        if move.lower() == "exit":
            break
        move_queue.put(move)
        time.sleep(1)  # หน่วงเวลาเล็กน้อยเพื่อให้เกมทำงานทัน

if __name__ == "__main__":
    move_queue = multiprocessing.Manager().Queue()
    
    # รัน `game.py` ใน Process แยก
    game_process = multiprocessing.Process(target=__import__("game").play_chess, args=(move_queue,))
    game_process.start()

    # เริ่มรับ input จากผู้ใช้
    send_moves(move_queue)

    # รอให้เกมจบ
    game_process.join()
