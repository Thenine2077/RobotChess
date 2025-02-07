import sys

def get_user_move():
    # คืนค่าข้อมูลที่ต้องการ, ในที่นี้จะสมมติว่าผู้ใช้ใส่คำสั่งผ่าน command line
    return sys.argv[1] if len(sys.argv) > 1 else "e2e4"  # ตัวอย่าง: King's Pawn เดินจาก e2 ไป e4

if __name__ == "__main__":
    print(get_user_move())
