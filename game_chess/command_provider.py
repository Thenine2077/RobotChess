def get_user_move():
    """
    ฟังก์ชันนี้คืนค่าการเดินหมากที่ผู้ใช้พิมพ์
    """
    user_input = input("Enter your move (e.g., e2e4): ")
    return user_input.strip()
