import tkinter as tk
import tkinter.messagebox as messagebox
import math
import serial
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from serial.tools import list_ports
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from tkinter import *
from PIL import Image, ImageTk
import cv2

# ฟังก์ชันค้นหา Arduino
def find_arduinos():
    ports = list_ports.comports()
    arduino_ports = []
    for port in ports:
        if "Arduino" in port.description:
            arduino_ports.append((port.device, port.description))
    return arduino_ports


# ฟังก์ชันเลือกพอร์ต Arduino ผ่าน OptionMenu
def select_arduino_gui(arduino_ports):
    root = tk.Tk()
    root.withdraw()  # ซ่อนหน้าต่างหลัก
    if not arduino_ports:
        messagebox.showerror("Error", "ไม่พบ Arduino!")
        root.destroy()
        return None


    # สร้างหน้าต่างใหม่สำหรับเลือกพอร์ต
    top = Toplevel(root)
    top.title("เลือกพอร์ต Arduino")
    
    selected_port = StringVar(top)
    selected_port.set(f"{arduino_ports[0][0]} - {arduino_ports[0][1]}")  # ค่าเริ่มต้น

    # Dropdown menu
    dropdown = OptionMenu(top, selected_port, *[f"{p[0]} - {p[1]}" for p in arduino_ports])
    dropdown.pack(pady=1)

    # ปุ่มยืนยัน
    def confirm_selection():
        top.destroy()
        root.quit()
    
    Button(top, text="ยืนยัน", command=confirm_selection).pack(pady=5)
    root.mainloop()

    for port in arduino_ports:
        if selected_port.get().startswith(port[0]):
            return port[0]
    return None


# ค้นหา Arduino
arduino_ports = find_arduinos()
if not arduino_ports:
    messagebox.showerror("Error", "ไม่พบพอร์ต Arduino! กรุณาเชื่อมต่อ Arduino และลองอีกครั้ง")
    exit()

# ให้ผู้ใช้เลือกพอร์ต
arduino_port = select_arduino_gui(arduino_ports)
if not arduino_port:
    messagebox.showinfo("Cancel", "คุณไม่ได้เลือกพอร์ต Arduino")
    exit()

# ตั้งค่า Serial
try:
    arduino = serial.Serial(arduino_port, 9600, timeout=1)
    messagebox.showinfo("Success", f"เชื่อมต่อกับ Arduino สำเร็จที่พอร์ต {arduino_port}")
except Exception as e:
    messagebox.showerror("Error", f"ไม่สามารถเชื่อมต่อกับ Arduino: {e}")
    exit()








# ตัวแปรคงที่สำหรับความยาวของแขนหุ่นยนต์
OA = 122.75 #
AB = 53.10 #
BC = 104.05  # Up j2 stage 2 gear ratio
CD = 194.71  # La2
DE = 150.22  # La3
EF = 10  # Lpl
alpha = math.radians(45)

# ฟังก์ชันเพื่อส่งคำสั่งไปยัง Arduino
def send_command(command_str):
    command_str += '\n'
    arduino.write(command_str.encode())
    time.sleep(0.1)
    if arduino.in_waiting > 0:
        response = arduino.readline().decode().strip()
        print("Arduino:", response)
    messagebox.showinfo("Command Sent", f"Command sent to Arduino: {command_str.strip()}")


# ฟังก์ชันการคำนวณ Forward Kinematics
def calculate_forward_kinematics(theta1, theta2, theta3, theta4):
    xA, yA, zA = (0, 0, OA)

    xB = AB * math.cos(math.radians(theta1))
    yB = AB * math.sin(math.radians(theta1))
    zB = OA

    xC = xB + BC * math.cos(alpha) * math.cos(math.radians(theta1))
    yC = yB + BC * math.cos(alpha) * math.sin(math.radians(theta1))
    zC = zB + BC * math.sin(alpha)

    xD = xC + CD * math.cos(math.radians(theta2)) * math.cos(math.radians(theta1))
    yD = yC + CD * math.cos(math.radians(theta2)) * math.sin(math.radians(theta1))
    zD = zC + CD * math.sin(math.radians(theta2))

    xE = xD + DE * math.cos(math.radians(theta2 + theta3)) * math.cos(math.radians(theta1))
    yE = yD + DE * math.cos(math.radians(theta2 + theta3)) * math.sin(math.radians(theta1))
    zE = zD + DE * math.sin(math.radians(theta2 + theta3))

    xF = xE + EF * math.cos(math.radians(theta2 + theta3 + theta4)) * math.cos(math.radians(theta1))
    yF = yE + EF * math.cos(math.radians(theta2 + theta3 + theta4)) * math.sin(math.radians(theta1))
    zF = zE + EF * math.sin(math.radians(theta2 + theta3 + theta4))

    return [(0, 0, OA), (xB, yB, zB), (xC, yC, zC), (xD, yD, zD), (xE, yE, zE), (xF, yF, zF)]

# ฟังก์ชันการคำนวณ Inverse Kinematics
def calculate_inverse_kinematics(x_target, y_target, z_target, gamma):
    theta1 = math.degrees(math.atan2(y_target, x_target))

    CG = math.sqrt(x_target**2 + y_target**2) - AB * math.cos(alpha)
    EG = z_target - OA - BC * math.sin(alpha)

    term = ((CG**2 + EG**2 - CD**2 - DE**2) / (2 * CD * DE))
    term = max(min(term, 1), -1)
    theta3 = math.degrees(math.acos(term))

    EH = DE * math.sin(math.radians(theta3))
    CH = CD + DE * math.cos(math.radians(theta3))
    beta = math.degrees(math.atan2(EH, CH))

    theta2 = math.degrees(math.atan2(EG, CG)) - beta
    theta4 = gamma - theta1 - theta2 - theta3

    return theta1, theta2, theta3, theta4

# ฟังก์ชันอัปเดต x, y, z, a โดยใช้ Forward Kinematics
def update_xyz_from_theta():
    try:
        theta1 = float(theta1_entry.get() or 0)
        theta2 = float(theta2_entry.get() or 0)
        theta3 = float(theta3_entry.get() or 0)
        theta4 = float(theta4_entry.get() or 0)

        points = calculate_forward_kinematics(theta1, theta2, theta3, theta4)
        xF, yF, zF = points[-1]
        gamma = theta2 + theta3 + theta4  # อาจปรับตามต้องการ

        x_entry.delete(0, tk.END)
        x_entry.insert(0, f"{xF:.2f}")
        y_entry.delete(0, tk.END)
        y_entry.insert(0, f"{yF:.2f}")
        z_entry.delete(0, tk.END)
        z_entry.insert(0, f"{zF:.2f}")
        a_entry.delete(0, tk.END)
        a_entry.insert(0, f"{gamma:.2f}")

        plot_robot_arm(theta1, theta2, theta3, theta4)
    except ValueError:
        pass  # Ignore invalid inputs

# ฟังก์ชันอัปเดต theta โดยใช้ Inverse Kinematics
def update_theta_from_xyz():
    try:
        x_target = float(x_entry.get() or 0)
        y_target = float(y_entry.get() or 0)
        z_target = float(z_entry.get() or 0)
        gamma = float(a_entry.get() or 0)

        results = calculate_inverse_kinematics(x_target, y_target, z_target, gamma)
        if results:
            theta1, theta2, theta3, theta4 = results
            theta1_entry.delete(0, tk.END)
            theta1_entry.insert(0, f"{theta1:.2f}")
            theta2_entry.delete(0, tk.END)
            theta2_entry.insert(0, f"{theta2:.2f}")
            theta3_entry.delete(0, tk.END)
            theta3_entry.insert(0, f"{theta3:.2f}")
            theta4_entry.delete(0, tk.END)
            theta4_entry.insert(0, f"{theta4:.2f}")

            plot_robot_arm(theta1, theta2, theta3, theta4)
    except ValueError:
        pass  # Ignore invalid inputs

# ฟังก์ชัน Move สำหรับค่า x, y, z, a โดยใช้ Inverse Kinematics และส่งคำสั่งไปยัง Arduino
def move_to_xyz():
    try:
        x_target = float(x_entry.get() or 0)
        y_target = float(y_entry.get() or 0)
        z_target = float(z_entry.get() or 0)
        gamma = float(a_entry.get() or 0)

        results = calculate_inverse_kinematics(x_target, y_target, z_target, gamma)
        if results:
            theta1, theta2, theta3, theta4 = results
            send_command(f"f {int(round(theta1))} {int(round(theta2))} {int(round(theta3))} {int(round(theta4))}")
    except ValueError:
        pass  # Ignore invalid inputs

# ฟังก์ชันสำหรับการเคลื่อนที่ทั้งหมด (คำสั่ง 'f')
def move_all_motors(theta1, theta2, theta3, theta4):
    command_str = f"f {int(round(theta1))} {int(round(theta2))} {int(round(theta3))} {int(round(theta4))}"
    send_command(command_str)

# ฟังก์ชันสำหรับการวาดกราฟหุ่นยนต์
def plot_robot_arm(theta1, theta2, theta3, theta4):
    ax.clear()

    # คำนวณตำแหน่งของแขน
    points = calculate_forward_kinematics(theta1, theta2, theta3, theta4)

    # วาดกราฟจุดทั้งหมด
    x_points = [p[0] for p in points]
    y_points = [p[1] for p in points]
    z_points = [p[2] for p in points]

    # เพิ่มจุดสุดท้าย
    ax.scatter(x_points, y_points, z_points, color='r', label='Joints', s=100)

    # วาดกราฟ
    ax.plot(x_points, y_points, z_points, label='Arm Path', marker='o')

    ax.set_xlabel('X-axis (mm)')
    ax.set_ylabel('Y-axis (mm)')
    ax.set_zlabel('Z-axis (mm)')
    ax.set_title("3D Plot of Robotic Arm Position")
    ax.legend()
    canvas.draw()  # ใช้เพื่ออัป
    

# ฟังก์ชัน Home
def home_position():
    send_command("h")  # ส่งคำสั่ง 'h' ไปยัง Arduino
    # คำนวณและอัปเดตกราฟสำหรับตำแหน่งโฮม
    theta1, theta2, theta3, theta4 = 0, 90, 0, 0  # กำหนดตำแหน่งโฮมที่ (0, 90, 0, 0)
    plot_robot_arm(theta1, theta2, theta3, theta4)

# ฟังก์ชันสำหรับการส่งคำสั่ง f 0 0 0 0
def move_to_zero():
    send_command("f 0 0 0 0")  # ส่งคำสั่ง 'f 0 0 0 0' ไปยัง Arduino
    plot_robot_arm(0, 90, 0, 0)  # อัปเดตกราฟให้แสดงตำแหน่งโฮม

# ส่วนของ UI
root = tk.Tk()
root.title("Robotic Arm Kinematics")

# ตั้งค่าฟอนต์
font_style = ('Helvetica', 14)
font_style_header =('cursive', 20)

# ตั้งค่ากราฟ
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
canvas = FigureCanvasTkAgg(fig, master=root)

# กราฟอยู่ทางขวา
canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=1)

# สร้าง Frame สำหรับปุ่มทางซ้าย
left_frame = tk.Frame(root)
left_frame.pack(side=tk.LEFT, padx=10, pady=10)

# ตั้งค่า grid layout ให้พื้นที่ทางซ้ายกว้างขึ้น
root.grid_columnconfigure(0, weight=1, minsize=400)  # กำหนดความกว้างของคอลัมน์แรก
root.grid_columnconfigure(1, weight=2, minsize=600)  # กำหนดความกว้างของคอลัมน์ที่ 2 (กราฟ)
root.grid_rowconfigure(0, weight=1)


# การส่งค่า theta ไปยัง Arduino สำหรับการเคลื่อนที่ทั้งหมด
def on_move_all():
    theta1 = float(theta1_entry.get() or 0)
    theta2 = float(theta2_entry.get() or 0)
    theta3 = float(theta3_entry.get() or 0)
    theta4 = float(theta4_entry.get() or 0)
    move_all_motors(theta1, theta2, theta3, theta4)



# เลือกพอร์ตที่ใช้งานอยู่ (ตามที่ผู้ใช้เลือกไว้ในหน้าต่างก่อนหน้านี้)
selected_index = 0
for i, (port, _) in enumerate(arduino_ports):
    if port == arduino_port:
        selected_index = i
        break

# ฟังก์ชันเปิดกล้อง
def open_camera():
    # เปิดกล้อง
    cap = cv2.VideoCapture(1)

    # ฟังก์ชันเพื่ออัพเดทภาพใน GUI
    def update_frame():
        ret, frame = cap.read()
        if ret:
            # แปลงภาพจาก BGR (OpenCV) เป็น RGB (Pillow)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            img_tk = ImageTk.PhotoImage(image=img)

            # อัพเดทภาพใน Label
            label.config(image=img_tk)
            label.image = img_tk

        # เรียกใช้อีกครั้งทุกๆ 10 มิลลิวินาที
        label.after(10, update_frame)

    # สร้างหน้าต่างแสดงผล
    window = tk.Toplevel()
    window.title("Camera")

    # แสดงภาพ
    label = tk.Label(window)
    label.pack()

    # เริ่มอัพเดทภาพ
    update_frame()

# สร้าง Listbox ก่อน
listbox = tk.Listbox(left_frame, font=('Helvetica', 12), height=3, selectmode=tk.SINGLE)
listbox.pack(pady=(5, 5), fill=tk.X)

# ตั้งค่า index ที่ต้องการเลือก (เช่น index 0)
selected_index = 0

# ตรวจสอบว่ามีรายการใน Listbox ก่อน
if listbox.size() > 0:
    listbox.select_set(selected_index)  # เลือกพอร์ตที่ใช้งาน


tk.Label(left_frame, text="Theta 1", font=font_style).pack(pady=(5, 5))
theta1_entry = tk.Entry(left_frame, font=font_style, justify="center")  # ตั้งค่าตรงกลาง
theta1_entry.pack(pady=(0, 5))
theta1_entry.bind("<KeyRelease>", lambda event: update_xyz_from_theta())

tk.Label(left_frame, text="Theta 2", font=font_style).pack(pady=(5, 5))
theta2_entry = tk.Entry(left_frame, font=font_style, justify="center")  # ตั้งค่าตรงกลาง
theta2_entry.pack(pady=(0, 5))
theta2_entry.bind("<KeyRelease>", lambda event: update_xyz_from_theta())

tk.Label(left_frame, text="Theta 3", font=font_style).pack(pady=(5, 5))
theta3_entry = tk.Entry(left_frame, font=font_style, justify="center")  # ตั้งค่าตรงกลาง
theta3_entry.pack(pady=(0, 5))
theta3_entry.bind("<KeyRelease>", lambda event: update_xyz_from_theta())

tk.Label(left_frame, text="Theta 4", font=font_style).pack(pady=(5, 5))
theta4_entry = tk.Entry(left_frame, font=font_style, justify="center")  # ตั้งค่าตรงกลาง
theta4_entry.pack(pady=(0, 5))
theta4_entry.bind("<KeyRelease>", lambda event: update_xyz_from_theta())

tk.Button(left_frame, text="Move All Motors", command=on_move_all, font=font_style,height=1, width=25).pack(pady=(1, 1))

# UI การป้อนค่า x, y, z, a สำหรับการคำนวณ Inverse Kinematics
tk.Label(left_frame, text="X", font=font_style).pack(pady=(5, 5))
x_entry = tk.Entry(left_frame, font=font_style, justify="center")  # ตั้งค่าตรงกลาง
x_entry.pack(pady=(0, 5))
x_entry.bind("<KeyRelease>", lambda event: update_theta_from_xyz())

tk.Label(left_frame, text="Y", font=font_style).pack(pady=(5, 5))
y_entry = tk.Entry(left_frame, font=font_style, justify="center")  # ตั้งค่าตรงกลาง
y_entry.pack(pady=(0, 5))
y_entry.bind("<KeyRelease>", lambda event: update_theta_from_xyz())

tk.Label(left_frame, text="Z", font=font_style).pack(pady=(5, 5))
z_entry = tk.Entry(left_frame, font=font_style, justify="center")  # ตั้งค่าตรงกลาง
z_entry.pack(pady=(0, 5))
z_entry.bind("<KeyRelease>", lambda event: update_theta_from_xyz())

tk.Label(left_frame, text="Angle A", font=font_style).pack(pady=(5, 5))
a_entry = tk.Entry(left_frame, font=font_style, justify="center")  # ตั้งค่าตรงกลาง
a_entry.pack(pady=(0, 5))
a_entry.bind("<KeyRelease>", lambda event: update_theta_from_xyz())


# ปุ่ม Move สำหรับการใช้ค่า x, y, z, a
tk.Button(left_frame, text="Move to X, Y, Z, A", command=move_to_xyz, font=font_style,height=2, width=25).pack(pady=(1, 1))


# ปุ่มสำหรับส่งคำสั่ง f 0 0 0 0
tk.Button(left_frame, text="Move to Zero", command=move_to_zero, font=font_style,height=2, width=25).pack(pady=(1, 1))

#เปิดกล้อง

tk.Button(left_frame, text="Camera", command=open_camera, font=font_style,height=2, width=25).pack(pady=(1, 1))

# ปุ่ม Home
tk.Button(left_frame, text="Home", command=home_position, font=font_style,height=1, width=25).pack(pady=(0, 1))


# ปุ่ม Stop
tk.Button(left_frame, text="Stop", command=lambda: send_command("s"), font=font_style,height=3, width=25,bg='red',foreground="white").pack(pady=(0, 1))

# เพิ่มฟังก์ชันสำหรับอัปเดตพอร์ต Arduino แบบเรียลไทม์
def update_arduino_ports():
    global listbox  # ใช้ตัวแปร listbox จาก global scope
    arduino_ports = find_arduinos()  # ค้นหา Arduino ใหม่
    listbox_items = listbox.get(0, tk.END)  # ดึงรายการใน Listbox

    # ลบรายการเก่าออกจาก Listbox
    for item in listbox_items:
        listbox.delete(0, tk.END)

    # เพิ่มรายการใหม่เข้าไปใน Listbox
    for port, description in arduino_ports:
        listbox.insert(tk.END, f"{port} - {description}")

    # ถ้าพอร์ตที่เลือกถูกถอดออก ให้แจ้งเตือนและล้างการเลือก
    selected = listbox.curselection()
    if selected:
        selected_port = listbox.get(selected[0])
        if selected_port not in current_port_list:
            listbox.selection_clear(0, tk.END)  # ล้างการเลือก
            messagebox.showwarning("Warning", f"พอร์ต {selected_port} ถูกถอดออก!")

    # ตั้งเวลาให้ฟังก์ชันนี้เรียกซ้ำอีกครั้งใน 1 วินาที
    root.after(1000, update_arduino_ports)

# เรียกใช้ฟังก์ชัน update_arduino_ports หลังจากสร้างหน้าต่าง
update_arduino_ports()

def schedule_update():
    update_arduino_ports()
    root.after(1000, schedule_update)  # อัปเดตทุกๆ 1 วินาที

# เริ่มการอัปเดต
schedule_update()

# เริ่ม Tkinter main loop
root.mainloop()