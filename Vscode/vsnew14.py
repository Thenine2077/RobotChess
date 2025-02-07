import tkinter as tk
import tkinter.messagebox as messagebox
import math
import serial
import serial.tools.list_ports
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ตัวแปรคงที่สำหรับความยาวของแขนหุ่นยนต์
OA = 45.5
AB = 36
BC = 104.05
CD = 194.71
DE = 150.22
EF = 10
alpha = math.radians(45)

# ฟังก์ชันการตั้งค่า Serial
arduino = None
def connect_to_arduino():
    global arduino
    selected_port = port_var.get()
    if selected_port:
        try:
            arduino = serial.Serial(selected_port, 9600, timeout=1)
            messagebox.showinfo("Connection", f"Connected to {selected_port}")
        except serial.SerialException:
            messagebox.showerror("Error", f"Failed to connect to {selected_port}")

# ฟังก์ชันเพื่อส่งคำสั่งไปยัง Arduino
def send_command(command_str):
    if arduino and arduino.is_open:
        command_str += '\n'
        arduino.write(command_str.encode())
        time.sleep(0.1)
        if arduino.in_waiting > 0:
            response = arduino.readline().decode().strip()
            print("Arduino:", response)
        messagebox.showinfo("Command Sent", f"Command sent to Arduino: {command_str.strip()}")
    else:
        messagebox.showwarning("Warning", "Arduino not connected.")

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
        gamma = theta2 + theta3 + theta4

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

# ฟังก์ชันสำหรับการวาดกราฟหุ่นยนต์
def plot_robot_arm(theta1, theta2, theta3, theta4):
    ax.clear()

    points = calculate_forward_kinematics(theta1, theta2, theta3, theta4)
    x_points = [p[0] for p in points]
    y_points = [p[1] for p in points]
    z_points = [p[2] for p in points]

    ax.scatter(x_points, y_points, z_points, color='r', label='Joints', s=100)
    ax.plot(x_points, y_points, z_points, label='Arm Path', marker='o')

    ax.set_xlabel('X-axis (mm)')
    ax.set_ylabel('Y-axis (mm)')
    ax.set_zlabel('Z-axis (mm)')
    ax.set_title("3D Plot of Robotic Arm Position")
    ax.legend()
    canvas.draw()

# ฟังก์ชัน Home
def home_position():
    send_command("h")
    theta1, theta2, theta3, theta4 = 0, 90, 0, 0
    plot_robot_arm(theta1, theta2, theta3, theta4)

# สร้างหน้าต่าง UI
root = tk.Tk()
root.title("Robotic Arm Kinematics")

# การแสดงกราฟ
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# พอร์ตคอมโบบ็อกซ์สำหรับเลือกพอร์ต
port_frame = tk.Frame(root)
port_frame.pack()
tk.Label(port_frame, text="Select Arduino Port:").grid(row=0, column=0)
port_var = tk.StringVar()
ports = [port.device for port in serial.tools.list_ports.comports()]
port_dropdown = tk.OptionMenu(port_frame, port_var, *ports)
port_dropdown.grid(row=0, column=1)
tk.Button(port_frame, text="Connect", command=connect_to_arduino).grid(row=0, column=2)

# สร้างอินพุตและปุ่มต่าง ๆ สำหรับ theta และ xyz
frame = tk.Frame(root)
frame.pack()

# Fields for theta inputs
tk.Label(frame, text="Theta 1 (°):").grid(row=0, column=0)
theta1_entry = tk.Entry(frame)
theta1_entry.grid(row=0, column=1)

tk.Label(frame, text="Theta 2 (°):").grid(row=1, column=0)
theta2_entry = tk.Entry(frame)
theta2_entry.grid(row=1, column=1)

tk.Label(frame, text="Theta 3 (°):").grid(row=2, column=0)
theta3_entry = tk.Entry(frame)
theta3_entry.grid(row=2, column=1)

tk.Label(frame, text="Theta 4 (°):").grid(row=3, column=0)
theta4_entry = tk.Entry(frame)
theta4_entry.grid(row=3, column=1)

# Fields for XYZ inputs
tk.Label(frame, text="X (mm):").grid(row=0, column=2)
x_entry = tk.Entry(frame)
x_entry.grid(row=0, column=3)

tk.Label(frame, text="Y (mm):").grid(row=1, column=2)
y_entry = tk.Entry(frame)
y_entry.grid(row=1, column=3)

tk.Label(frame, text="Z (mm):").grid(row=2, column=2)
z_entry = tk.Entry(frame)
z_entry.grid(row=2, column=3)

tk.Label(frame, text="A (°):").grid(row=3, column=2)
a_entry = tk.Entry(frame)
a_entry.grid(row=3, column=3)

# Buttons for kinematic calculations
tk.Button(frame, text="Move with Forward Point", command=update_xyz_from_theta).grid(row=4, column=0, columnspan=2)
tk.Button(frame, text="Move with Inverse Point", command=update_theta_from_xyz).grid(row=4, column=2, columnspan=2)
tk.Button(frame, text="Move to XYZ", command=move_to_xyz).grid(row=5, column=0, columnspan=4)
tk.Button(frame, text="Home", command=home_position).grid(row=6, column=0, columnspan=4)

# เริ่มต้น loop ของ tkinter
root.mainloop()
