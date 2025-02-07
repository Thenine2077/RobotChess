import tkinter as tk
from tkinter import messagebox, StringVar, Toplevel, OptionMenu, Button, Label, Entry
import math
import serial
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from serial.tools import list_ports
from PIL import Image, ImageTk
import cv2
from scipy.optimize import minimize  # Import for optimization

# Function to find Arduino devices
def find_arduinos():
    ports = list_ports.comports()
    arduino_ports = []
    for port in ports:
        if "Arduino" in port.description:
            arduino_ports.append((port.device, port.description))
    return arduino_ports

# Function to select Arduino port via OptionMenu
def select_arduino_gui(arduino_ports):
    root = tk.Tk()
    root.withdraw()  # Hide the main window
    if not arduino_ports:
        messagebox.showerror("Error", "No Arduino found!")
        root.destroy()
        return None

    # Create a new window for port selection
    top = Toplevel(root)
    top.title("Select Arduino Port")
    
    selected_port = StringVar(top)
    selected_port.set(f"{arduino_ports[0][0]} - {arduino_ports[0][1]}")  # Default value

    # Dropdown menu
    dropdown = OptionMenu(top, selected_port, *[f"{p[0]} - {p[1]}" for p in arduino_ports])
    dropdown.pack(pady=1)

    # Confirm button
    def confirm_selection():
        top.destroy()
        root.quit()
    
    Button(top, text="Confirm", command=confirm_selection).pack(pady=5)
    root.mainloop()

    for port in arduino_ports:
        if selected_port.get().startswith(port[0]):
            return port[0]
    return None

# Find Arduino devices
arduino_ports = find_arduinos()
if not arduino_ports:
    messagebox.showerror("Error", "No Arduino ports found! Please connect Arduino and try again.")
    exit()

# Let the user select a port
arduino_port = select_arduino_gui(arduino_ports)
if not arduino_port:
    messagebox.showinfo("Cancel", "You did not select an Arduino port.")
    exit()

# Set up Serial connection
try:
    arduino = serial.Serial(arduino_port, 9600, timeout=1)
    messagebox.showinfo("Success", f"Connected to Arduino at port {arduino_port}")
except Exception as e:
    messagebox.showerror("Error", f"Could not connect to Arduino: {e}")
    exit()

# Constants for robotic arm lengths
OA = 122.75
AB = 53.10
BC = 104.05
CD = 194.71
DE = 150.22
EF = 10
alpha = math.radians(45)

# Define angle limits for theta
LIMITS = {
    "theta1": (-1, 180),
    "theta2": (-1, 160),
    "theta3": (-120, 154),
    "theta4": (-180, 180),  # Adjusted limits for theta4
}

# Function to validate angles and display messages
def validate_all_thetas(theta_values):
    invalid_thetas = []
    for angle_name, value in theta_values.items():
        min_val, max_val = LIMITS[angle_name]
        if value < min_val or value > max_val:
            invalid_thetas.append(angle_name)
    if invalid_thetas:
        # If theta4 is the only angle exceeding limits, allow sending the command
        if invalid_thetas == ['theta4']:
            error_message_label.config(text="Warning: Theta 4 exceeds the specified limits")
            return True
        else:
            error_message_label.config(text="Cannot set angle beyond limits in " + ", ".join(invalid_thetas))
            return False
    else:
        error_message_label.config(text="")
        return True

# Function to send commands to Arduino
def send_command(command_str):
    command_str += '\n'
    arduino.write(command_str.encode())
    time.sleep(0.1)
    if arduino.in_waiting > 0:
        response = arduino.readline().decode().strip()
        print("Arduino:", response)
    messagebox.showinfo("Command Sent", f"Command sent to Arduino: {command_str.strip()}")

# Function for Forward Kinematics calculation
def calculate_forward_kinematics(theta1, theta2, theta3, theta4):
    xA, yA, zA = (0, 0, OA)

    xB = AB * math.cos(alpha) * math.cos(math.radians(theta1))
    yB = AB * math.cos(alpha) * math.sin(math.radians(theta1))
    zB = OA + AB * math.sin(alpha)

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

# Inverse Kinematics using optimization for higher accuracy
from scipy.optimize import minimize  # Import for optimization

def inverse_kinematics_optimization(x_target, y_target, z_target, gamma, initial_guess):
    # Objective function to minimize
    def objective(thetas):
        theta1, theta2, theta3, theta4 = thetas
        # Calculate forward kinematics
        points = calculate_forward_kinematics(theta1, theta2, theta3, theta4)
        xF, yF, zF = points[-1]
        gamma_calc = theta2 + theta3 + theta4
        # Position error
        pos_error = ((xF - x_target)**2 + (yF - y_target)**2 + (zF - z_target)**2)
        # Orientation error
        ori_error = (gamma_calc - gamma)**2
        # Total error
        total_error = pos_error + ori_error
        return total_error

    # Constraints for joint limits
    bounds = [
        LIMITS['theta1'],
        LIMITS['theta2'],
        LIMITS['theta3'],
        (-180, 180),  # For theta4, you mentioned to allow wider range
    ]

    # Run optimization
    result = minimize(
        objective,
        initial_guess,
        bounds=bounds,
        method='SLSQP',  # Sequential Least Squares Programming
        options={'ftol': 1e-8, 'disp': False}
    )

    if result.success:
        theta1, theta2, theta3, theta4 = result.x
        return theta1, theta2, theta3, theta4
    else:
        messagebox.showerror("Error", "Inverse kinematics optimization failed.")
        return None

# Function to update x, y, z, a using Forward Kinematics
def update_xyz_from_theta():
    try:
        theta1 = float(theta1_entry.get() or 0)
        theta2 = float(theta2_entry.get() or 0)
        theta3 = float(theta3_entry.get() or 0)
        theta4 = float(theta4_entry.get() or 0)

        theta_values = {
            "theta1": theta1,
            "theta2": theta2,
            "theta3": theta3,
            "theta4": theta4,
        }

        if not validate_all_thetas(theta_values):
            return  # Invalid input, do not proceed

        points = calculate_forward_kinematics(theta1, theta2, theta3, theta4)
        xF, yF, zF = points[-1]
        gamma = theta2 + theta3 + theta4  # Adjust as needed

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

# Function to update theta using Inverse Kinematics
def update_theta_from_xyz():
    try:
        x_target = float(x_entry.get() or 0)
        y_target = float(y_entry.get() or 0)
        z_target = float(z_entry.get() or 0)
        gamma = float(a_entry.get() or 0)

        # Initial guess for the optimizer
        initial_guess = [
            float(theta1_entry.get() or 0),
            float(theta2_entry.get() or 0),
            float(theta3_entry.get() or 0),
            float(theta4_entry.get() or 0),
        ]

        # Run the inverse kinematics optimization
        results = inverse_kinematics_optimization(x_target, y_target, z_target, gamma, initial_guess)
        if results:
            theta1, theta2, theta3, theta4 = results

            theta_values = {
                "theta1": theta1,
                "theta2": theta2,
                "theta3": theta3,
                "theta4": theta4,
            }

            if not validate_all_thetas(theta_values):
                return  # Invalid input, do not proceed

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

# Function to move to x, y, z, a using Inverse Kinematics and send command to Arduino
def move_to_xyz():
    try:
        x_target = float(x_entry.get() or 0)
        y_target = float(y_entry.get() or 0)
        z_target = float(z_entry.get() or 0)
        gamma = float(a_entry.get() or 0)

        # Initial guess for the optimizer
        initial_guess = [
            float(theta1_entry.get() or 0),
            float(theta2_entry.get() or 0),
            float(theta3_entry.get() or 0),
            float(theta4_entry.get() or 0),
        ]

        # Run the inverse kinematics optimization
        results = inverse_kinematics_optimization(x_target, y_target, z_target, gamma, initial_guess)
        if results:
            theta1, theta2, theta3, theta4 = results

            theta_values = {
                "theta1": theta1,
                "theta2": theta2,
                "theta3": theta3,
                "theta4": theta4,
            }

            if not validate_all_thetas(theta_values):
                return  # Invalid input, do not proceed

            send_command(f"f {int(round(theta1))} {int(round(theta2))} {int(round(theta3))} {int(round(theta4))}")
    except ValueError:
        pass  # Ignore invalid inputs

# Function to move all motors (command 'f')
def move_all_motors(theta1, theta2, theta3, theta4):
    command_str = f"f {int(round(theta1))} {int(round(theta2))} {int(round(theta3))} {int(round(theta4))}"
    send_command(command_str)

# Function to plot the robotic arm
def plot_robot_arm(theta1, theta2, theta3, theta4):
    ax.clear()

    # Calculate arm positions
    points = calculate_forward_kinematics(theta1, theta2, theta3, theta4)

    # Plot all points
    x_points = [p[0] for p in points]
    y_points = [p[1] for p in points]
    z_points = [p[2] for p in points]

    # Add the final point
    ax.scatter(x_points, y_points, z_points, color='r', label='Joints', s=100)

    # Plot the arm path
    ax.plot(x_points, y_points, z_points, label='Arm Path', marker='o')

    ax.set_xlabel('X-axis (mm)')
    ax.set_ylabel('Y-axis (mm)')
    ax.set_zlabel('Z-axis (mm)')
    ax.set_title("3D Plot of Robotic Arm Position")
    ax.legend()
    canvas.draw()  # Update the graph

# Home function
def home_position():
    send_command("h")  # Send 'h' command to Arduino
    # Compute and update the graph for home position
    theta1, theta2, theta3, theta4 = 0, 90, 0, 0  # Define home position
    plot_robot_arm(theta1, theta2, theta3, theta4)

# Function to send 'f 0 0 0 0' command
def move_to_zero():
    send_command("f 0 0 0 0")  # Send command to Arduino
    plot_robot_arm(0, 90, 0, 0)  # Update the graph to show home position

# UI Setup
root = tk.Tk()
root.title("Robotic Arm Kinematics")

# Set font styles
font_style = ('Helvetica', 14)

# Create left frame for buttons
left_frame = tk.Frame(root)
left_frame.pack(side=tk.LEFT, padx=10, pady=10)

# Create right frame for graph and error messages
right_frame = tk.Frame(root)
right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=1)

# Label to display error messages
error_message_label = tk.Label(right_frame, text="", fg="red", font=font_style)
error_message_label.pack(side=tk.TOP, pady=5)

# Graph setup
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
canvas = FigureCanvasTkAgg(fig, master=right_frame)
canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=1)

# Function to move all motors
def on_move_all():
    try:
        theta1 = float(theta1_entry.get() or 0)
        theta2 = float(theta2_entry.get() or 0)
        theta3 = float(theta3_entry.get() or 0)
        theta4 = float(theta4_entry.get() or 0)

        theta_values = {
            "theta1": theta1,
            "theta2": theta2,
            "theta3": theta3,
            "theta4": theta4,
        }

        if not validate_all_thetas(theta_values):
            return  # Invalid input, do not proceed

        move_all_motors(theta1, theta2, theta3, theta4)
    except ValueError:
        pass  # Ignore invalid inputs

# Function to open the camera
def open_camera():
    # Open the camera
    cap = cv2.VideoCapture(0)  # Change to 1 if not the primary camera

    # Function to update frames in GUI
    def update_frame():
        ret, frame = cap.read()
        if ret:
            # Convert image from BGR (OpenCV) to RGB (Pillow)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            img_tk = ImageTk.PhotoImage(image=img)

            # Update the image in the Label
            label.config(image=img_tk)
            label.image = img_tk
        else:
            messagebox.showerror("Camera Error", "Failed to capture frame from camera.")
            window.destroy()
            return

        # Call this function again after 10 milliseconds
        label.after(10, update_frame)

    # Function to handle window closing
    def on_closing():
        cap.release()
        window.destroy()

    # Create a new window to display the camera
    window = tk.Toplevel()
    window.title("Camera")

    # Create a Label to display the video frame
    label = tk.Label(window)
    label.pack()

    # Start updating the frames
    update_frame()

    # Set the protocol to release the camera when window is closed
    window.protocol("WM_DELETE_WINDOW", on_closing)

# Create input fields and buttons
tk.Label(left_frame, text="Theta 1", font=font_style).pack(pady=(5, 5))
theta1_entry = tk.Entry(left_frame, font=font_style, justify="center")
theta1_entry.pack(pady=(0, 5))
theta1_entry.bind("<KeyRelease>", lambda event: update_xyz_from_theta())

tk.Label(left_frame, text="Theta 2", font=font_style).pack(pady=(5, 5))
theta2_entry = tk.Entry(left_frame, font=font_style, justify="center")
theta2_entry.pack(pady=(0, 5))
theta2_entry.bind("<KeyRelease>", lambda event: update_xyz_from_theta())

tk.Label(left_frame, text="Theta 3", font=font_style).pack(pady=(5, 5))
theta3_entry = tk.Entry(left_frame, font=font_style, justify="center")
theta3_entry.pack(pady=(0, 5))
theta3_entry.bind("<KeyRelease>", lambda event: update_xyz_from_theta())

tk.Label(left_frame, text="Theta 4", font=font_style).pack(pady=(5, 5))
theta4_entry = tk.Entry(left_frame, font=font_style, justify="center")
theta4_entry.pack(pady=(0, 5))
theta4_entry.bind("<KeyRelease>", lambda event: update_xyz_from_theta())

tk.Button(left_frame, text="Move All Motors", command=on_move_all, font=font_style, height=1, width=25).pack(pady=(1, 1))

# Input fields for x, y, z, a
tk.Label(left_frame, text="X", font=font_style).pack(pady=(5, 5))
x_entry = tk.Entry(left_frame, font=font_style, justify="center")
x_entry.pack(pady=(0, 5))
x_entry.bind("<KeyRelease>", lambda event: update_theta_from_xyz())

tk.Label(left_frame, text="Y", font=font_style).pack(pady=(5, 5))
y_entry = tk.Entry(left_frame, font=font_style, justify="center")
y_entry.pack(pady=(0, 5))
y_entry.bind("<KeyRelease>", lambda event: update_theta_from_xyz())

tk.Label(left_frame, text="Z", font=font_style).pack(pady=(5, 5))
z_entry = tk.Entry(left_frame, font=font_style, justify="center")
z_entry.pack(pady=(0, 5))
z_entry.bind("<KeyRelease>", lambda event: update_theta_from_xyz())

tk.Label(left_frame, text="Angle A", font=font_style).pack(pady=(5, 5))
a_entry = tk.Entry(left_frame, font=font_style, justify="center")
a_entry.pack(pady=(0, 5))
a_entry.bind("<KeyRelease>", lambda event: update_theta_from_xyz())

# Move button for x, y, z, a
tk.Button(left_frame, text="Move to X, Y, Z, A", command=move_to_xyz, font=font_style, height=2, width=25).pack(pady=(1, 1))

# Button to send 'f 0 0 0 0' command
tk.Button(left_frame, text="Move to Zero", command=move_to_zero, font=font_style, height=2, width=25).pack(pady=(1, 1))

# Open camera
tk.Button(left_frame, text="Camera", command=open_camera, font=font_style, height=2, width=25).pack(pady=(1, 1))

# Home button
tk.Button(left_frame, text="Home", command=home_position, font=font_style, height=1, width=25).pack(pady=(0, 1))

# Stop button
tk.Button(left_frame, text="Stop", command=lambda: send_command("s"), font=font_style, height=3, width=25, bg='red', foreground="white").pack(pady=(0, 1))

# Start Tkinter main loop
root.mainloop()
