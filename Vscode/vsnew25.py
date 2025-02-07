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
import numpy as np
from scipy.optimize import minimize

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
    time.sleep(2)  # Wait for Arduino to be ready
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
    "theta1": (-1, 181),
    "theta2": (-1, 161),
    "theta3": (-121, 155),
    "theta4": (-180, 180),
}

# Function to validate angles and display messages
def validate_all_thetas(theta_values):
    invalid_thetas = []
    for angle_name, value in theta_values.items():
        min_val, max_val = LIMITS[angle_name]
        if value < min_val or value > max_val:
            invalid_thetas.append(angle_name)
    if invalid_thetas:
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

# Function for Forward Kinematics calculation
def calculate_forward_kinematics(theta1, theta2, theta3, theta4):
    # Convert angles to radians
    theta1_rad = math.radians(theta1)
    theta2_rad = math.radians(theta2)
    theta3_rad = math.radians(theta3)
    theta4_rad = math.radians(theta4)

    xA, yA, zA = (0, 0, OA)

    xB = AB * math.cos(alpha) * math.cos(theta1_rad)
    yB = AB * math.cos(alpha) * math.sin(theta1_rad)
    zB = OA + AB * math.sin(alpha)

    xC = xB + BC * math.cos(alpha) * math.cos(theta1_rad)
    yC = yB + BC * math.cos(alpha) * math.sin(theta1_rad)
    zC = zB + BC * math.sin(alpha)

    xD = xC + CD * math.cos(theta2_rad) * math.cos(theta1_rad)
    yD = yC + CD * math.cos(theta2_rad) * math.sin(theta1_rad)
    zD = zC + CD * math.sin(theta2_rad)

    xE = xD + DE * math.cos(theta2_rad + theta3_rad) * math.cos(theta1_rad)
    yE = yD + DE * math.cos(theta2_rad + theta3_rad) * math.sin(theta1_rad)
    zE = zD + DE * math.sin(theta2_rad + theta3_rad)

    xF = xE + EF * math.cos(theta2_rad + theta3_rad + theta4_rad) * math.cos(theta1_rad)
    yF = yE + EF * math.cos(theta2_rad + theta3_rad + theta4_rad) * math.sin(theta1_rad)
    zF = zE + EF * math.sin(theta2_rad + theta3_rad + theta4_rad)

    # Shift z values so that base is at z = 0
    zA -= OA
    zB -= OA
    zC -= OA
    zD -= OA
    zE -= OA
    zF -= OA

    return [(0, 0, zA), (xB, yB, zB), (xC, yC, zC), (xD, yD, zD), (xE, yE, zE), (xF, yF, zF)]

# Inverse Kinematics using optimization
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
        LIMITS['theta4'],
    ]

    # Run optimization
    result = minimize(
        objective,
        initial_guess,
        bounds=bounds,
        method='SLSQP',
        options={'ftol': 1e-8, 'disp': False}
    )

    if result.success:
        theta1, theta2, theta3, theta4 = result.x
        return theta1, theta2, theta3, theta4
    else:
        error_message_label.config(text="Inverse kinematics optimization failed.")
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

    # Calculate joint positions
    points = calculate_forward_kinematics(theta1, theta2, theta3, theta4)

    # Plot the arm segments between joints
    for i in range(len(points)-1):
        x_points = [points[i][0], points[i+1][0]]
        y_points = [points[i][1], points[i+1][1]]
        z_points = [points[i][2], points[i+1][2]]
        ax.plot(x_points, y_points, z_points, color='blue', linewidth=4)

    # Plot the joints
    x_coords = [p[0] for p in points]
    y_coords = [p[1] for p in points]
    z_coords = [p[2] for p in points]
    ax.scatter(x_coords, y_coords, z_coords, color='red', s=100)

    # Set labels and title
    ax.set_xlabel('X-axis (mm)', labelpad=20)
    ax.set_ylabel('Y-axis (mm)', labelpad=20)
    ax.set_zlabel('Z-axis (mm)', labelpad=20)
    ax.set_title("3D Plot of Robotic Arm Position", pad=30)

    # Set fixed axis limits
    ax.set_xlim([-450, 450])
    ax.set_ylim([-450, 450])
    ax.set_zlim([0, 450])  # Start from z = 0

    # Set equal aspect ratio
    ax.set_box_aspect([1,1,1])

    # Adjust the viewing angle
    ax.view_init(elev=30, azim=60)

    canvas.draw()  # Update the graph

# Home function
def home_position():
    send_command("h")  # Send 'h' command to Arduino
    # Set home position by setting all angles to 0
    theta1, theta2, theta3, theta4 = 0, 0, 0, 0
    # Update GUI entries to match new angles
    theta1_entry.delete(0, tk.END)
    theta1_entry.insert(0, f"{theta1}")
    theta2_entry.delete(0, tk.END)
    theta2_entry.insert(0, f"{theta2}")
    theta3_entry.delete(0, tk.END)
    theta3_entry.insert(0, f"{theta3}")
    theta4_entry.delete(0, tk.END)
    theta4_entry.insert(0, f"{theta4}")
    # Update x, y, z, a using Forward Kinematics
    update_xyz_from_theta()
    # Plot the robotic arm at the new position
    plot_robot_arm(theta1, theta2, theta3, theta4)

# Function to send 'f 0 0 0 0' command
def move_to_zero():
    send_command("f 0 0 0 0")
    # Set angles to 0
    theta1, theta2, theta3, theta4 = 0, 0, 0, 0
    # Update GUI entries
    theta1_entry.delete(0, tk.END)
    theta1_entry.insert(0, f"{theta1}")
    theta2_entry.delete(0, tk.END)
    theta2_entry.insert(0, f"{theta2}")
    theta3_entry.delete(0, tk.END)
    theta3_entry.insert(0, f"{theta3}")
    theta4_entry.delete(0, tk.END)
    theta4_entry.insert(0, f"{theta4}")
    # Update x, y, z, a
    update_xyz_from_theta()
    # Plot the robotic arm at zero position
    plot_robot_arm(theta1, theta2, theta3, theta4)

# **New functions to handle the new buttons**
def activate_code1():
    send_command("code1activate")

def activate_code2():
    send_command("code2activate")

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
fig = plt.figure(figsize=(12, 9))  # Increased figure size
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

# Variables to manage camera state
camera_window = None
cap = None
camera_running = False  # Flag to control the camera loop

# Calibration parameter (pixels per cm)
pixels_per_cm = 37  # Adjust based on calibration

# Function to send commands to robot
def send_command_to_robot(command):
    send_command(command)

# Function to detect red circle
def detect_red_circle(frame):
    blurred = cv2.GaussianBlur(frame, (5, 5), 2)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Red color filter in HSV
    lower_red_1 = np.array([0, 100, 50])
    upper_red_1 = np.array([10, 255, 255])
    lower_red_2 = np.array([170, 100, 50])
    upper_red_2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    mask_red = cv2.add(mask1, mask2)

    return detect_object(frame, mask_red, is_circle=True)

# Function to detect blue rectangle
def detect_blue_rectangle(frame):
    blurred = cv2.GaussianBlur(frame, (5, 5), 2)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Blue color filter in HSV
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([140, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # Use Morphological Transformations
    kernel = np.ones((5, 5), np.uint8)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

    return detect_object(frame, mask_blue, is_circle=False)

# General object detection function
def detect_object(frame, mask, is_circle):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detected_object = None
    for contour in contours:
        if cv2.contourArea(contour) > 500:  # Filter by size
            x, y, w, h = cv2.boundingRect(contour)

            if is_circle and len(contour) >= 5:
                (x_center, y_center), radius = cv2.minEnclosingCircle(contour)
                diameter_cm = (2 * radius) / pixels_per_cm  # Convert to cm
                cv2.circle(frame, (int(x_center), int(y_center)), int(radius), (0, 255, 0), 2)
                cv2.putText(frame, f"Diameter: {diameter_cm:.2f} cm", (int(x_center), int(y_center) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                detected_object = (int(x_center), int(y_center))
            else:
                # Check aspect ratio of rectangle
                aspect_ratio = w / float(h)
                if 0.8 <= aspect_ratio <= 1.2:  # Approximate square
                    width_cm = w / pixels_per_cm
                    height_cm = h / pixels_per_cm
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    cv2.putText(frame, f"W: {width_cm:.2f}cm H: {height_cm:.2f}cm",
                                (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    detected_object = (x, y, w, h)

    return frame, detected_object

# Function to open the camera and start object detection
def open_camera():
    global camera_window, cap, camera_running
    if camera_window is not None:
        error_message_label.config(text="Camera is already open.")
        return

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        error_message_label.config(text="Failed to open camera.")
        return

    camera_running = True  # Set the flag to True

    # Create a new window to display the camera
    camera_window = tk.Toplevel()
    camera_window.title("Camera")

    # Create a Label to display the video frame
    label = tk.Label(camera_window)
    label.pack()

    # Function to handle window closing
    def on_closing():
        close_camera()

    # Set the protocol to release the camera when window is closed
    camera_window.protocol("WM_DELETE_WINDOW", on_closing)

    # Enable the "Close Camera" button
    close_camera_button.config(state=tk.NORMAL)

    # Start updating frames
    run_camera(label)

# Updated run_camera function without threading
def run_camera(label):
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    center_x = frame_width // 2

    def update_frame():
        if not camera_running:
            return  # Exit if camera is not running

        ret, frame = cap.read()
        if not ret:
            error_message_label.config(text="Failed to capture frame from camera.")
            close_camera()
            return

        # Detect red circle
        frame, red_center = detect_red_circle(frame)

        # Detect blue rectangle
        frame, blue_object = detect_blue_rectangle(frame)

        # Handle red circle
        if red_center:
            print(f"Red Circle detected at: {red_center}")
            x_diff = center_x - red_center[0]
            if abs(x_diff) > 20:
                send_command_to_robot(f"MOVE_RED {x_diff}")
            else:
                send_command_to_robot("STOP_RED")

        # Handle blue rectangle
        if blue_object:
            print(f"Blue Rectangle detected at: {blue_object}")
            x_diff = center_x - (blue_object[0] + blue_object[2] // 2)
            if abs(x_diff) > 20:
                send_command_to_robot(f"MOVE_BLUE {x_diff}")
            else:
                send_command_to_robot("STOP_BLUE")

        # Convert image from BGR to RGB for Tkinter display
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        img_tk = ImageTk.PhotoImage(image=img)

        # Update the image in the Label
        label.config(image=img_tk)
        label.image = img_tk

        # Schedule the next frame update
        if camera_running:
            label.after(10, update_frame)

    # Start updating frames
    update_frame()

# Function to close the camera
def close_camera():
    global camera_window, cap, camera_running
    if camera_window is not None:
        camera_running = False  # Stop the update loop
        camera_window.destroy()
        camera_window = None
    if cap is not None:
        cap.release()
        cap = None
    close_camera_button.config(state=tk.DISABLED)

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

tk.Button(left_frame, text="Move Kinematics", command=on_move_all, font=font_style, height=1, width=25).pack(pady=(1, 1))

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
tk.Button(left_frame, text="Move Inverse", command=move_to_xyz, font=font_style, height=2, width=25).pack(pady=(1, 1))

# Button to send 'f 0 0 0 0' command
tk.Button(left_frame, text="Move to Zero point", command=move_to_zero, font=font_style, height=2, width=25).pack(pady=(1, 1))

# Open camera button
tk.Button(left_frame, text="Camera", command=open_camera, font=font_style, height=2, width=25).pack(pady=(1, 1))

# Close camera button
close_camera_button = tk.Button(left_frame, text="Close Camera", command=close_camera, font=font_style, height=2, width=25)
close_camera_button.pack(pady=(1, 1))
close_camera_button.config(state=tk.DISABLED)

# **New buttons for activating codes**
tk.Button(left_frame, text="Activate Code 1", command=activate_code1, font=font_style, height=2, width=25).pack(pady=(1, 1))
tk.Button(left_frame, text="Activate Code 2", command=activate_code2, font=font_style, height=2, width=25).pack(pady=(1, 1))

# Home button
tk.Button(left_frame, text="Home", command=home_position, font=font_style, height=1, width=25).pack(pady=(0, 1))

# Stop button
tk.Button(left_frame, text="Stop", command=lambda: send_command("s"), font=font_style, height=3, width=25, bg='red', foreground="white").pack(pady=(0, 1))

# Start Tkinter main loop
root.mainloop()
