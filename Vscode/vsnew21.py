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
import numpy as np  # Import numpy

# Constants for robotic arm lengths (in mm)
OA = 122.75
AB = 53.10
BC = 104.05
CD = 194.71
DE = 150.22
EF = 10
alpha = math.radians(45)  # Convert alpha to radians

# Define angle limits for theta (in degrees)
LIMITS = {
    "theta1": (-1, 180),
    "theta2": (-1, 160),
    "theta3": (-120, 154),
    "theta4": (-180, 180),  # Adjusted limits for theta4
}

# Function to normalize angles to -180 to 180 degrees
def normalize_angle(angle):
    """Normalize angle to be within -180 to 180 degrees."""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

# Function to find Arduino devices
def find_arduinos():
    ports = list_ports.comports()
    arduino_ports = []
    for port in ports:
        if "Arduino" in port.description:
            arduino_ports.append((port.device, port.description))
    return arduino_ports

# Function to select Arduino port via a modal Toplevel window
def select_arduino_gui(parent, arduino_ports):
    if not arduino_ports:
        messagebox.showerror("Error", "No Arduino found!", parent=parent)
        return None

    # Create a modal Toplevel window
    top = Toplevel(parent)
    top.title("Select Arduino Port")
    top.grab_set()  # Make the window modal

    selected_port = StringVar(top)
    selected_port.set(f"{arduino_ports[0][0]} - {arduino_ports[0][1]}")  # Default value

    # Dropdown menu
    dropdown = OptionMenu(top, selected_port, *[f"{p[0]} - {p[1]}" for p in arduino_ports])
    dropdown.pack(pady=10, padx=10)

    # Confirm button
    def confirm_selection():
        top.destroy()

    Button(top, text="Confirm", command=confirm_selection).pack(pady=5)

    parent.wait_window(top)  # Wait until the Toplevel window is closed

    for port in arduino_ports:
        if selected_port.get().startswith(port[0]):
            return port[0]
    return None

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
    try:
        arduino.write(command_str.encode())
        time.sleep(0.1)
        if arduino.in_waiting > 0:
            response = arduino.readline().decode().strip()
            print("Arduino:", response)
        messagebox.showinfo("Command Sent", f"Command sent to Arduino: {command_str.strip()}", parent=root)
    except Exception as e:
        messagebox.showerror("Error", f"Failed to send command: {e}", parent=root)

# Function for Forward Kinematics calculation
def calculate_forward_kinematics(theta1, theta2, theta3, theta4):
    # Convert angles to radians
    theta1_rad = math.radians(theta1)
    theta2_rad = math.radians(theta2)
    theta3_rad = math.radians(theta3)
    theta4_rad = math.radians(theta4)

    # Base position
    x0, y0, z0 = 0, 0, OA

    # Position after first link (AB)
    x1 = AB * math.cos(alpha) * math.cos(theta1_rad)
    y1 = AB * math.cos(alpha) * math.sin(theta1_rad)
    z1 = z0 + AB * math.sin(alpha)

    # Position after second link (BC)
    x2 = x1 + BC * math.cos(alpha) * math.cos(theta1_rad)
    y2 = y1 + BC * math.cos(alpha) * math.sin(theta1_rad)
    z2 = z1 + BC * math.sin(alpha)

    # Position after third link (CD)
    x3 = x2 + CD * math.cos(theta2_rad) * math.cos(theta1_rad)
    y3 = y2 + CD * math.cos(theta2_rad) * math.sin(theta1_rad)
    z3 = z2 + CD * math.sin(theta2_rad)

    # Position after fourth link (DE)
    theta23_rad = theta2_rad + theta3_rad
    x4 = x3 + DE * math.cos(theta23_rad) * math.cos(theta1_rad)
    y4 = y3 + DE * math.cos(theta23_rad) * math.sin(theta1_rad)
    z4 = z3 + DE * math.sin(theta23_rad)

    # Position after end effector (EF)
    theta234_rad = theta23_rad + theta4_rad
    x5 = x4 + EF * math.cos(theta234_rad) * math.cos(theta1_rad)
    y5 = y4 + EF * math.cos(theta234_rad) * math.sin(theta1_rad)
    z5 = z4 + EF * math.sin(theta234_rad)

    return [(x0, y0, z0), (x1, y1, z1), (x2, y2, z2),
            (x3, y3, z3), (x4, y4, z4), (x5, y5, z5)]

# Corrected Analytical Inverse Kinematics Function
def analytical_inverse_kinematics(x_target, y_target, z_target, gamma, initial_guess):
    try:
        # Calculate theta1
        theta1 = math.degrees(math.atan2(y_target, x_target))
        theta1 = normalize_angle(theta1)

        # Offsets due to initial links
        x_fixed = (AB + BC) * math.cos(alpha) * math.cos(math.radians(theta1))
        y_fixed = (AB + BC) * math.cos(alpha) * math.sin(math.radians(theta1))
        z_fixed = OA + (AB + BC) * math.sin(alpha)

        # Compute wrist position
        x_wrist = x_target - EF * math.cos(math.radians(gamma)) * math.cos(math.radians(theta1))
        y_wrist = y_target - EF * math.cos(math.radians(gamma)) * math.sin(math.radians(theta1))
        z_wrist = z_target - EF * math.sin(math.radians(gamma))

        # Compute delta from fixed links
        delta_x = x_wrist - x_fixed
        delta_y = y_wrist - y_fixed
        delta_z = z_wrist - z_fixed

        # Planar distance and vertical distance
        r = math.sqrt(delta_x**2 + delta_y**2)
        s = delta_z

        # Lengths of the arms
        L1 = CD
        L2 = DE

        # Compute D using the Law of Cosines
        D = (r**2 + s**2 - L1**2 - L2**2) / (2 * L1 * L2)
        # Implement tolerance to account for floating-point errors
        if D < -1.0001 or D > 1.0001:
            messagebox.showerror("Error", "Position unreachable.", parent=root)
            return None
        # Clamp D within [-1, 1]
        D = max(min(D, 1), -1)

        # Two possible solutions for theta3
        theta3_options = [math.degrees(math.acos(D)), -math.degrees(math.acos(D))]

        solutions = []
        for theta3 in theta3_options:
            # Compute theta2 based on theta3
            k1 = L1 + L2 * math.cos(math.radians(theta3))
            k2 = L2 * math.sin(math.radians(theta3))
            phi = math.degrees(math.atan2(s, r))
            theta2 = phi + math.degrees(math.atan2(k2, k1))  # Corrected atan2 arguments

            # Compute theta4
            theta4 = gamma - theta2 - theta3

            # Normalize angles
            theta1_norm = normalize_angle(theta1)
            theta2_norm = normalize_angle(theta2)
            theta3_norm = normalize_angle(theta3)
            theta4_norm = normalize_angle(theta4)

            solution = (theta1_norm, theta2_norm, theta3_norm, theta4_norm)
            solutions.append(solution)

            # Debugging: Print each solution
            print(f"Solution: Theta1={theta1_norm}, Theta2={theta2_norm}, Theta3={theta3_norm}, Theta4={theta4_norm}")

        if not solutions:
            messagebox.showerror("Error", "No valid solutions found.", parent=root)
            return None

        # Select the solution closest to the initial guess
        initial_guess = [float(angle) for angle in initial_guess]
        min_error = float('inf')
        best_solution = None
        for sol in solutions:
            error = sum((sol_i - guess_i)**2 for sol_i, guess_i in zip(sol, initial_guess))
            print(f"Evaluating solution {sol} with error {error}")
            if error < min_error:
                min_error = error
                best_solution = sol

        return best_solution
    except Exception as e:
        messagebox.showerror("Error", f"Invalid target position: {e}", parent=root)
        return None

# Function to update x, y, z, a using Forward Kinematics
def update_xyz_from_theta(event=None):
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
def update_theta_from_xyz(event=None):
    try:
        x_target = float(x_entry.get() or 0)
        y_target = float(y_entry.get() or 0)
        z_target = float(z_entry.get() or 0)
        gamma = float(a_entry.get() or 0)

        # Initial guess for selecting the best solution
        initial_guess = [
            float(theta1_entry.get() or 0),
            float(theta2_entry.get() or 0),
            float(theta3_entry.get() or 0),
            float(theta4_entry.get() or 0),
        ]

        # Run the analytical inverse kinematics
        result = analytical_inverse_kinematics(x_target, y_target, z_target, gamma, initial_guess)
        if result:
            theta1, theta2, theta3, theta4 = result

            theta_values = {
                "theta1": theta1,
                "theta2": theta2,
                "theta3": theta3,
                "theta4": theta4,
            }

            if not validate_all_thetas(theta_values):
                return  # Invalid input, do not proceed

            theta1_entry.delete(0, tk.END)
            theta1_entry.insert(0, f"{theta1:.5f}")
            theta2_entry.delete(0, tk.END)
            theta2_entry.insert(0, f"{theta2:.5f}")
            theta3_entry.delete(0, tk.END)
            theta3_entry.insert(0, f"{theta3:.5f}")
            theta4_entry.delete(0, tk.END)
            theta4_entry.insert(0, f"{theta4:.5f}")

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

        # Initial guess for selecting the best solution
        initial_guess = [
            float(theta1_entry.get() or 0),
            float(theta2_entry.get() or 0),
            float(theta3_entry.get() or 0),
            float(theta4_entry.get() or 0),
        ]

        # Run the analytical inverse kinematics
        result = analytical_inverse_kinematics(x_target, y_target, z_target, gamma, initial_guess)
        if result:
            theta1, theta2, theta3, theta4 = result

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
    ax.set_xlim(-500, 500)
    ax.set_ylim(-500, 500)
    ax.set_zlim(0, 600)
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

# Create left frame for buttons and inputs
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

# Initialize Arduino connection
arduino_ports = find_arduinos()
if not arduino_ports:
    messagebox.showerror("Error", "No Arduino ports found! Please connect Arduino and try again.", parent=root)
    root.destroy()
    exit()

# Let the user select a port
arduino_port = select_arduino_gui(root, arduino_ports)
if not arduino_port:
    messagebox.showinfo("Cancel", "You did not select an Arduino port.", parent=root)
    root.destroy()
    exit()

# Set up Serial connection
try:
    arduino = serial.Serial(arduino_port, 9600, timeout=1)
    messagebox.showinfo("Success", f"Connected to Arduino at port {arduino_port}", parent=root)
except Exception as e:
    messagebox.showerror("Error", f"Could not connect to Arduino: {e}", parent=root)
    root.destroy()
    exit()

# Function to plot the robotic arm on startup
def initialize_plot():
    plot_robot_arm(0, 90, 0, 0)  # Home position

initialize_plot()

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
            messagebox.showerror("Camera Error", "Failed to capture frame from camera.", parent=window)
            window.destroy()
            return

        # Call this function again after 10 milliseconds
        label.after(10, update_frame)

    # Function to handle window closing
    def on_closing():
        cap.release()
        window.destroy()

    # Create a new window to display the camera
    window = tk.Toplevel(root)
    window.title("Camera")

    # Create a Label to display the video frame
    label = tk.Label(window)
    label.pack()

    # Start updating the frames
    update_frame()

    # Set the protocol to release the camera when window is closed
    window.protocol("WM_DELETE_WINDOW", on_closing)

# Create input fields and buttons
# Joint Angles
tk.Label(left_frame, text="Theta 1 (°)", font=font_style).pack(pady=(5, 5))
theta1_entry = tk.Entry(left_frame, font=font_style, justify="center")
theta1_entry.pack(pady=(0, 5))
theta1_entry.bind("<KeyRelease>", update_xyz_from_theta)

tk.Label(left_frame, text="Theta 2 (°)", font=font_style).pack(pady=(5, 5))
theta2_entry = tk.Entry(left_frame, font=font_style, justify="center")
theta2_entry.pack(pady=(0, 5))
theta2_entry.bind("<KeyRelease>", update_xyz_from_theta)

tk.Label(left_frame, text="Theta 3 (°)", font=font_style).pack(pady=(5, 5))
theta3_entry = tk.Entry(left_frame, font=font_style, justify="center")
theta3_entry.pack(pady=(0, 5))
theta3_entry.bind("<KeyRelease>", update_xyz_from_theta)

tk.Label(left_frame, text="Theta 4 (°)", font=font_style).pack(pady=(5, 5))
theta4_entry = tk.Entry(left_frame, font=font_style, justify="center")
theta4_entry.pack(pady=(0, 5))
theta4_entry.bind("<KeyRelease>", update_xyz_from_theta)

tk.Button(left_frame, text="Move All Motors", command=on_move_all, font=font_style, height=1, width=25).pack(pady=(10, 10))

# Position Inputs
tk.Label(left_frame, text="X (mm)", font=font_style).pack(pady=(5, 5))
x_entry = tk.Entry(left_frame, font=font_style, justify="center")
x_entry.pack(pady=(0, 5))
x_entry.bind("<KeyRelease>", update_theta_from_xyz)

tk.Label(left_frame, text="Y (mm)", font=font_style).pack(pady=(5, 5))
y_entry = tk.Entry(left_frame, font=font_style, justify="center")
y_entry.pack(pady=(0, 5))
y_entry.bind("<KeyRelease>", update_theta_from_xyz)

tk.Label(left_frame, text="Z (mm)", font=font_style).pack(pady=(5, 5))
z_entry = tk.Entry(left_frame, font=font_style, justify="center")
z_entry.pack(pady=(0, 5))
z_entry.bind("<KeyRelease>", update_theta_from_xyz)

tk.Label(left_frame, text="Angle A (°)", font=font_style).pack(pady=(5, 5))
a_entry = tk.Entry(left_frame, font=font_style, justify="center")
a_entry.pack(pady=(0, 5))
a_entry.bind("<KeyRelease>", update_theta_from_xyz)

# Move button for position inputs
tk.Button(left_frame, text="Move to X, Y, Z, A", command=move_to_xyz, font=font_style, height=2, width=25).pack(pady=(10, 10))

# Move to Zero button
tk.Button(left_frame, text="Move to Zero", command=move_to_zero, font=font_style, height=2, width=25).pack(pady=(0, 10))

# Open Camera button
tk.Button(left_frame, text="Camera", command=open_camera, font=font_style, height=2, width=25).pack(pady=(0, 10))

# Home button
tk.Button(left_frame, text="Home", command=home_position, font=font_style, height=1, width=25).pack(pady=(0, 10))

# Stop button
tk.Button(left_frame, text="Stop", command=lambda: send_command("s"), font=font_style, height=3, width=25, bg='red', fg="white").pack(pady=(0, 10))

# Start Tkinter main loop
root.mainloop()
