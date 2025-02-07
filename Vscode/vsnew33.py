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
import threading
import logging
import sys
import queue  # Import for thread-safe queue

# Initialize logging
logging.basicConfig(filename='robotic_arm.log', level=logging.INFO,
                    format='%(asctime)s:%(levelname)s:%(message)s')


class ArduinoController:
    def __init__(self, baudrate=9600, timeout=1):
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None

    def find_arduinos(self):
        ports = list_ports.comports()
        arduino_ports = []
        for port in ports:
            if "Arduino" in port.description or "ttyACM" in port.device or "ttyUSB" in port.device:
                arduino_ports.append((port.device, port.description))
        return arduino_ports

    def connect(self, port):
        try:
            self.serial = serial.Serial(port, self.baudrate, timeout=self.timeout)
            time.sleep(2)  # Wait for Arduino to initialize
            messagebox.showinfo("Success", f"Connected to Arduino at port {port}")
            logging.info(f"Connected to Arduino at port {port}")
        except Exception as e:
            messagebox.showerror("Error", f"Could not connect to Arduino: {e}")
            logging.error(f"Failed to connect to Arduino: {e}")
            sys.exit()

    def send_command(self, command_str):
        try:
            if self.serial and self.serial.is_open:
                full_command = f"{command_str}\n"
                self.serial.write(bytes(full_command, 'utf-8'))
                logging.info(f"Sent command: {command_str}")
                time.sleep(0.1)
                if self.serial.in_waiting > 0:
                    response = self.serial.readline().decode().strip()
                    logging.info(f"Received response: {response}")
                    print("Arduino:", response)
                    return response
            else:
                logging.error("Serial port is not open.")
                messagebox.showerror("Error", "Serial port is not open.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send command: {e}")
            logging.error(f"Failed to send command '{command_str}': {e}")

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            logging.info("Serial connection closed.")


class Kinematics:
    def __init__(self):
        # Constants for robotic arm lengths (in millimeters)
        self.OA = 122.75
        self.AB = 53.10
        self.BC = 104.05
        self.CD = 194.71
        self.DE = 150.22
        self.EF = 10
        self.alpha = math.radians(45)

        # Define angle limits for theta
        self.LIMITS = {
            "theta1": (0, 180),
            "theta2": (0, 160),
            "theta3": (-150, 150),
            "theta4": (-120, 120),
        }

    def calculate_forward_kinematics(self, theta1, theta2, theta3, theta4):
        # Convert angles to radians
        theta1_rad = math.radians(theta1)
        theta2_rad = math.radians(theta2)
        theta3_rad = math.radians(theta3)
        theta4_rad = math.radians(theta4)

        xA, yA, zA = (0, 0, self.OA)

        xB = self.AB * math.cos(self.alpha) * math.cos(theta1_rad)
        yB = self.AB * math.cos(self.alpha) * math.sin(theta1_rad)
        zB = self.OA + self.AB * math.sin(self.alpha)

        xC = xB + self.BC * math.cos(self.alpha) * math.cos(theta1_rad)
        yC = yB + self.BC * math.cos(self.alpha) * math.sin(theta1_rad)
        zC = zB + self.BC * math.sin(self.alpha)

        xD = xC + self.CD * math.cos(theta2_rad) * math.cos(theta1_rad)
        yD = yC + self.CD * math.cos(theta2_rad) * math.sin(theta1_rad)
        zD = zC + self.CD * math.sin(theta2_rad)

        xE = xD + self.DE * math.cos(theta2_rad + theta3_rad) * math.cos(theta1_rad)
        yE = yD + self.DE * math.cos(theta2_rad + theta3_rad) * math.sin(theta1_rad)
        zE = zD + self.DE * math.sin(theta2_rad + theta3_rad)

        xF = xE + self.EF * math.cos(theta2_rad + theta3_rad + theta4_rad) * math.cos(theta1_rad)
        yF = yE + self.EF * math.cos(theta2_rad + theta3_rad + theta4_rad) * math.sin(theta1_rad)
        zF = zE + self.EF * math.sin(theta2_rad + theta3_rad + theta4_rad)

        # Shift z values so that base is at z = 0
        zA -= self.OA
        zB -= self.OA
        zC -= self.OA
        zD -= self.OA
        zE -= self.OA
        zF -= self.OA

        return [(0, 0, zA), (xB, yB, zB), (xC, yC, zC),
                (xD, yD, zD), (xE, yE, zE), (xF, yF, zF)]

    def inverse_kinematics_optimization(self, x_target, y_target, z_target, gamma, initial_guess):
        # Objective function to minimize
        def objective(thetas):
            theta1, theta2, theta3, theta4 = thetas
            # Calculate forward kinematics
            points = self.calculate_forward_kinematics(theta1, theta2, theta3, theta4)
            xF, yF, zF = points[-1]
            gamma_calc = theta2 + theta3 + theta4
            # Position error
            pos_error = ((xF - x_target) ** 2 +
                         (yF - y_target) ** 2 +
                         (zF - z_target) ** 2)
            # Orientation error
            ori_error = (gamma_calc - gamma) ** 2
            # Total error
            total_error = pos_error + ori_error
            return total_error

        # Constraints for joint limits
        bounds = [
            self.LIMITS['theta1'],
            self.LIMITS['theta2'],
            self.LIMITS['theta3'],
            self.LIMITS['theta4'],
        ]

        # Run optimization
        try:
            result = minimize(
                objective,
                initial_guess,
                bounds=bounds,
                method='SLSQP',
                options={'ftol': 1e-8, 'disp': False}
            )
        except Exception as e:
            logging.error(f"Optimization exception: {e}")
            return None

        if result.success:
            theta1, theta2, theta3, theta4 = result.x
            return theta1, theta2, theta3, theta4
        else:
            logging.warning(f"Optimization failed: {result.message}")
            return None


class RoboticArmGUI:
    def __init__(self, root, arduino, kinematics):
        self.root = root
        self.arduino = arduino
        self.kinematics = kinematics
        self.camera_window = None
        self.cap = None
        self.camera_running = False  # Flag to control the camera loop
        self.PIXELS_PER_CM = 20.00  # Updated calibration: pixels per centimeter
        self.CAMERA_FOV = 90  # Updated Horizontal field of view in degrees
        self.THETA_THRESHOLD = 2  # Threshold in degrees for stopping
        self.last_command_time = 0
        self.command_interval = 0.1  # seconds
        self.tracking_enabled = False  # Tracking control variable

        # New attributes for smoother camera operation
        self.frame_queue = queue.Queue(maxsize=10)
        self.stop_event = threading.Event()

        self.stopped = False  # Flag to track if robot arm is stopped
        self.missing_frames = 0  # Counter for consecutive missing frames
        self.max_missing_frames = 5  # Threshold to send stop command

        self.setup_ui()

    def setup_ui(self):
        self.root.title("Robotic Arm Kinematics")
        font_style = ('Helvetica', 14)

        # Create frames
        self.left_frame = tk.Frame(self.root)
        self.left_frame.pack(side=tk.LEFT, padx=10, pady=10)

        self.right_frame = tk.Frame(self.root)
        self.right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=1)

        # Error message label
        self.error_message_label = tk.Label(self.right_frame, text="", fg="red", font=font_style)
        self.error_message_label.pack(side=tk.TOP, pady=5)

        # Matplotlib Figure
        self.fig = plt.figure(figsize=(12, 9))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.right_frame)
        self.canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=1)

        # Input fields and buttons
        self.create_input_fields(font_style)
        self.create_buttons(font_style)

        # Initialize plot
        self.plot_robot_arm(0, 135, -90, -45)

    def create_input_fields(self, font_style):
        # Theta entries
        self.theta_entries = {}
        for i, theta in enumerate(["Theta 1", "Theta 2", "Theta 3", "Theta 4"], start=1):
            tk.Label(self.left_frame, text=theta, font=font_style).pack(pady=(5, 5))
            entry = tk.Entry(self.left_frame, font=font_style, justify="center")
            entry.pack(pady=(0, 5))
            entry.bind("<KeyRelease>", lambda event, t=theta.lower().replace(" ", ""): self.update_xyz_from_theta())
            self.theta_entries[theta.lower().replace(" ", "")] = entry

        # Button "Move Kinematics" placed above X field
        tk.Button(self.left_frame, text="Move Kinematics", command=self.on_move_all,
                  font=font_style, height=1, width=25).pack(pady=(1, 1))

        # X, Y, Z, A entries
        for param in ["X", "Y", "Z", "Angle A"]:
            tk.Label(self.left_frame, text=param, font=font_style).pack(pady=(5, 5))
            entry = tk.Entry(self.left_frame, font=font_style, justify="center")
            entry.pack(pady=(0, 5))
            entry.bind("<KeyRelease>", lambda event, p=param.lower().replace(" ", ""): self.update_theta_from_xyz())
            self.theta_entries[param.lower().replace(" ", "")] = entry

    def create_buttons(self, font_style):
        # Move Inverse Button
        tk.Button(self.left_frame, text="Move Inverse", command=self.move_to_xyz,
                  font=font_style, height=1, width=25).pack(pady=(1, 1))

        # Move to Zero Button
        tk.Button(self.left_frame, text="Move to Vision position", command=self.move_to_zero,
                  font=font_style, height=2, width=25).pack(pady=(1, 1))

        # Camera Buttons
        tk.Button(self.left_frame, text="Camera", command=self.open_camera,
                  font=font_style, height=2, width=25).pack(pady=(1, 1))
        self.close_camera_button = tk.Button(self.left_frame, text="Close Camera",
                                             command=self.close_camera, font=font_style,
                                             height=2, width=25, state=tk.DISABLED)
        self.close_camera_button.pack(pady=(1, 1))

        # Add Start Track and Stop Track buttons
        self.start_track_button = tk.Button(self.left_frame, text="Start Track",
                                            command=self.start_tracking, font=font_style,
                                            height=2, width=25)
        self.start_track_button.pack(pady=(1, 1))

        self.stop_track_button = tk.Button(self.left_frame, text="Stop Track",
                                           command=self.stop_tracking, font=font_style,
                                           height=2, width=25, state=tk.DISABLED)
        self.stop_track_button.pack(pady=(1, 1))

        # Home and Stop Buttons
        tk.Button(self.left_frame, text="Home", command=self.home_position,
                  font=font_style, height=1, width=25).pack(pady=(0, 1))
        tk.Button(self.left_frame, text="Stop", command=lambda: self.arduino.send_command("s"),
                  font=font_style, height=3, width=25, bg='red', fg="white").pack(pady=(0, 1))

    def start_tracking(self):
        self.tracking_enabled = True
        self.start_track_button.config(state=tk.DISABLED)
        self.stop_track_button.config(state=tk.NORMAL)
        print("Tracking started.")

    def stop_tracking(self):
        self.tracking_enabled = False
        self.start_track_button.config(state=tk.NORMAL)
        self.stop_track_button.config(state=tk.DISABLED)
        self.arduino.send_command("s")  # Stop the robot arm
        print("Tracking stopped.")

    def on_move_all(self):
        try:
            theta1 = float(self.theta_entries["theta1"].get() or 0)
            theta2 = float(self.theta_entries["theta2"].get() or 0)
            theta3 = float(self.theta_entries["theta3"].get() or 0)
            theta4 = float(self.theta_entries["theta4"].get() or 0)

            theta_values = {
                "theta1": theta1,
                "theta2": theta2,
                "theta3": theta3,
                "theta4": theta4,
            }

            if not self.validate_all_thetas(theta_values):
                return  # Invalid input

            self.move_all_motors(theta1, theta2, theta3, theta4)
        except ValueError:
            self.error_message_label.config(text="Invalid input in theta fields.")

    def validate_all_thetas(self, theta_values):
        invalid_thetas = []
        for angle_name, value in theta_values.items():
            min_val, max_val = self.kinematics.LIMITS.get(angle_name, (-math.inf, math.inf))
            if value < min_val or value > max_val:
                invalid_thetas.append(angle_name)
        if invalid_thetas:
            if invalid_thetas == ['theta4']:
                self.error_message_label.config(text="Warning: Theta 4 exceeds the specified limits")
                return True
            else:
                self.error_message_label.config(text="Cannot set angle beyond limits in " + ", ".join(invalid_thetas))
                return False
        else:
            self.error_message_label.config(text="")
            return True

    def move_all_motors(self, theta1, theta2, theta3, theta4):
        command_str = f"f {int(round(theta1))} {int(round(theta2))} {int(round(theta3))} {int(round(theta4))}"
        self.arduino.send_command(command_str)
        # Update GUI entries and internal state
        self.theta_entries["theta1"].delete(0, tk.END)
        self.theta_entries["theta1"].insert(0, f"{theta1:.2f}")
        self.theta_entries["theta2"].delete(0, tk.END)
        self.theta_entries["theta2"].insert(0, f"{theta2:.2f}")
        self.theta_entries["theta3"].delete(0, tk.END)
        self.theta_entries["theta3"].insert(0, f"{theta3:.2f}")
        self.theta_entries["theta4"].delete(0, tk.END)
        self.theta_entries["theta4"].insert(0, f"{theta4:.2f}")
        self.update_xyz_from_theta()

    def update_xyz_from_theta(self):
        try:
            theta1 = float(self.theta_entries["theta1"].get() or 0)
            theta2 = float(self.theta_entries["theta2"].get() or 0)
            theta3 = float(self.theta_entries["theta3"].get() or 0)
            theta4 = float(self.theta_entries["theta4"].get() or 0)

            theta_values = {
                "theta1": theta1,
                "theta2": theta2,
                "theta3": theta3,
                "theta4": theta4,
            }

            if not self.validate_all_thetas(theta_values):
                return  # Invalid input

            points = self.kinematics.calculate_forward_kinematics(theta1, theta2, theta3, theta4)
            xF, yF, zF = points[-1]
            gamma = theta2 + theta3 + theta4

            self.theta_entries["x"].delete(0, tk.END)
            self.theta_entries["x"].insert(0, f"{xF/10:.2f}")  # Convert mm to cm
            self.theta_entries["y"].delete(0, tk.END)
            self.theta_entries["y"].insert(0, f"{yF/10:.2f}")  # Convert mm to cm
            self.theta_entries["z"].delete(0, tk.END)
            self.theta_entries["z"].insert(0, f"{zF/10:.2f}")  # Convert mm to cm
            self.theta_entries["anglea"].delete(0, tk.END)
            self.theta_entries["anglea"].insert(0, f"{gamma:.2f}")

            self.plot_robot_arm(theta1, theta2, theta3, theta4)
        except ValueError:
            self.error_message_label.config(text="Invalid numerical input.")

    def update_theta_from_xyz(self):
        try:
            x_target = float(self.theta_entries["x"].get() or 0) * 10  # Convert cm to mm
            y_target = float(self.theta_entries["y"].get() or 0) * 10  # Convert cm to mm
            z_target = float(self.theta_entries["z"].get() or 0) * 10  # Convert cm to mm
            gamma = float(self.theta_entries["anglea"].get() or 0)

            initial_guess = [
                float(self.theta_entries["theta1"].get() or 0),
                float(self.theta_entries["theta2"].get() or 0),
                float(self.theta_entries["theta3"].get() or 0),
                float(self.theta_entries["theta4"].get() or 0),
            ]

            results = self.kinematics.inverse_kinematics_optimization(x_target, y_target, z_target, gamma, initial_guess)
            if results:
                theta1, theta2, theta3, theta4 = results

                theta_values = {
                    "theta1": theta1,
                    "theta2": theta2,
                    "theta3": theta3,
                    "theta4": theta4,
                }

                if not self.validate_all_thetas(theta_values):
                    return  # Invalid input

                self.theta_entries["theta1"].delete(0, tk.END)
                self.theta_entries["theta1"].insert(0, f"{theta1:.2f}")
                self.theta_entries["theta2"].delete(0, tk.END)
                self.theta_entries["theta2"].insert(0, f"{theta2:.2f}")
                self.theta_entries["theta3"].delete(0, tk.END)
                self.theta_entries["theta3"].insert(0, f"{theta3:.2f}")
                self.theta_entries["theta4"].delete(0, tk.END)
                self.theta_entries["theta4"].insert(0, f"{theta4:.2f}")

                self.plot_robot_arm(theta1, theta2, theta3, theta4)
        except ValueError:
            self.error_message_label.config(text="Invalid numerical input.")

    def move_to_xyz(self):
        try:
            x_target = float(self.theta_entries["x"].get() or 0) * 10  # Convert cm to mm
            y_target = float(self.theta_entries["y"].get() or 0) * 10  # Convert cm to mm
            z_target = float(self.theta_entries["z"].get() or 0) * 10  # Convert cm to mm
            gamma = float(self.theta_entries["anglea"].get() or 0)

            initial_guess = [
                float(self.theta_entries["theta1"].get() or 0),
                float(self.theta_entries["theta2"].get() or 0),
                float(self.theta_entries["theta3"].get() or 0),
                float(self.theta_entries["theta4"].get() or 0),
            ]

            results = self.kinematics.inverse_kinematics_optimization(x_target, y_target, z_target, gamma, initial_guess)
            if results:
                theta1, theta2, theta3, theta4 = results

                theta_values = {
                    "theta1": theta1,
                    "theta2": theta2,
                    "theta3": theta3,
                    "theta4": theta4,
                }

                if not self.validate_all_thetas(theta_values):
                    return  # Invalid input

                self.theta_entries["theta1"].delete(0, tk.END)
                self.theta_entries["theta1"].insert(0, f"{theta1:.2f}")
                self.theta_entries["theta2"].delete(0, tk.END)
                self.theta_entries["theta2"].insert(0, f"{theta2:.2f}")
                self.theta_entries["theta3"].delete(0, tk.END)
                self.theta_entries["theta3"].insert(0, f"{theta3:.2f}")
                self.theta_entries["theta4"].delete(0, tk.END)
                self.theta_entries["theta4"].insert(0, f"{theta4:.2f}")

                self.plot_robot_arm(theta1, theta2, theta3, theta4)
                self.move_all_motors(theta1, theta2, theta3, theta4)
        except ValueError:
            self.error_message_label.config(text="Invalid numerical input.")

    def move_to_zero(self):
        # Removed all commands as per your previous request
        pass

    def home_position(self):
        self.arduino.send_command("h")
        # Set home position by setting all angles to 0
        for theta in ["theta1", "theta2", "theta3", "theta4"]:
            self.theta_entries[theta].delete(0, tk.END)
            self.theta_entries[theta].insert(0, "0")
        # Update x, y, z, a
        self.update_xyz_from_theta()
        # Plot the robotic arm at the new position
        self.plot_robot_arm(0, 135, -90, -45)

    def plot_robot_arm(self, theta1, theta2, theta3, theta4):
        self.ax.clear()
        points = self.kinematics.calculate_forward_kinematics(theta1, theta2, theta3, theta4)

        # Plot segments
        for i in range(len(points)-1):
            x_points = [points[i][0], points[i+1][0]]
            y_points = [points[i][1], points[i+1][1]]
            z_points = [points[i][2], points[i+1][2]]
            self.ax.plot(x_points, y_points, z_points, color='blue', linewidth=4)

        # Plot joints
        x_coords = [p[0] for p in points]
        y_coords = [p[1] for p in points]
        z_coords = [p[2] for p in points]
        self.ax.scatter(x_coords, y_coords, z_coords, color='red', s=100)

        # Labels and title
        self.ax.set_xlabel('X-axis (cm)', labelpad=20)
        self.ax.set_ylabel('Y-axis (cm)', labelpad=20)
        self.ax.set_zlabel('Z-axis (cm)', labelpad=20)
        self.ax.set_title("3D Plot of Robotic Arm Position", pad=30)

        # Axis limits
        self.ax.set_xlim([-500, 500])
        self.ax.set_ylim([-500, 500])
        self.ax.set_zlim([0, 500])

        # Aspect ratio and view
        self.ax.set_box_aspect([1, 1, 1])
        self.ax.view_init(elev=30, azim=60)

        self.canvas.draw()

    # **Enhanced Detection and Camera Integration Methods**

    def open_camera(self):
        if self.camera_window is not None:
            self.error_message_label.config(text="Camera is already open.")
            return

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.error_message_label.config(text="Failed to open camera.")
            return

        # Attempt to set camera FPS to 30
        fps_set = self.cap.set(cv2.CAP_PROP_FPS, 30)
        if not fps_set:
            logging.warning("Failed to set camera FPS to 30. Using default FPS.")

        # Verify the set FPS
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        logging.info(f"Camera FPS set to: {actual_fps}")
        print(f"Camera FPS set to: {actual_fps}")

        self.camera_running = True  # Set the flag to True

        # Create a new window to display the camera
        self.camera_window = Toplevel()
        self.camera_window.title("Camera")

        # Create a Label to display the video frame
        self.camera_label = tk.Label(self.camera_window)
        self.camera_label.pack()

        # Function to handle window closing
        self.camera_window.protocol("WM_DELETE_WINDOW", self.close_camera)

        # Enable the "Close Camera" button
        self.close_camera_button.config(state=tk.NORMAL)

        # Start updating frames in a separate thread
        threading.Thread(target=self.run_camera, daemon=True).start()

        # Start processing the frame queue
        self.root.after(0, self.process_frame_queue)

    def close_camera(self):
        if self.camera_window is not None:
            self.camera_running = False  # Stop the update loop
            self.camera_window.destroy()
            self.camera_window = None
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        self.close_camera_button.config(state=tk.DISABLED)
        self.error_message_label.config(text="Camera closed.")

    def run_camera(self):
        frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        center_x = frame_width // 2

        # Desired FPS
        desired_fps = 30
        frame_delay = 1 / desired_fps  # Time per frame in seconds (~0.033s)

        while self.camera_running and not self.stop_event.is_set():
            start_time = time.time()

            ret, frame = self.cap.read()
            if not ret:
                self.error_message_label.config(text="Failed to capture frame from camera.")
                self.close_camera()
                break

            # Optional: Resize frame for faster processing
            frame = cv2.resize(frame, (640, 480))
            frame_width = frame.shape[1]
            center_x = frame_width // 2

            # Detect objects
            frame, red_center = self.detect_red_circle(frame)
            frame, blue_object = self.detect_blue_rectangle(frame)

            current_time = time.time()
            if current_time - self.last_command_time > self.command_interval:
                if red_center or blue_object:
                    # If objects are detected, send movement commands
                    if red_center:
                        print(f"Red Circle detected at: {red_center}")
                        x_diff = red_center[0] - center_x  # Adjusted x_diff calculation
                        theta = self.compute_theta(x_diff, frame_width, fov=self.CAMERA_FOV)
                        print(f"x_diff: {x_diff}, theta: {theta}")  # Debugging
                        if abs(theta) > self.THETA_THRESHOLD:
                            self.send_command_to_robot(theta)
                        else:
                            self.send_command_to_robot(0)

                    if blue_object:
                        print(f"Blue Rectangle detected at: {blue_object}")
                        x_center = blue_object[0] + blue_object[2] // 2
                        x_diff = x_center - center_x  # Adjusted x_diff calculation
                        theta = self.compute_theta(x_diff, frame_width, fov=self.CAMERA_FOV)
                        print(f"x_diff: {x_diff}, theta: {theta}")  # Debugging
                        if abs(theta) > self.THETA_THRESHOLD:
                            self.send_command_to_robot(theta)
                        else:
                            self.send_command_to_robot(0)

                    self.last_command_time = current_time

                    # Reset the stopped flag since an object is detected
                    if self.stopped:
                        self.stopped = False
                else:
                    # No objects detected, send stop command if not already stopped
                    if not self.stopped:
                        print("No objects detected. Sending stop command.")
                        self.arduino.send_command("s")  # Assuming "s" is the stop command
                        self.stopped = True
                        self.last_command_time = current_time

            # Convert image from BGR to RGB for Tkinter display
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            img_tk = ImageTk.PhotoImage(image=img)

            # Put the frame into the queue
            if not self.frame_queue.full():
                self.frame_queue.put(img_tk)

            # Calculate elapsed time and sleep accordingly to maintain 30 FPS
            elapsed_time = time.time() - start_time
            time_to_sleep = frame_delay - elapsed_time
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)

        # Release resources if loop exits
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.close_camera_button.config(state=tk.DISABLED)

    def process_frame_queue(self):
        try:
            while True:
                img_tk = self.frame_queue.get_nowait()
                self.camera_label.config(image=img_tk)
                self.camera_label.image = img_tk
        except queue.Empty:
            pass
        if self.camera_running:
            self.root.after(10, self.process_frame_queue)  # Schedule next check

    def compute_theta(self, x_diff, frame_width, fov=70):
        """
        Convert horizontal pixel difference to an angle theta.

        Args:
            x_diff (int): Difference in pixels between object center and frame center.
            frame_width (int): Width of the video frame in pixels.
            fov (float): Horizontal field of view of the camera in degrees.

        Returns:
            float: Angle theta in degrees.
        """
        proportion = x_diff / (frame_width / 2)
        theta = proportion * (fov / 2)
        theta = max(-fov / 2, min(theta, fov / 2))
        print(f"x_diff: {x_diff}, theta: {theta}")  # Debugging
        return theta

    def send_command_to_robot(self, theta):
        """
        Send a command to the Arduino in the format "f theta1 theta2 theta3 theta4".

        Args:
            theta (float): The angle adjustment to send to the Arduino.
        """
        # Get current theta1 value from GUI
        current_theta1 = float(self.theta_entries["theta1"].get() or 0)
        # Calculate new theta1 by adding the adjustment
        new_theta1 = current_theta1 + (-theta)  # Invert theta if necessary
        # Ensure new_theta1 is within limits
        min_theta1, max_theta1 = self.kinematics.LIMITS['theta1']
        new_theta1 = max(min_theta1, min(new_theta1, max_theta1))
        # Prepare the command with the new theta1 and current theta2, theta3, theta4
        theta2 = float(self.theta_entries["theta2"].get() or 0)
        theta3 = float(self.theta_entries["theta3"].get() or 0)
        theta4 = float(self.theta_entries["theta4"].get() or 0)
        command = f"f {int(round(new_theta1))} {int(round(theta2))} {int(round(theta3))} {int(round(theta4))}"
        print(f"Current theta1: {current_theta1}, Adjusted theta: {theta}, New theta1: {new_theta1}")
        print(f"Sending command to Arduino: {command}")  # Debugging
        self.arduino.send_command(command)
        # Update the GUI entries
        self.theta_entries["theta1"].delete(0, tk.END)
        self.theta_entries["theta1"].insert(0, f"{new_theta1:.2f}")
        # Update internal state
        self.update_xyz_from_theta()

    def detect_red_circle(self, frame):
        """Detect red circles in the frame."""
        blurred = cv2.GaussianBlur(frame, (5, 5), 2)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Updated Red color range in HSV after calibration
        lower_red_1 = np.array([0, 120, 70])
        upper_red_1 = np.array([10, 255, 255])
        lower_red_2 = np.array([170, 120, 70])
        upper_red_2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        mask_red = cv2.add(mask1, mask2)

        # Apply morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask_red = cv2.erode(mask_red, kernel, iterations=1)
        mask_red = cv2.dilate(mask_red, kernel, iterations=2)

        return self.detect_object(frame, mask_red, is_circle=True)

    def detect_blue_rectangle(self, frame):
        """Detect blue rectangles in the frame, including Ford Tractor Blue."""
        blurred = cv2.GaussianBlur(frame, (5, 5), 2)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Existing Blue color range in HSV
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Ford Tractor Blue HSV range
        lower_ford_blue = np.array([105, 150, 50])  # Adjusted based on calibration
        upper_ford_blue = np.array([125, 255, 255])  # Adjusted based on calibration
        mask_ford_blue = cv2.inRange(hsv, lower_ford_blue, upper_ford_blue)

        # Combine both masks to detect all shades of blue including Ford Tractor Blue
        mask_combined_blue = cv2.bitwise_or(mask_blue, mask_ford_blue)

        # Morphological transformations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask_combined_blue = cv2.morphologyEx(mask_combined_blue, cv2.MORPH_CLOSE, kernel)
        mask_combined_blue = cv2.morphologyEx(mask_combined_blue, cv2.MORPH_OPEN, kernel)

        return self.detect_object(frame, mask_combined_blue, is_circle=False)

    def detect_object(self, frame, mask, is_circle):
        """Generic object detection based on mask."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected_object = None

        for contour in contours:
            if cv2.contourArea(contour) > 500:
                if is_circle:
                    if len(contour) >= 5:
                        (x_center, y_center), radius = cv2.minEnclosingCircle(contour)
                        diameter_cm = (2 * radius) / self.PIXELS_PER_CM
                        cv2.circle(frame, (int(x_center), int(y_center)), int(radius), (0, 255, 0), 2)
                        cv2.putText(frame, f"D: {diameter_cm:.2f} cm",
                                    (int(x_center) - 50, int(y_center) - int(radius) - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        detected_object = (int(x_center), int(y_center))
                else:
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = w / float(h)
                    # Adjust aspect ratio range for rectangles if needed
                    if 0.5 <= aspect_ratio <= 2.0:
                        width_cm = w / self.PIXELS_PER_CM
                        height_cm = h / self.PIXELS_PER_CM
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                        cv2.putText(frame, f"W: {width_cm:.2f}cm H: {height_cm:.2f}cm",
                                    (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        detected_object = (x, y, w, h)

        return frame, detected_object


def select_port_gui(root, arduino_ports):
    # Create a new window for port selection
    top = Toplevel(root)
    top.title("Select Arduino Port")
    top.geometry("400x150")
    top.resizable(False, False)

    selected_port = StringVar(top)
    selected_port.set(f"{arduino_ports[0][0]} - {arduino_ports[0][1]}")  # Default value

    # Dropdown menu
    dropdown = OptionMenu(top, selected_port, *[f"{p[0]} - {p[1]}" for p in arduino_ports])
    dropdown.pack(pady=20)

    # Confirm button
    def confirm_selection():
        top.destroy()

    Button(top, text="Confirm", command=confirm_selection, width=15).pack(pady=10)
    root.wait_window(top)

    for port in arduino_ports:
        if selected_port.get().startswith(port[0]):
            return port[0]
    return None


def on_closing(root, arduino, gui):
    if messagebox.askokcancel("Quit", "Do you want to quit?"):
        if gui.camera_running:
            gui.close_camera()
        arduino.close()
        root.destroy()


def main():
    root = tk.Tk()
    root.withdraw()  # Hide the main window during port selection

    arduino_controller = ArduinoController()
    arduino_ports = arduino_controller.find_arduinos()

    if not arduino_ports:
        messagebox.showerror("Error", "No Arduino ports found! Please connect Arduino and try again.")
        root.destroy()
        sys.exit()

    # Select Arduino port
    selected_port = select_port_gui(root, arduino_ports)
    if not selected_port:
        messagebox.showinfo("Cancel", "You did not select an Arduino port.")
        root.destroy()
        sys.exit()

    # Connect to Arduino
    arduino_controller.connect(selected_port)

    # Initialize kinematics
    kinematics = Kinematics()

    # Initialize GUI
    root.deiconify()  # Show the main window
    app = RoboticArmGUI(root, arduino_controller, kinematics)
    root.protocol("WM_DELETE_WINDOW", lambda: on_closing(root, arduino_controller, app))
    root.mainloop()


if __name__ == "__main__":
    main()
