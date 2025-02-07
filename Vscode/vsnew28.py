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
import queue

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
            if "Arduino" in port.description:
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
            full_command = f"{command_str}\n"
            self.serial.write(full_command.encode())
            logging.info(f"Sent command: {command_str}")
            time.sleep(0.05)  # Reduced sleep for faster communication
            if self.serial.in_waiting > 0:
                response = self.serial.readline().decode().strip()
                logging.info(f"Received response: {response}")
                return response
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
            "theta1": (-1, 181),
            "theta2": (-1, 161),
            "theta3": (-121, 155),
            "theta4": (-180, 180),
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
            return None


class RoboticArmGUI:
    def __init__(self, root, arduino, kinematics):
        self.root = root
        self.arduino = arduino
        self.kinematics = kinematics
        self.camera_window = None
        self.cap = None
        self.camera_running = False  # Flag to control the camera loop
        self.PIXELS_PER_CM = 37  # Calibration: pixels per centimeter
        self.CAMERA_FOV = 60  # Horizontal field of view in degrees
        self.THETA_THRESHOLD = 2  # Threshold in degrees for stopping
        self.last_command_time = 0
        self.command_interval = 0.1  # seconds

        self.frame_queue = queue.Queue(maxsize=10)  # Queue for thread-safe frame passing
        self.hsv_values = {'lower1': np.array([0, 100, 50]),
                           'upper1': np.array([10, 255, 255]),
                           'lower2': np.array([170, 100, 50]),
                           'upper2': np.array([180, 255, 255]),
                           'lower_blue': np.array([100, 150, 50]),
                           'upper_blue': np.array([140, 255, 255])}

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
        self.fig = plt.figure(figsize=(8, 6))  # Reduced size for faster plotting
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.right_frame)
        self.canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=1)

        # Input fields and buttons
        self.create_input_fields(font_style)
        self.create_buttons(font_style)
        self.create_calibration_button(font_style)

        # Initialize plot
        self.plot_robot_arm(0, 0, 0, 0)

    def create_input_fields(self, font_style):
        # Theta entries
        self.theta_entries = {}
        for i, theta in enumerate(["Theta 1", "Theta 2", "Theta 3", "Theta 4"], start=1):
            tk.Label(self.left_frame, text=theta, font=font_style).pack(pady=(5, 5))
            entry = tk.Entry(self.left_frame, font=font_style, justify="center")
            entry.pack(pady=(0, 5))
            entry.bind("<KeyRelease>", lambda event, t=theta.lower().replace(" ", ""): self.update_xyz_from_theta())
            self.theta_entries[theta.lower().replace(" ", "")] = entry

        # X, Y, Z, A entries
        for param in ["X", "Y", "Z", "Angle A"]:
            tk.Label(self.left_frame, text=param, font=font_style).pack(pady=(5, 5))
            entry = tk.Entry(self.left_frame, font=font_style, justify="center")
            entry.pack(pady=(0, 5))
            entry.bind("<KeyRelease>", lambda event, p=param.lower().replace(" ", ""): self.update_theta_from_xyz())
            self.theta_entries[param.lower().replace(" ", "")] = entry

    def create_buttons(self, font_style):
        # Move Kinematics Button
        tk.Button(self.left_frame, text="Move Kinematics", command=self.on_move_all,
                  font=font_style, height=1, width=25).pack(pady=(1, 1))

        # Move Inverse Button
        tk.Button(self.left_frame, text="Move Inverse", command=self.move_to_xyz,
                  font=font_style, height=2, width=25).pack(pady=(1, 1))

        # Move to Zero Button
        tk.Button(self.left_frame, text="Move to Zero point", command=self.move_to_zero,
                  font=font_style, height=2, width=25).pack(pady=(1, 1))

        # Camera Buttons
        tk.Button(self.left_frame, text="Camera", command=self.open_camera,
                  font=font_style, height=2, width=25).pack(pady=(1, 1))
        self.close_camera_button = tk.Button(self.left_frame, text="Close Camera",
                                             command=self.close_camera, font=font_style,
                                             height=2, width=25, state=tk.DISABLED)
        self.close_camera_button.pack(pady=(1, 1))

        # Home and Stop Buttons
        tk.Button(self.left_frame, text="Home", command=self.home_position,
                  font=font_style, height=1, width=25).pack(pady=(0, 1))
        tk.Button(self.left_frame, text="Stop", command=lambda: self.arduino.send_command("s"),
                  font=font_style, height=3, width=25, bg='red', fg="white").pack(pady=(0, 1))

    def create_calibration_button(self, font_style):
        # Calibration Button
        tk.Button(self.left_frame, text="Calibrate Colors", command=self.open_calibration_window,
                  font=font_style, height=2, width=25).pack(pady=(10, 1))

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
            self.theta_entries["x"].insert(0, f"{xF:.2f}")
            self.theta_entries["y"].delete(0, tk.END)
            self.theta_entries["y"].insert(0, f"{yF:.2f}")
            self.theta_entries["z"].delete(0, tk.END)
            self.theta_entries["z"].insert(0, f"{zF:.2f}")
            self.theta_entries["anglea"].delete(0, tk.END)
            self.theta_entries["anglea"].insert(0, f"{gamma:.2f}")

            self.plot_robot_arm(theta1, theta2, theta3, theta4)
        except ValueError:
            self.error_message_label.config(text="Invalid numerical input.")

    def update_theta_from_xyz(self):
        try:
            x_target = float(self.theta_entries["x"].get() or 0)
            y_target = float(self.theta_entries["y"].get() or 0)
            z_target = float(self.theta_entries["z"].get() or 0)
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
            x_target = float(self.theta_entries["x"].get() or 0)
            y_target = float(self.theta_entries["y"].get() or 0)
            z_target = float(self.theta_entries["z"].get() or 0)
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
                self.arduino.send_command(f"f {int(round(theta1))} {int(round(theta2))} {int(round(theta3))} {int(round(theta4))}")
        except ValueError:
            self.error_message_label.config(text="Invalid numerical input.")

    def move_to_zero(self):
        self.arduino.send_command("f 0 0 0 0")
        # Update GUI entries
        for theta in ["theta1", "theta2", "theta3", "theta4"]:
            self.theta_entries[theta].delete(0, tk.END)
            self.theta_entries[theta].insert(0, "0")
        # Update x, y, z, a
        self.update_xyz_from_theta()
        # Plot the robotic arm at zero position
        self.plot_robot_arm(0, 0, 0, 0)

    def home_position(self):
        self.arduino.send_command("h")
        # Set home position by setting all angles to 0
        for theta in ["theta1", "theta2", "theta3", "theta4"]:
            self.theta_entries[theta].delete(0, tk.END)
            self.theta_entries[theta].insert(0, "0")
        # Update x, y, z, a
        self.update_xyz_from_theta()
        # Plot the robotic arm at the new position
        self.plot_robot_arm(0, 0, 0, 0)

    def plot_robot_arm(self, theta1, theta2, theta3, theta4):
        self.ax.cla()  # Clear the current axes

        points = self.kinematics.calculate_forward_kinematics(theta1, theta2, theta3, theta4)

        # Extract coordinates
        x_coords = [p[0] for p in points]
        y_coords = [p[1] for p in points]
        z_coords = [p[2] for p in points]

        # Plot segments
        self.ax.plot(x_coords, y_coords, z_coords, color='blue', linewidth=4)

        # Plot joints
        self.ax.scatter(x_coords, y_coords, z_coords, color='red', s=100)

        # Labels and title
        self.ax.set_xlabel('X-axis (mm)', labelpad=10)
        self.ax.set_ylabel('Y-axis (mm)', labelpad=10)
        self.ax.set_zlabel('Z-axis (mm)', labelpad=10)
        self.ax.set_title("3D Plot of Robotic Arm Position", pad=10)

        # Axis limits
        self.ax.set_xlim([-450, 450])
        self.ax.set_ylim([-450, 450])
        self.ax.set_zlim([0, 450])

        # Aspect ratio and view
        self.ax.set_box_aspect([1, 1, 1])
        self.ax.view_init(elev=30, azim=60)

        self.canvas.draw()

    # **New Detection and Camera Integration Methods**

    def open_camera(self):
        if self.camera_window is not None:
            self.error_message_label.config(text="Camera is already open.")
            return

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.error_message_label.config(text="Failed to open camera.")
            return

        # Optionally set fixed brightness and exposure
        # fixed_brightness = 0.5  # Example value between 0 and 1
        # fixed_exposure = -6     # Example exposure value
        # self.cap.set(cv2.CAP_PROP_BRIGHTNESS, fixed_brightness)
        # self.cap.set(cv2.CAP_PROP_EXPOSURE, fixed_exposure)

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
        self.root.after(100, self.process_frame_queue)  # Start processing the frame queue

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

        frame_count = 0  # To limit object detection frequency

        while self.camera_running:
            ret, frame = self.cap.read()
            if not ret:
                self.error_message_label.config(text="Failed to capture frame from camera.")
                self.close_camera()
                break

            # Optional: Resize frame for faster processing
            # Adjust the resolution as needed
            desired_width = 640
            desired_height = 480
            frame = cv2.resize(frame, (desired_width, desired_height))

            # Increment frame count
            frame_count += 1

            # Perform object detection every 3 frames to reduce processing load
            if frame_count % 3 == 0:
                frame, red_center = self.detect_red_circle(frame)
                frame, blue_object = self.detect_blue_rectangle(frame)

                current_time = time.time()
                if current_time - self.last_command_time > self.command_interval:
                    # Handle red circle
                    if red_center:
                        x_diff = center_x - red_center[0]
                        theta = self.compute_theta(x_diff, frame_width, fov=self.CAMERA_FOV)
                        if abs(theta) > self.THETA_THRESHOLD:
                            self.send_command_to_robot(theta)
                        else:
                            self.send_command_to_robot(0)

                    # Handle blue rectangle
                    if blue_object:
                        x_center = blue_object[0] + blue_object[2] // 2
                        x_diff = center_x - x_center
                        theta = self.compute_theta(x_diff, frame_width, fov=self.CAMERA_FOV)
                        if abs(theta) > self.THETA_THRESHOLD:
                            self.send_command_to_robot(theta)
                        else:
                            self.send_command_to_robot(0)

                    self.last_command_time = current_time

            # Convert image from BGR to RGB for Tkinter display
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            img_tk = ImageTk.PhotoImage(image=img)

            # Put the frame in the queue
            if not self.frame_queue.full():
                self.frame_queue.put(img_tk)

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
            self.root.after(10, self.process_frame_queue)  # Continue processing

    def compute_theta(self, x_diff, frame_width, fov=60):
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
        return theta

    def send_command_to_robot(self, theta):
        """
        Send a command to the Arduino in the format "1 theta".

        Args:
            theta (float): The angle to send to the Arduino.
        """
        command = f"1 {theta}"
        self.arduino.send_command(command)

    def detect_red_circle(self, frame):
        """Detect red circles in the frame."""
        blurred = cv2.GaussianBlur(frame, (5, 5), 2)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Red color range in HSV from calibrated values
        lower_red_1 = self.hsv_values['lower1']
        upper_red_1 = self.hsv_values['upper1']
        lower_red_2 = self.hsv_values['lower2']
        upper_red_2 = self.hsv_values['upper2']

        mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        mask_red = cv2.add(mask1, mask2)

        return self.detect_object(frame, mask_red, is_circle=True)

    def detect_blue_rectangle(self, frame):
        """Detect blue rectangles in the frame."""
        blurred = cv2.GaussianBlur(frame, (5, 5), 2)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Blue color range in HSV from calibrated values
        lower_blue = self.hsv_values['lower_blue']
        upper_blue = self.hsv_values['upper_blue']
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Morphological transformations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

        return self.detect_object(frame, mask_blue, is_circle=False)

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

    def open_calibration_window(self):
        """Open a window with trackbars to calibrate HSV values."""
        self.calibration_window = Toplevel(self.root)
        self.calibration_window.title("HSV Calibration")
        self.calibration_window.geometry("400x600")
        self.calibration_window.resizable(False, False)

        # Create trackbars for red color
        tk.Label(self.calibration_window, text="Red Color Calibration", font=('Helvetica', 12, 'bold')).pack(pady=10)

        # Lower Red 1
        cv2.namedWindow("Calibrate Red Range 1")
        cv2.createTrackbar("H_lower1", "Calibrate Red Range 1", 0, 179, lambda x: None)
        cv2.createTrackbar("S_lower1", "Calibrate Red Range 1", 100, 255, lambda x: None)
        cv2.createTrackbar("V_lower1", "Calibrate Red Range 1", 50, 255, lambda x: None)

        # Upper Red 1
        cv2.createTrackbar("H_upper1", "Calibrate Red Range 1", 10, 179, lambda x: None)
        cv2.createTrackbar("S_upper1", "Calibrate Red Range 1", 255, 255, lambda x: None)
        cv2.createTrackbar("V_upper1", "Calibrate Red Range 1", 255, 255, lambda x: None)

        # Lower Red 2
        cv2.createTrackbar("H_lower2", "Calibrate Red Range 1", 170, 179, lambda x: None)
        cv2.createTrackbar("S_lower2", "Calibrate Red Range 1", 100, 255, lambda x: None)
        cv2.createTrackbar("V_lower2", "Calibrate Red Range 1", 50, 255, lambda x: None)

        # Upper Red 2
        cv2.createTrackbar("H_upper2", "Calibrate Red Range 1", 180, 179, lambda x: None)
        cv2.createTrackbar("S_upper2", "Calibrate Red Range 1", 255, 255, lambda x: None)
        cv2.createTrackbar("V_upper2", "Calibrate Red Range 1", 255, 255, lambda x: None)

        # Create trackbars for blue color
        tk.Label(self.calibration_window, text="Blue Color Calibration", font=('Helvetica', 12, 'bold')).pack(pady=10)

        cv2.namedWindow("Calibrate Blue Range")
        cv2.createTrackbar("H_lower_blue", "Calibrate Blue Range", 100, 179, lambda x: None)
        cv2.createTrackbar("S_lower_blue", "Calibrate Blue Range", 150, 255, lambda x: None)
        cv2.createTrackbar("V_lower_blue", "Calibrate Blue Range", 50, 255, lambda x: None)

        cv2.createTrackbar("H_upper_blue", "Calibrate Blue Range", 140, 179, lambda x: None)
        cv2.createTrackbar("S_upper_blue", "Calibrate Blue Range", 255, 255, lambda x: None)
        cv2.createTrackbar("V_upper_blue", "Calibrate Blue Range", 255, 255, lambda x: None)

        # Instruction Label
        tk.Label(self.calibration_window, text="Adjust the sliders to match your object's color.\nPress 'q' in the OpenCV window to finish calibration.",
                 font=('Helvetica', 10), justify="left").pack(pady=10)

        # Start calibration in a separate thread
        threading.Thread(target=self.calibrate_hsv, daemon=True).start()

    def calibrate_hsv(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue

            frame = cv2.resize(frame, (640, 480))
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Get current positions of all trackbars
            lower_red1 = np.array([
                cv2.getTrackbarPos("H_lower1", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("S_lower1", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("V_lower1", "Calibrate Red Range 1")
            ])
            upper_red1 = np.array([
                cv2.getTrackbarPos("H_upper1", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("S_upper1", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("V_upper1", "Calibrate Red Range 1")
            ])
            lower_red2 = np.array([
                cv2.getTrackbarPos("H_lower2", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("S_lower2", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("V_lower2", "Calibrate Red Range 1")
            ])
            upper_red2 = np.array([
                cv2.getTrackbarPos("H_upper2", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("S_upper2", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("V_upper2", "Calibrate Red Range 1")
            ])
            lower_blue = np.array([
                cv2.getTrackbarPos("H_lower_blue", "Calibrate Blue Range"),
                cv2.getTrackbarPos("S_lower_blue", "Calibrate Blue Range"),
                cv2.getTrackbarPos("V_lower_blue", "Calibrate Blue Range")
            ])
            upper_blue = np.array([
                cv2.getTrackbarPos("H_upper_blue", "Calibrate Blue Range"),
                cv2.getTrackbarPos("S_upper_blue", "Calibrate Blue Range"),
                cv2.getTrackbarPos("V_upper_blue", "Calibrate Blue Range")
            ])

            # Update HSV values
            self.hsv_values['lower1'] = lower_red1
            self.hsv_values['upper1'] = upper_red1
            self.hsv_values['lower2'] = lower_red2
            self.hsv_values['upper2'] = upper_red2
            self.hsv_values['lower_blue'] = lower_blue
            self.hsv_values['upper_blue'] = upper_blue

            # Create masks
            mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = cv2.add(mask_red1, mask_red2)
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

            # Combine masks for display
            combined_mask = cv2.bitwise_or(mask_red, mask_blue)

            # Display the masks
            cv2.imshow("Calibrate Red Range 1", mask_red)
            cv2.imshow("Calibrate Blue Range", mask_blue)

            # Exit calibration if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()
        self.calibration_window.destroy()

    def run_camera(self):
        frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        center_x = frame_width // 2

        frame_count = 0  # To limit object detection frequency

        while self.camera_running:
            ret, frame = self.cap.read()
            if not ret:
                self.error_message_label.config(text="Failed to capture frame from camera.")
                self.close_camera()
                break

            # Optional: Resize frame for faster processing
            # Adjust the resolution as needed
            desired_width = 640
            desired_height = 480
            frame = cv2.resize(frame, (desired_width, desired_height))

            # Increment frame count
            frame_count += 1

            # Perform object detection every 3 frames to reduce processing load
            if frame_count % 3 == 0:
                frame, red_center = self.detect_red_circle(frame)
                frame, blue_object = self.detect_blue_rectangle(frame)

                current_time = time.time()
                if current_time - self.last_command_time > self.command_interval:
                    # Handle red circle
                    if red_center:
                        x_diff = center_x - red_center[0]
                        theta = self.compute_theta(x_diff, frame_width, fov=self.CAMERA_FOV)
                        if abs(theta) > self.THETA_THRESHOLD:
                            self.send_command_to_robot(theta)
                        else:
                            self.send_command_to_robot(0)

                    # Handle blue rectangle
                    if blue_object:
                        x_center = blue_object[0] + blue_object[2] // 2
                        x_diff = center_x - x_center
                        theta = self.compute_theta(x_diff, frame_width, fov=self.CAMERA_FOV)
                        if abs(theta) > self.THETA_THRESHOLD:
                            self.send_command_to_robot(theta)
                        else:
                            self.send_command_to_robot(0)

                    self.last_command_time = current_time

            # Convert image from BGR to RGB for Tkinter display
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            img_tk = ImageTk.PhotoImage(image=img)

            # Put the frame in the queue
            if not self.frame_queue.full():
                self.frame_queue.put(img_tk)

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
            self.root.after(10, self.process_frame_queue)  # Continue processing

    def compute_theta(self, x_diff, frame_width, fov=60):
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
        return theta

    def send_command_to_robot(self, theta):
        """
        Send a command to the Arduino in the format "1 theta".

        Args:
            theta (float): The angle to send to the Arduino.
        """
        command = f"1 {theta}"
        self.arduino.send_command(command)

    def detect_red_circle(self, frame):
        """Detect red circles in the frame."""
        blurred = cv2.GaussianBlur(frame, (5, 5), 2)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Red color range in HSV from calibrated values
        lower_red1 = self.hsv_values['lower1']
        upper_red1 = self.hsv_values['upper1']
        lower_red2 = self.hsv_values['lower2']
        upper_red2 = self.hsv_values['upper2']

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.add(mask1, mask2)

        return self.detect_object(frame, mask_red, is_circle=True)

    def detect_blue_rectangle(self, frame):
        """Detect blue rectangles in the frame."""
        blurred = cv2.GaussianBlur(frame, (5, 5), 2)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Blue color range in HSV from calibrated values
        lower_blue = self.hsv_values['lower_blue']
        upper_blue = self.hsv_values['upper_blue']
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Morphological transformations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

        return self.detect_object(frame, mask_blue, is_circle=False)

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

    def open_calibration_window(self):
        """Open a window with trackbars to calibrate HSV values."""
        self.calibration_window = Toplevel(self.root)
        self.calibration_window.title("HSV Calibration")
        self.calibration_window.geometry("400x600")
        self.calibration_window.resizable(False, False)

        # Create trackbars for red color
        tk.Label(self.calibration_window, text="Red Color Calibration", font=('Helvetica', 12, 'bold')).pack(pady=10)

        # Lower Red 1
        cv2.namedWindow("Calibrate Red Range 1")
        cv2.createTrackbar("H_lower1", "Calibrate Red Range 1", 0, 179, lambda x: None)
        cv2.createTrackbar("S_lower1", "Calibrate Red Range 1", 100, 255, lambda x: None)
        cv2.createTrackbar("V_lower1", "Calibrate Red Range 1", 50, 255, lambda x: None)

        # Upper Red 1
        cv2.createTrackbar("H_upper1", "Calibrate Red Range 1", 10, 179, lambda x: None)
        cv2.createTrackbar("S_upper1", "Calibrate Red Range 1", 255, 255, lambda x: None)
        cv2.createTrackbar("V_upper1", "Calibrate Red Range 1", 255, 255, lambda x: None)

        # Lower Red 2
        cv2.createTrackbar("H_lower2", "Calibrate Red Range 1", 170, 179, lambda x: None)
        cv2.createTrackbar("S_lower2", "Calibrate Red Range 1", 100, 255, lambda x: None)
        cv2.createTrackbar("V_lower2", "Calibrate Red Range 1", 50, 255, lambda x: None)

        # Upper Red 2
        cv2.createTrackbar("H_upper2", "Calibrate Red Range 1", 180, 179, lambda x: None)
        cv2.createTrackbar("S_upper2", "Calibrate Red Range 1", 255, 255, lambda x: None)
        cv2.createTrackbar("V_upper2", "Calibrate Red Range 1", 255, 255, lambda x: None)

        # Create trackbars for blue color
        tk.Label(self.calibration_window, text="Blue Color Calibration", font=('Helvetica', 12, 'bold')).pack(pady=10)

        cv2.namedWindow("Calibrate Blue Range")
        cv2.createTrackbar("H_lower_blue", "Calibrate Blue Range", 100, 179, lambda x: None)
        cv2.createTrackbar("S_lower_blue", "Calibrate Blue Range", 150, 255, lambda x: None)
        cv2.createTrackbar("V_lower_blue", "Calibrate Blue Range", 50, 255, lambda x: None)

        cv2.createTrackbar("H_upper_blue", "Calibrate Blue Range", 140, 179, lambda x: None)
        cv2.createTrackbar("S_upper_blue", "Calibrate Blue Range", 255, 255, lambda x: None)
        cv2.createTrackbar("V_upper_blue", "Calibrate Blue Range", 255, 255, lambda x: None)

        # Instruction Label
        tk.Label(self.calibration_window, text="Adjust the sliders to match your object's color.\nPress 'q' in the OpenCV window to finish calibration.",
                 font=('Helvetica', 10), justify="left").pack(pady=10)

        # Start calibration in a separate thread
        threading.Thread(target=self.calibrate_hsv, daemon=True).start()

    def calibrate_hsv(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue

            frame = cv2.resize(frame, (640, 480))
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Get current positions of all trackbars
            lower_red1 = np.array([
                cv2.getTrackbarPos("H_lower1", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("S_lower1", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("V_lower1", "Calibrate Red Range 1")
            ])
            upper_red1 = np.array([
                cv2.getTrackbarPos("H_upper1", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("S_upper1", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("V_upper1", "Calibrate Red Range 1")
            ])
            lower_red2 = np.array([
                cv2.getTrackbarPos("H_lower2", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("S_lower2", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("V_lower2", "Calibrate Red Range 1")
            ])
            upper_red2 = np.array([
                cv2.getTrackbarPos("H_upper2", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("S_upper2", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("V_upper2", "Calibrate Red Range 1")
            ])
            lower_blue = np.array([
                cv2.getTrackbarPos("H_lower_blue", "Calibrate Blue Range"),
                cv2.getTrackbarPos("S_lower_blue", "Calibrate Blue Range"),
                cv2.getTrackbarPos("V_lower_blue", "Calibrate Blue Range")
            ])
            upper_blue = np.array([
                cv2.getTrackbarPos("H_upper_blue", "Calibrate Blue Range"),
                cv2.getTrackbarPos("S_upper_blue", "Calibrate Blue Range"),
                cv2.getTrackbarPos("V_upper_blue", "Calibrate Blue Range")
            ])

            # Update HSV values
            self.hsv_values['lower1'] = lower_red1
            self.hsv_values['upper1'] = upper_red1
            self.hsv_values['lower2'] = lower_red2
            self.hsv_values['upper2'] = upper_red2
            self.hsv_values['lower_blue'] = lower_blue
            self.hsv_values['upper_blue'] = upper_blue

            # Create masks
            mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = cv2.add(mask_red1, mask_red2)
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

            # Combine masks for display
            combined_mask = cv2.bitwise_or(mask_red, mask_blue)

            # Display the masks
            cv2.imshow("Calibrate Red Range 1", mask_red)
            cv2.imshow("Calibrate Blue Range", mask_blue)

            # Exit calibration if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()
        self.calibration_window.destroy()

    def run_camera(self):
        frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        center_x = frame_width // 2

        frame_count = 0  # To limit object detection frequency

        while self.camera_running:
            ret, frame = self.cap.read()
            if not ret:
                self.error_message_label.config(text="Failed to capture frame from camera.")
                self.close_camera()
                break

            # Optional: Resize frame for faster processing
            # Adjust the resolution as needed
            desired_width = 640
            desired_height = 480
            frame = cv2.resize(frame, (desired_width, desired_height))

            # Increment frame count
            frame_count += 1

            # Perform object detection every 3 frames to reduce processing load
            if frame_count % 3 == 0:
                frame, red_center = self.detect_red_circle(frame)
                frame, blue_object = self.detect_blue_rectangle(frame)

                current_time = time.time()
                if current_time - self.last_command_time > self.command_interval:
                    # Handle red circle
                    if red_center:
                        x_diff = center_x - red_center[0]
                        theta = self.compute_theta(x_diff, frame_width, fov=self.CAMERA_FOV)
                        if abs(theta) > self.THETA_THRESHOLD:
                            self.send_command_to_robot(theta)
                        else:
                            self.send_command_to_robot(0)

                    # Handle blue rectangle
                    if blue_object:
                        x_center = blue_object[0] + blue_object[2] // 2
                        x_diff = center_x - x_center
                        theta = self.compute_theta(x_diff, frame_width, fov=self.CAMERA_FOV)
                        if abs(theta) > self.THETA_THRESHOLD:
                            self.send_command_to_robot(theta)
                        else:
                            self.send_command_to_robot(0)

                    self.last_command_time = current_time

            # Convert image from BGR to RGB for Tkinter display
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            img_tk = ImageTk.PhotoImage(image=img)

            # Put the frame in the queue
            if not self.frame_queue.full():
                self.frame_queue.put(img_tk)

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
            self.root.after(10, self.process_frame_queue)  # Continue processing

    def compute_theta(self, x_diff, frame_width, fov=60):
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
        return theta

    def send_command_to_robot(self, theta):
        """
        Send a command to the Arduino in the format "1 theta".

        Args:
            theta (float): The angle to send to the Arduino.
        """
        command = f"1 {theta}"
        self.arduino.send_command(command)

    def detect_red_circle(self, frame):
        """Detect red circles in the frame."""
        blurred = cv2.GaussianBlur(frame, (5, 5), 2)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Red color range in HSV from calibrated values
        lower_red1 = self.hsv_values['lower1']
        upper_red1 = self.hsv_values['upper1']
        lower_red2 = self.hsv_values['lower2']
        upper_red2 = self.hsv_values['upper2']

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.add(mask1, mask2)

        return self.detect_object(frame, mask_red, is_circle=True)

    def detect_blue_rectangle(self, frame):
        """Detect blue rectangles in the frame."""
        blurred = cv2.GaussianBlur(frame, (5, 5), 2)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Blue color range in HSV from calibrated values
        lower_blue = self.hsv_values['lower_blue']
        upper_blue = self.hsv_values['upper_blue']
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Morphological transformations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

        return self.detect_object(frame, mask_blue, is_circle=False)

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

    def open_calibration_window(self):
        """Open a window with trackbars to calibrate HSV values."""
        self.calibration_window = Toplevel(self.root)
        self.calibration_window.title("HSV Calibration")
        self.calibration_window.geometry("400x600")
        self.calibration_window.resizable(False, False)

        # Create trackbars for red color
        tk.Label(self.calibration_window, text="Red Color Calibration", font=('Helvetica', 12, 'bold')).pack(pady=10)

        # Lower Red 1
        cv2.namedWindow("Calibrate Red Range 1")
        cv2.createTrackbar("H_lower1", "Calibrate Red Range 1", 0, 179, lambda x: None)
        cv2.createTrackbar("S_lower1", "Calibrate Red Range 1", 100, 255, lambda x: None)
        cv2.createTrackbar("V_lower1", "Calibrate Red Range 1", 50, 255, lambda x: None)

        # Upper Red 1
        cv2.createTrackbar("H_upper1", "Calibrate Red Range 1", 10, 179, lambda x: None)
        cv2.createTrackbar("S_upper1", "Calibrate Red Range 1", 255, 255, lambda x: None)
        cv2.createTrackbar("V_upper1", "Calibrate Red Range 1", 255, 255, lambda x: None)

        # Lower Red 2
        cv2.createTrackbar("H_lower2", "Calibrate Red Range 1", 170, 179, lambda x: None)
        cv2.createTrackbar("S_lower2", "Calibrate Red Range 1", 100, 255, lambda x: None)
        cv2.createTrackbar("V_lower2", "Calibrate Red Range 1", 50, 255, lambda x: None)

        # Upper Red 2
        cv2.createTrackbar("H_upper2", "Calibrate Red Range 1", 180, 179, lambda x: None)
        cv2.createTrackbar("S_upper2", "Calibrate Red Range 1", 255, 255, lambda x: None)
        cv2.createTrackbar("V_upper2", "Calibrate Red Range 1", 255, 255, lambda x: None)

        # Create trackbars for blue color
        tk.Label(self.calibration_window, text="Blue Color Calibration", font=('Helvetica', 12, 'bold')).pack(pady=10)

        cv2.namedWindow("Calibrate Blue Range")
        cv2.createTrackbar("H_lower_blue", "Calibrate Blue Range", 100, 179, lambda x: None)
        cv2.createTrackbar("S_lower_blue", "Calibrate Blue Range", 150, 255, lambda x: None)
        cv2.createTrackbar("V_lower_blue", "Calibrate Blue Range", 50, 255, lambda x: None)

        cv2.createTrackbar("H_upper_blue", "Calibrate Blue Range", 140, 179, lambda x: None)
        cv2.createTrackbar("S_upper_blue", "Calibrate Blue Range", 255, 255, lambda x: None)
        cv2.createTrackbar("V_upper_blue", "Calibrate Blue Range", 255, 255, lambda x: None)

        # Instruction Label
        tk.Label(self.calibration_window, text="Adjust the sliders to match your object's color.\nPress 'q' in the OpenCV window to finish calibration.",
                 font=('Helvetica', 10), justify="left").pack(pady=10)

        # Start calibration in a separate thread
        threading.Thread(target=self.calibrate_hsv, daemon=True).start()

    def calibrate_hsv(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue

            frame = cv2.resize(frame, (640, 480))
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Get current positions of all trackbars
            lower_red1 = np.array([
                cv2.getTrackbarPos("H_lower1", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("S_lower1", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("V_lower1", "Calibrate Red Range 1")
            ])
            upper_red1 = np.array([
                cv2.getTrackbarPos("H_upper1", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("S_upper1", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("V_upper1", "Calibrate Red Range 1")
            ])
            lower_red2 = np.array([
                cv2.getTrackbarPos("H_lower2", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("S_lower2", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("V_lower2", "Calibrate Red Range 1")
            ])
            upper_red2 = np.array([
                cv2.getTrackbarPos("H_upper2", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("S_upper2", "Calibrate Red Range 1"),
                cv2.getTrackbarPos("V_upper2", "Calibrate Red Range 1")
            ])
            lower_blue = np.array([
                cv2.getTrackbarPos("H_lower_blue", "Calibrate Blue Range"),
                cv2.getTrackbarPos("S_lower_blue", "Calibrate Blue Range"),
                cv2.getTrackbarPos("V_lower_blue", "Calibrate Blue Range")
            ])
            upper_blue = np.array([
                cv2.getTrackbarPos("H_upper_blue", "Calibrate Blue Range"),
                cv2.getTrackbarPos("S_upper_blue", "Calibrate Blue Range"),
                cv2.getTrackbarPos("V_upper_blue", "Calibrate Blue Range")
            ])

            # Update HSV values
            self.hsv_values['lower1'] = lower_red1
            self.hsv_values['upper1'] = upper_red1
            self.hsv_values['lower2'] = lower_red2
            self.hsv_values['upper2'] = upper_red2
            self.hsv_values['lower_blue'] = lower_blue
            self.hsv_values['upper_blue'] = upper_blue

            # Create masks
            mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = cv2.add(mask_red1, mask_red2)
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

            # Combine masks for display
            combined_mask = cv2.bitwise_or(mask_red, mask_blue)

            # Display the masks
            cv2.imshow("Calibrate Red Range 1", mask_red)
            cv2.imshow("Calibrate Blue Range", mask_blue)

            # Exit calibration if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()
        self.calibration_window.destroy()

    def run_camera(self):
        frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        center_x = frame_width // 2

        frame_count = 0  # To limit object detection frequency

        while self.camera_running:
            ret, frame = self.cap.read()
            if not ret:
                self.error_message_label.config(text="Failed to capture frame from camera.")
                self.close_camera()
                break

            # Optional: Resize frame for faster processing
            # Adjust the resolution as needed
            desired_width = 640
            desired_height = 480
            frame = cv2.resize(frame, (desired_width, desired_height))

            # Increment frame count
            frame_count += 1

            # Perform object detection every 3 frames to reduce processing load
            if frame_count % 3 == 0:
                frame, red_center = self.detect_red_circle(frame)
                frame, blue_object = self.detect_blue_rectangle(frame)

                current_time = time.time()
                if current_time - self.last_command_time > self.command_interval:
                    # Handle red circle
                    if red_center:
                        x_diff = center_x - red_center[0]
                        theta = self.compute_theta(x_diff, frame_width, fov=self.CAMERA_FOV)
                        if abs(theta) > self.THETA_THRESHOLD:
                            self.send_command_to_robot(theta)
                        else:
                            self.send_command_to_robot(0)

                    # Handle blue rectangle
                    if blue_object:
                        x_center = blue_object[0] + blue_object[2] // 2
                        x_diff = center_x - x_center
                        theta = self.compute_theta(x_diff, frame_width, fov=self.CAMERA_FOV)
                        if abs(theta) > self.THETA_THRESHOLD:
                            self.send_command_to_robot(theta)
                        else:
                            self.send_command_to_robot(0)

                    self.last_command_time = current_time

            # Convert image from BGR to RGB for Tkinter display
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            img_tk = ImageTk.PhotoImage(image=img)

            # Put the frame in the queue
            if not self.frame_queue.full():
                self.frame_queue.put(img_tk)

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
            self.root.after(10, self.process_frame_queue)  # Continue processing

    def compute_theta(self, x_diff, frame_width, fov=60):
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
        return theta

    def send_command_to_robot(self, theta):
        """
        Send a command to the Arduino in the format "1 theta".

        Args:
            theta (float): The angle to send to the Arduino.
        """
        command = f"1 {theta}"
        self.arduino.send_command(command)

    def detect_red_circle(self, frame):
        """Detect red circles in the frame."""
        blurred = cv2.GaussianBlur(frame, (5, 5), 2)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Red color range in HSV from calibrated values
        lower_red1 = self.hsv_values['lower1']
        upper_red1 = self.hsv_values['upper1']
        lower_red2 = self.hsv_values['lower2']
        upper_red2 = self.hsv_values['upper2']

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.add(mask1, mask2)

        return self.detect_object(frame, mask_red, is_circle=True)

    def detect_blue_rectangle(self, frame):
        """Detect blue rectangles in the frame."""
        blurred = cv2.GaussianBlur(frame, (5, 5), 2)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Blue color range in HSV from calibrated values
        lower_blue = self.hsv_values['lower_blue']
        upper_blue = self.hsv_values['upper_blue']
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Morphological transformations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

        return self.detect_object(frame, mask_blue, is_circle=False)

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
