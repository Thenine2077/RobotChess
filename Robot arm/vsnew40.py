import tkinter as tk
from tkinter import messagebox, StringVar, Toplevel, OptionMenu, Button, Label, Entry, Frame
import math
import serial
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from serial.tools import list_ports
import sys
import logging
from scipy.optimize import minimize

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
                self.serial.write(full_command.encode('utf-8'))
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
        # Robotic arm segment lengths in millimeters
        self.OA = 45.5
        self.AB = 36
        self.BC = 104.05
        self.CD = 194.71
        self.DE = 150.22
        self.EF = 164.59
        self.alpha = math.radians(45)
        # Joint angle limits (in degrees)
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

        # Base point (A)
        xA, yA, zA = (0, 0, self.OA)

        # Point B
        xB = self.AB * math.cos(self.alpha) * math.cos(theta1_rad)
        yB = self.AB * math.cos(self.alpha) * math.sin(theta1_rad)
        zB = self.OA + self.AB * math.sin(self.alpha)

        # Point C
        xC = xB + self.BC * math.cos(self.alpha) * math.cos(theta1_rad)
        yC = yB + self.BC * math.cos(self.alpha) * math.sin(theta1_rad)
        zC = zB + self.BC * math.sin(self.alpha)

        # Point D
        xD = xC + self.CD * math.cos(theta2_rad) * math.cos(theta1_rad)
        yD = yC + self.CD * math.cos(theta2_rad) * math.sin(theta1_rad)
        zD = zC + self.CD * math.sin(theta2_rad)

        # Point E
        xE = xD + self.DE * math.cos(theta2_rad + theta3_rad) * math.cos(theta1_rad)
        yE = yD + self.DE * math.cos(theta2_rad + theta3_rad) * math.sin(theta1_rad)
        zE = zD + self.DE * math.sin(theta2_rad + theta3_rad)

        # Point F (end-effector)
        xF = xE + self.EF * math.cos(theta2_rad + theta3_rad + theta4_rad) * math.cos(theta1_rad)
        yF = yE + self.EF * math.cos(theta2_rad + theta3_rad + theta4_rad) * math.sin(theta1_rad)
        zF = zE + self.EF * math.sin(theta2_rad + theta3_rad + theta4_rad)

        # Shift z coordinates so the base is at z = 0
        zA -= self.OA
        zB -= self.OA
        zC -= self.OA
        zD -= self.OA
        zE -= self.OA
        zF -= self.OA

        return [(0, 0, zA), (xB, yB, zB), (xC, yC, zC),
                (xD, yD, zD), (xE, yE, zE), (xF, yF, zF)]

    def inverse_kinematics_optimization(self, x_target, y_target, z_target, gamma, initial_guess,
                                        pos_weight=1.0, ori_weight=0.01):
        """
        Solves inverse kinematics by minimizing a weighted error function.
        
        Args:
            x_target, y_target, z_target (float): Target end-effector position in mm.
            gamma (float): Target orientation (theta2+theta3+theta4) in degrees.
            initial_guess (list): Initial guess for the four joint angles.
            pos_weight (float): Weight for the position error.
            ori_weight (float): Weight for the orientation error.
            
        Returns:
            tuple or None: (theta1, theta2, theta3, theta4) if optimization succeeds; otherwise None.
        """
        def objective(thetas):
            theta1, theta2, theta3, theta4 = thetas
            points = self.calculate_forward_kinematics(theta1, theta2, theta3, theta4)
            xF, yF, zF = points[-1]
            gamma_calc = theta2 + theta3 + theta4
            pos_error = (xF - x_target)**2 + (yF - y_target)**2 + (zF - z_target)**2
            ori_error = (gamma_calc - gamma)**2
            return pos_weight * pos_error + ori_weight * ori_error

        # Joint limits as bounds
        bounds = [
            self.LIMITS['theta1'],
            self.LIMITS['theta2'],
            self.LIMITS['theta3'],
            self.LIMITS['theta4'],
        ]
        
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
        self.setup_ui()

    def setup_ui(self):
        self.root.title("Robotic Arm Kinematics")
        font_style = ('Helvetica', 14)

        # Create frames
        self.left_frame = tk.Frame(self.root)
        self.left_frame.pack(side=tk.LEFT, padx=10, pady=10)

        self.right_frame = tk.Frame(self.root)
        self.right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # Error message label
        self.error_message_label = tk.Label(self.right_frame, text="", fg="red", font=font_style)
        self.error_message_label.pack(side=tk.TOP, pady=5)

        # Matplotlib Figure and canvas for 3D plot
        self.fig = plt.figure(figsize=(12, 9))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.right_frame)
        self.canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        # Input fields and buttons
        self.create_input_fields(font_style)
        self.create_buttons(font_style)

        # Initialize plot with a default configuration
        self.plot_robot_arm(0, 135, -90, -45)

    def create_input_fields(self, font_style):
        self.theta_entries = {}
        # Theta entries for joints
        for theta in ["Theta 1", "Theta 2", "Theta 3", "Theta 4"]:
            tk.Label(self.left_frame, text=theta, font=font_style).pack(pady=(5, 5))
            entry_frame = Frame(self.left_frame)
            entry_frame.pack(pady=(0, 5))
            entry = tk.Entry(entry_frame, font=font_style, justify="center", width=10)
            entry.pack(side=tk.LEFT)
            entry.bind("<KeyRelease>", lambda event, t=theta.lower().replace(" ", ""): self.update_xyz_from_theta())
            self.theta_entries[theta.lower().replace(" ", "")] = entry

            # Increment and decrement buttons for each theta
            increment_button = tk.Button(entry_frame, text="+", command=lambda t=theta.lower().replace(" ", ""): self.increment_theta(t),
                                         font=font_style, width=1, height=1)
            increment_button.pack(side=tk.LEFT)
            decrement_button = tk.Button(entry_frame, text="-", command=lambda t=theta.lower().replace(" ", ""): self.decrement_theta(t),
                                         font=font_style, width=1, height=1)
            decrement_button.pack(side=tk.LEFT)

        # "Move Kinematics" button (to command all joints)
        tk.Button(self.left_frame, text="Move Kinematics", command=self.on_move_all,
                  font=font_style, height=1, width=25).pack(pady=(1, 1))

        # Entries for X, Y, Z, and Angle A (orientation)
        for param in ["X", "Y", "Z", "Angle A"]:
            tk.Label(self.left_frame, text=param, font=font_style).pack(pady=(5, 5))
            entry_frame = Frame(self.left_frame)
            entry_frame.pack(pady=(0, 5))
            entry = tk.Entry(entry_frame, font=font_style, justify="center", width=10)
            entry.pack(side=tk.LEFT)
            entry.bind("<KeyRelease>", lambda event, p=param.lower().replace(" ", ""): self.update_theta_from_xyz())
            self.theta_entries[param.lower().replace(" ", "")] = entry

            increment_button = tk.Button(entry_frame, text="+", command=lambda p=param.lower().replace(" ", ""): self.increment_xyz(p),
                                         font=font_style, width=1, height=1)
            increment_button.pack(side=tk.LEFT)
            decrement_button = tk.Button(entry_frame, text="-", command=lambda p=param.lower().replace(" ", ""): self.decrement_xyz(p),
                                         font=font_style, width=1, height=1)
            decrement_button.pack(side=tk.LEFT)

    def create_buttons(self, font_style):
        # Button to use inverse kinematics (move based on X, Y, Z, Angle A values)
        tk.Button(self.left_frame, text="Move Inverse", command=self.move_to_xyz,
                  font=font_style, height=1, width=25).pack(pady=(1, 1))
        # Home button to return to a default position
        tk.Button(self.left_frame, text="Home", command=self.home_position,
                  font=font_style, height=1, width=25).pack(pady=(0, 1))
        # Stop button to send the stop command ("s") to the Arduino
        tk.Button(self.left_frame, text="Stop", command=lambda: self.arduino.send_command("s"),
                  font=font_style, height=3, width=25, bg='red', fg="white").pack(pady=(0, 1))

    def increment_theta(self, theta_name):
        try:
            current_value = float(self.theta_entries[theta_name].get() or 0)
            new_value = current_value + 1  # Increment by 1 degree
            min_val, max_val = self.kinematics.LIMITS.get(theta_name, (-math.inf, math.inf))
            if new_value > max_val:
                new_value = max_val
            self.theta_entries[theta_name].delete(0, tk.END)
            self.theta_entries[theta_name].insert(0, f"{new_value:.2f}")
            self.update_xyz_from_theta()
        except ValueError:
            pass

    def decrement_theta(self, theta_name):
        try:
            current_value = float(self.theta_entries[theta_name].get() or 0)
            new_value = current_value - 1  # Decrement by 1 degree
            min_val, max_val = self.kinematics.LIMITS.get(theta_name, (-math.inf, math.inf))
            if new_value < min_val:
                new_value = min_val
            self.theta_entries[theta_name].delete(0, tk.END)
            self.theta_entries[theta_name].insert(0, f"{new_value:.2f}")
            self.update_xyz_from_theta()
        except ValueError:
            pass

    def increment_xyz(self, param_name):
        try:
            current_value = float(self.theta_entries[param_name].get() or 0)
            new_value = current_value + 0.1  # Increment by 0.1 cm
            self.theta_entries[param_name].delete(0, tk.END)
            self.theta_entries[param_name].insert(0, f"{new_value:.2f}")
            self.update_theta_from_xyz()
        except ValueError:
            pass

    def decrement_xyz(self, param_name):
        try:
            current_value = float(self.theta_entries[param_name].get() or 0)
            new_value = current_value - 0.1  # Decrement by 0.1 cm
            self.theta_entries[param_name].delete(0, tk.END)
            self.theta_entries[param_name].insert(0, f"{new_value:.2f}")
            self.update_theta_from_xyz()
        except ValueError:
            pass

    def on_move_all(self):
        try:
            theta1 = float(self.theta_entries["theta1"].get() or 0)
            theta2 = float(self.theta_entries["theta2"].get() or 0)
            theta3 = float(self.theta_entries["theta3"].get() or 0)
            theta4 = float(self.theta_entries["theta4"].get() or 0)
            theta_values = {"theta1": theta1, "theta2": theta2, "theta3": theta3, "theta4": theta4}
            if not self.validate_all_thetas(theta_values):
                return
            self.move_all_motors(theta1, theta2, theta3, theta4)
        except ValueError:
            self.error_message_label.config(text="Invalid input in theta fields.")

    def validate_all_thetas(self, theta_values):
        invalid = []
        for angle, value in theta_values.items():
            min_val, max_val = self.kinematics.LIMITS.get(angle, (-math.inf, math.inf))
            if value < min_val or value > max_val:
                invalid.append(angle)
        if invalid:
            if invalid == ['theta4']:
                self.error_message_label.config(text="Warning: Theta 4 exceeds the specified limits")
                return True
            else:
                self.error_message_label.config(text="Cannot set angle beyond limits: " + ", ".join(invalid))
                return False
        self.error_message_label.config(text="")
        return True

    def move_all_motors(self, theta1, theta2, theta3, theta4):
        command_str = f"f {int(round(theta1))} {int(round(theta2))} {int(round(theta3))} {int(round(theta4))}"
        self.arduino.send_command(command_str)
        # Update GUI entries to match the commanded values
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
            theta_values = {"theta1": theta1, "theta2": theta2, "theta3": theta3, "theta4": theta4}
            if not self.validate_all_thetas(theta_values):
                return
            points = self.kinematics.calculate_forward_kinematics(theta1, theta2, theta3, theta4)
            xF, yF, zF = points[-1]
            gamma = theta2 + theta3 + theta4
            self.theta_entries["x"].delete(0, tk.END)
            self.theta_entries["x"].insert(0, f"{xF/10:.2f}")  # Convert mm to cm
            self.theta_entries["y"].delete(0, tk.END)
            self.theta_entries["y"].insert(0, f"{yF/10:.2f}")
            self.theta_entries["z"].delete(0, tk.END)
            self.theta_entries["z"].insert(0, f"{zF/10:.2f}")
            self.theta_entries["anglea"].delete(0, tk.END)
            self.theta_entries["anglea"].insert(0, f"{gamma:.2f}")
            self.plot_robot_arm(theta1, theta2, theta3, theta4)
        except ValueError:
            self.error_message_label.config(text="Invalid numerical input.")

    def update_theta_from_xyz(self):
        try:
            # Convert input values from cm to mm
            x_target = float(self.theta_entries["x"].get() or 0) * 10
            y_target = float(self.theta_entries["y"].get() or 0) * 10
            z_target = float(self.theta_entries["z"].get() or 0) * 10
            gamma = float(self.theta_entries["anglea"].get() or 0)
            initial_guess = [
                float(self.theta_entries["theta1"].get() or 0),
                float(self.theta_entries["theta2"].get() or 0),
                float(self.theta_entries["theta3"].get() or 0),
                float(self.theta_entries["theta4"].get() or 0)
            ]
            results = self.kinematics.inverse_kinematics_optimization(x_target, y_target, z_target, gamma, initial_guess)
            if results:
                theta1, theta2, theta3, theta4 = results
                theta_values = {"theta1": theta1, "theta2": theta2, "theta3": theta3, "theta4": theta4}
                if not self.validate_all_thetas(theta_values):
                    return
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
            x_target = float(self.theta_entries["x"].get() or 0) * 10
            y_target = float(self.theta_entries["y"].get() or 0) * 10
            z_target = float(self.theta_entries["z"].get() or 0) * 10
            gamma = float(self.theta_entries["anglea"].get() or 0)
            initial_guess = [
                float(self.theta_entries["theta1"].get() or 0),
                float(self.theta_entries["theta2"].get() or 0),
                float(self.theta_entries["theta3"].get() or 0),
                float(self.theta_entries["theta4"].get() or 0)
            ]
            results = self.kinematics.inverse_kinematics_optimization(x_target, y_target, z_target, gamma, initial_guess)
            if results:
                theta1, theta2, theta3, theta4 = results
                theta_values = {"theta1": theta1, "theta2": theta2, "theta3": theta3, "theta4": theta4}
                if not self.validate_all_thetas(theta_values):
                    return
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

    def home_position(self):
        self.arduino.send_command("h")
        # Set all joint angles to 0 (home position)
        for theta in ["theta1", "theta2", "theta3", "theta4"]:
            self.theta_entries[theta].delete(0, tk.END)
            self.theta_entries[theta].insert(0, "0")
        self.update_xyz_from_theta()
        self.plot_robot_arm(0, 135, -90, -45)

    def plot_robot_arm(self, theta1, theta2, theta3, theta4):
        self.ax.clear()
        points = self.kinematics.calculate_forward_kinematics(theta1, theta2, theta3, theta4)
        # Plot each segment of the arm
        for i in range(len(points) - 1):
            x_vals = [points[i][0], points[i+1][0]]
            y_vals = [points[i][1], points[i+1][1]]
            z_vals = [points[i][2], points[i+1][2]]
            self.ax.plot(x_vals, y_vals, z_vals, color='blue', linewidth=4)
        # Plot the joint points
        x_coords = [p[0] for p in points]
        y_coords = [p[1] for p in points]
        z_coords = [p[2] for p in points]
        self.ax.scatter(x_coords, y_coords, z_coords, color='red', s=100)
        # Axis labels and view settings
        self.ax.set_xlabel('X-axis (cm)', labelpad=20)
        self.ax.set_ylabel('Y-axis (cm)', labelpad=20)
        self.ax.set_zlabel('Z-axis (cm)', labelpad=20)
        self.ax.set_title("3D Plot of Robotic Arm Position", pad=30)
        self.ax.set_xlim([-500, 500])
        self.ax.set_ylim([-500, 500])
        self.ax.set_zlim([0, 500])
        self.ax.set_box_aspect([1, 1, 1])
        self.ax.view_init(elev=30, azim=60)
        self.canvas.draw()

def select_port_gui(root, arduino_ports):
    # Create a GUI window for Arduino port selection.
    top = Toplevel(root)
    top.title("Select Arduino Port")
    top.geometry("400x150")
    top.resizable(False, False)
    selected_port = StringVar(top)
    selected_port.set(f"{arduino_ports[0][0]} - {arduino_ports[0][1]}")
    dropdown = OptionMenu(top, selected_port, *[f"{p[0]} - {p[1]}" for p in arduino_ports])
    dropdown.pack(pady=20)
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
    selected_port = select_port_gui(root, arduino_ports)
    if not selected_port:
        messagebox.showinfo("Cancel", "You did not select an Arduino port.")
        root.destroy()
        sys.exit()
    arduino_controller.connect(selected_port)
    kinematics = Kinematics()
    root.deiconify()  # Show the main window
    app = RoboticArmGUI(root, arduino_controller, kinematics)
    root.protocol("WM_DELETE_WINDOW", lambda: on_closing(root, arduino_controller, app))
    root.mainloop()

if __name__ == "__main__":
    main()
