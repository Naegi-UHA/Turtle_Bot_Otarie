import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import json
from pathlib import Path

VELOCITIES_PATH = Path(__file__).parent / "velocities.json"
POSES_PATH = Path(__file__).parent / "poses.json"

# Assuming Odometry and Subscriber classes are defined as in your previous code


# Define a class for the main application
class RobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Interface")

        # Setting up the Robot connection area
        self.connection_frame = ttk.LabelFrame(root, text="Robot Connection")
        self.connection_frame.grid(row=0, column=0, padx=10, pady=10, sticky="ew")

        ttk.Label(self.connection_frame, text="IP:").grid(row=0, column=0)
        self.ip_entry = ttk.Entry(self.connection_frame)
        self.ip_entry.grid(row=0, column=1)

        ttk.Label(self.connection_frame, text="Port:").grid(row=1, column=0)
        self.port_entry = ttk.Entry(self.connection_frame)
        self.port_entry.grid(row=1, column=1)

        self.start_button = ttk.Button(
            self.connection_frame, text="START", command=self.start_robot
        )
        self.start_button.grid(row=2, column=0, pady=5)

        self.stop_button = ttk.Button(
            self.connection_frame, text="STOP", command=self.stop_robot
        )
        self.stop_button.grid(row=2, column=1, pady=5)

        self.status_label = ttk.Label(
            self.connection_frame, text="Status: Disconnected", foreground="red"
        )
        self.status_label.grid(row=3, column=0, columnspan=2, pady=5)

        # Setting up the Robot movement zone
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew")

        self.ani = animation.FuncAnimation(
            self.fig, self.update_graph, interval=500, cache_frame_data=False
        )
        self.ani.event_source.stop()  # Start with animation stopped

        # Setting up the Order zone
        self.order_frame = ttk.LabelFrame(root, text="Order Zone")
        self.order_frame.grid(row=2, column=0, padx=10, pady=10, sticky="ew")

        self.forward_button = ttk.Button(
            self.order_frame, text="Forward", command=self.move_forward
        )
        self.forward_button.grid(row=0, column=1, padx=5, pady=5)

        self.back_button = ttk.Button(
            self.order_frame, text="Back", command=self.move_back
        )
        self.back_button.grid(row=2, column=1, padx=5, pady=5)

        self.rotate_left_button = ttk.Button(
            self.order_frame, text="Rotate Left", command=self.rotate_left
        )
        self.rotate_left_button.grid(row=1, column=0, padx=5, pady=5)

        self.rotate_right_button = ttk.Button(
            self.order_frame, text="Rotate Right", command=self.rotate_right
        )
        self.rotate_right_button.grid(row=1, column=2, padx=5, pady=5)

        ttk.Label(self.order_frame, text="Linear Speed:").grid(row=3, column=0)
        self.linear_speed = tk.DoubleVar(value=1.0)
        self.linear_speed_slider = ttk.Scale(
            self.order_frame,
            from_=0,
            to=5,
            orient=tk.HORIZONTAL,
            variable=self.linear_speed,
        )
        self.linear_speed_slider.grid(row=3, column=1, columnspan=2)

        ttk.Label(self.order_frame, text="Angular Speed:").grid(row=4, column=0)
        self.angular_speed = tk.DoubleVar(value=1.0)
        self.angular_speed_slider = ttk.Scale(
            self.order_frame,
            from_=0,
            to=5,
            orient=tk.HORIZONTAL,
            variable=self.angular_speed,
        )
        self.angular_speed_slider.grid(row=4, column=1, columnspan=2)

        self.running = False
        self.reset_position = False

        # Add a protocol handler for clean exit
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def on_close(self):
        self.running = False  # Stop any running processes
        self.root.quit()  # Stop the mainloop
        self.root.destroy()  # Destroy the window to free up resources

    def start_robot(self):
        # Implement robot start connection logic here
        self.status_label.config(text="Status: Connected", foreground="green")
        self.running = True
        self.reset_position = False
        self.ani.event_source.start()  # Start animation

    def stop_robot(self):
        # Implement robot stop connection logic here
        self.status_label.config(text="Status: Disconnected", foreground="red")
        self.running = False
        self.reset_position = True
        self.ani.event_source.stop()  # Stop animation
        self.update_graph(0)  # Update graph to show robot at (0,0)

    def move_forward(self):
        # Implement robot move forward logic here
        print("Moving Forward")

    def move_back(self):
        # Implement robot move back logic here
        print("Moving Back")

    def rotate_left(self):
        # Implement robot rotate left logic here
        print("Rotating Left")

    def rotate_right(self):
        # Implement robot rotate right logic here
        print("Rotating Right")

    # def update_graph(self):
    #     # Update the graph with the latest robot data
    #     if self.running:
    #         self.ax1.clear()
    #         self.ax2.clear()
    #         # Generate some example data
    #         t = np.arange(0.0, 2.0, 0.01)
    #         s1 = np.sin(2 * np.pi * t)
    #         s2 = np.cos(2 * np.pi * t)

    #         self.ax1.plot(t, s1)
    #         self.ax1.set_title("Linear and Angular Velocity")

    #         self.ax2.plot(t, s2)
    #         self.ax2.set_title("Position on 2D Plane")

    #         self.canvas.draw()
    #     self.root.after(500, self.update_graph)  # Update the graph every 500 ms

    def read_velocities(self):
        try:
            with open(VELOCITIES_PATH, "r") as f:
                data = json.load(f)
                linear_velocity_data = data.get("linear", [])
                angular_velocity_data = data.get("angular", [])
        except (FileNotFoundError, json.JSONDecodeError) as e:
            print(f"Error reading velocities: {e}")
            linear_velocity_data = []
            angular_velocity_data = []

        return linear_velocity_data, angular_velocity_data

    def read_poses(self):
        try:
            with open(POSES_PATH, "r") as f:
                data = json.load(f)
                if not isinstance(data, list):
                    raise ValueError("Invalid data format")
                x_positions = [pose["x"] for pose in data]
                y_positions = [pose["y"] for pose in data]
        except (FileNotFoundError, json.JSONDecodeError, ValueError) as e:
            print(f"Error reading poses: {e}")
            x_positions = []
            y_positions = []

        return x_positions, y_positions

    def update_graph(self, frame):
        if not self.running and not self.reset_position:
            return

        # Clear the axes
        self.ax1.clear()
        self.ax2.clear()

        if self.reset_position:
            x_positions = [0]
            y_positions = [0]
            self.reset_position = False
        else:
            # Read velocities from the file
            linear_velocity_data, angular_velocity_data = self.read_velocities()
            x_positions, y_positions = self.read_poses()

            t = np.arange(len(linear_velocity_data))

            # Plot linear and angular velocities
            self.ax1.plot(t, linear_velocity_data, label="Linear Velocity")
            self.ax1.plot(t, angular_velocity_data, label="Angular Velocity")
            self.ax1.set_title("Linear and Angular Velocity")
            self.ax1.legend()

        # Plot robot's position on 2D plane
        self.ax2.plot(x_positions, y_positions, label="Robot Position")
        self.ax2.set_title("Position on 2D Plane")
        self.ax2.legend()

        # Redraw the canvas
        self.canvas.draw()


# Main function to run the GUI
if __name__ == "__main__":
    root = tk.Tk()
    app = RobotGUI(root)
    # app.update_graph()  # Start updating the graph
    root.mainloop()
