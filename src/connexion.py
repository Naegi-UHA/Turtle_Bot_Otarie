import tkinter as tk
from tkinter import ttk
import rospy
import subprocess

class Connexion:
    def __init__(self, root, start_callback, stop_callback):
        self.root = root
        self.start_callback = start_callback
        self.stop_callback = stop_callback

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

    def start_robot(self):
        self.status_label.config(text="Status: Connecting...", foreground="orange")
        self.root.update_idletasks()
        
        # Start roscore
        self.roscore_process = subprocess.Popen(['roscore'])
        rospy.init_node('robot_interface', anonymous=True)
        
        # Launch Gazebo simulation
        self.gazebo_process = subprocess.Popen(['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_empty_world.launch'])

        # Call the callback to start the robot control
        self.start_callback()

        self.status_label.config(text="Status: Connected", foreground="green")

    def stop_robot(self):
        # Stop Gazebo simulation
        if hasattr(self, 'gazebo_process'):
            self.gazebo_process.terminate()
        
        # Stop roscore
        if hasattr(self, 'roscore_process'):
            self.roscore_process.terminate()

        # Call the callback to stop the robot control
        self.stop_callback()

        self.status_label.config(text="Status: Disconnected", foreground="red")

