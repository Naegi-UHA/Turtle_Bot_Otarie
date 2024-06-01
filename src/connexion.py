#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import rospy
import subprocess
import time
import os
import signal

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
            self.connection_frame, text="STOP_CONNECT", command=self.stop_connect_robot
        )
        self.stop_button.grid(row=2, column=1, pady=5)

        self.status_label = ttk.Label(
            self.connection_frame, text="Status: Disconnected", foreground="red"
        )
        self.status_label.grid(row=3, column=0, columnspan=2, pady=5)

        # Automatically start ROS core and launch Gazebo simulation
        self.start_ros_core()
        self.launch_gazebo_simulation()
        self.launch_rviz()

    def start_ros_core(self):
        self.status_label.config(text="Status: Starting ROS core...", foreground="orange")
        self.root.update_idletasks()

        # Start roscore
        self.roscore_process = subprocess.Popen(['roscore'], preexec_fn=os.setsid)

        # Wait for roscore to initialize
        while not self.is_roscore_running():
            time.sleep(1)
        
        self.status_label.config(text="Status: ROS core started", foreground="green")

    def is_roscore_running(self):
        try:
            output = subprocess.check_output(['rostopic', 'list'])
            return True
        except subprocess.CalledProcessError:
            return False

    def launch_gazebo_simulation(self):
        self.status_label.config(text="Status: Launching Gazebo simulation...", foreground="orange")
        self.root.update_idletasks()

        # Launch Gazebo simulation
        self.gazebo_process = subprocess.Popen(['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_empty_world.launch'], preexec_fn=os.setsid)
        self.status_label.config(text="Status: Gazebo simulation launched", foreground="green")

    def launch_rviz(self):
        self.status_label.config(text="Status: Launching RViz...", foreground="orange")
        self.root.update_idletasks()

        # Launch RViz
        self.rviz_process = subprocess.Popen(['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_gazebo_rviz.launch'], preexec_fn=os.setsid)
        self.status_label.config(text="Status: RViz launched", foreground="green")

    def start_robot(self):
        self.status_label.config(text="Status: Connecting...", foreground="orange")
        self.root.update_idletasks()

        # Call the callback to start the robot control
        self.start_callback()
        self.status_label.config(text="Status: Connected", foreground="green")

    def stop_connect_robot(self):
        # Stop Gazebo simulation
        if hasattr(self, 'gazebo_process'):
            os.killpg(os.getpgid(self.gazebo_process.pid), signal.SIGTERM)
        
        # Stop RViz
        if hasattr(self, 'rviz_process'):
            os.killpg(os.getpgid(self.rviz_process.pid), signal.SIGTERM)
        
        # Stop roscore
        if hasattr(self, 'roscore_process'):
            os.killpg(os.getpgid(self.roscore_process.pid), signal.SIGTERM)

        # Call the callback to stop the robot control
        self.stop_callback()

        self.status_label.config(text="Status: Disconnected", foreground="red")
