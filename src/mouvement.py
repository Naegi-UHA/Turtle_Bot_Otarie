#!/usr/bin/env python3

import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import rospy
from nav_msgs.msg import Odometry
import logging

# Définition des chemins pour les fichiers de données
VELOCITIES_PATH = Path(__file__).parent / "velocities.json"
POSES_PATH = Path(__file__).parent / "poses.json"

# Configuration du logging
logging.basicConfig(filename="robot_log.log", level=logging.DEBUG, 
                    format="%(asctime)s - %(levelname)s - %(message)s")

class Mouvement:
    def __init__(self, root):
        # Création des figures pour l'affichage des graphes
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1)
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().grid(row=1, column=0, padx=10, pady=10)

        self.running = False
        self.reset_position = False

        self.velocities = []
        self.poses = []

        # Abonnement au topic /odom pour recevoir les messages de type Odometry
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Configuration de l'animation pour mettre à jour les graphes
        self.ani = FuncAnimation(self.fig, self.update_graph, interval=500)  # Mise à jour toutes les 500 ms

    def odom_callback(self, data):
        if self.running:
            linear = data.twist.twist.linear.x
            angular = data.twist.twist.angular.z
            self.velocities.append({"linear": linear, "angular": angular})
            
            position = data.pose.pose.position
            self.poses.append({"x": position.x, "y": position.y})

            self.save_data()
            logging.debug(f"Vx={linear}, Ax={angular}")

    def save_data(self):
        # Sauvegarde des vitesses et des positions dans des fichiers JSON
        with open(VELOCITIES_PATH, "w") as f:
            json.dump(self.velocities, f)
        with open(POSES_PATH, "w") as f:
            json.dump(self.poses, f)

    def read_velocities(self):
        # Lecture des vitesses à partir du fichier JSON
        try:
            with open(VELOCITIES_PATH, "r") as f:
                data = json.load(f)
                if not isinstance(data, list):
                    raise ValueError("Invalid data format")
                linear_velocity_data = [entry["linear"] for entry in data]
                angular_velocity_data = [entry["angular"] for entry in data]
        except (FileNotFoundError, json.JSONDecodeError, ValueError) as e:
            print(f"Error reading velocities: {e}")
            linear_velocity_data = []
            angular_velocity_data = []

        return linear_velocity_data, angular_velocity_data

    def read_poses(self):
        # Lecture des positions à partir du fichier JSON
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

        self.ax1.clear()
        self.ax2.clear()

        if self.reset_position:
            x_positions = [0]
            y_positions = [0]
            self.reset_position = False
        else:
            linear_velocity_data, angular_velocity_data = self.read_velocities()
            x_positions, y_positions = self.read_poses()

            t = np.arange(len(linear_velocity_data))

            self.ax1.plot(t, linear_velocity_data, label="Vitesse linéaire")
            self.ax1.plot(t, angular_velocity_data, label="Vitesse angulaire")
            self.ax1.set_title("Vitesse linéaire et angulaire")
            self.ax1.legend()

        self.ax2.plot(x_positions, y_positions, label="Position")
        self.ax2.set_title("Position dans un plan 2D")
        self.ax2.legend()

        self.canvas.draw()

    def start_logging(self):
        logging.info("Robot connecté")

    def stop_logging(self):
        logging.info("Robot déconnecté")
