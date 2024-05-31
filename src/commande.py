import tkinter as tk
from tkinter import ttk
from geometry_msgs.msg import Twist
import rospy

class Commande:
    def __init__(self, root, mouvement):
        self.mouvement = mouvement

        self.command_frame = ttk.LabelFrame(root, text="Commandes du Robot")
        self.command_frame.grid(row=2, column=0, padx=10, pady=10, sticky="ew")

        self.forward_button = ttk.Button(self.command_frame, text="Marche Avant", command=self.move_forward)
        self.forward_button.grid(row=0, column=1, padx=5, pady=5)

        self.backward_button = ttk.Button(self.command_frame, text="Marche Arrière", command=self.move_backward)
        self.backward_button.grid(row=1, column=1, padx=5, pady=5)

        self.left_button = ttk.Button(self.command_frame, text="Gauche", command=self.turn_left)
        self.left_button.grid(row=1, column=0, padx=5, pady=5)

        self.right_button = ttk.Button(self.command_frame, text="Droite", command=self.turn_right)
        self.right_button.grid(row=1, column=2, padx=5, pady=5)

        self.stop_button = ttk.Button(self.command_frame, text="Arrêter", command=self.stop_robot)
        self.stop_button.grid(row=2, column=1, padx=5, pady=5)

        ttk.Label(self.command_frame, text="Vitesse Linéaire:").grid(row=3, column=0)
        self.linear_speed_slider = ttk.Scale(self.command_frame, from_=-1, to=1, orient=tk.HORIZONTAL, command=self.update_speed)
        self.linear_speed_slider.set(0)
        self.linear_speed_slider.grid(row=3, column=1, columnspan=2, padx=5, pady=5)

        ttk.Label(self.command_frame, text="Vitesse Angulaire:").grid(row=4, column=0)
        self.angular_speed_slider = ttk.Scale(self.command_frame, from_=-1, to=1, orient=tk.HORIZONTAL, command=self.update_speed)
        self.angular_speed_slider.set(0)
        self.angular_speed_slider.grid(row=4, column=1, columnspan=2, padx=5, pady=5)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.current_twist = Twist()

    def move_forward(self):
        self.linear_speed_slider.set(1)
        self.angular_speed_slider.set(0)
        self.current_twist.linear.x = 0.2
        self.publish_velocity()

    def move_backward(self):
        self.linear_speed_slider.set(-1)
        self.angular_speed_slider.set(0)
        self.current_twist.linear.x = -0.2
        self.publish_velocity()

    def turn_left(self):
        self.angular_speed_slider.set(-1)
        self.linear_speed_slider.set(0.2)
        self.current_twist.angular.z = -0.2
        self.publish_velocity()

    def turn_right(self):
        self.angular_speed_slider.set(1)
        self.linear_speed_slider.set(0.2)
        self.current_twist.angular.z = 0.2
        self.publish_velocity()

    def stop_robot(self):
        self.linear_speed_slider.set(0)
        self.angular_speed_slider.set(0)
        self.current_twist.linear.x = 0
        self.current_twist.angular.z = 0
        self.mouvement.running = False
        self.mouvement.reset_position = False
        self.publish_velocity()

    def update_speed(self, event=None):
        self.current_twist.linear.x = self.linear_speed_slider.get()
        self.current_twist.angular.z = self.angular_speed_slider.get()
        self.publish_velocity()

    def publish_velocity(self):
        self.velocity_publisher.publish(self.current_twist)

    def start_robot(self):
        self.mouvement.running = True
        self.mouvement.reset_position = True

