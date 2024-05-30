import tkinter as ttk
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

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.5
        self.velocity_publisher.publish(twist)

    def move_backward(self):
        twist = Twist()
        twist.linear.x = -0.5
        self.velocity_publisher.publish(twist)

    def turn_left(self):
        twist = Twist()
        twist.angular.z = 0.5
        self.velocity_publisher.publish(twist)

    def turn_right(self):
        twist = Twist()
        twist.angular.z = -0.5
        self.velocity_publisher.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.velocity_publisher.publish(twist)

    def start_robot(self):
        self.mouvement.running = True
        self.mouvement.reset_position = True

    def stop_robot(self):
        self.mouvement.running = False
        self.mouvement.reset_position = False


