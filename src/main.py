#!/usr/bin/env python3

import tkinter as tk
from connexion import Connexion
from mouvement import Mouvement
from commande import Commande
import rospy

if __name__ == "__main__":
    root = tk.Tk()
    root.title("Robot Interface")

    # Connexion and start ROS core, Gazebo, and RViz
    connexion = Connexion(root, None, None)

    # Ensure ROS is initialized after roscore is running
    rospy.init_node('robot_interface', anonymous=True)

    mouvement = Mouvement(root)
    commande = Commande(root, mouvement)

    # Pass the correct start and stop callbacks to Connexion
    connexion.start_callback = commande.start_robot
    connexion.stop_callback = connexion.stop_connect_robot

    root.mainloop()
