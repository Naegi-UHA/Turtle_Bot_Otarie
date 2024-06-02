#!/usr/bin/env python3

import tkinter as tk
from connexion import Connexion
from mouvement import Mouvement
from commande import Commande
import rospy

if __name__ == "__main__":
    root = tk.Tk()
    root.title("Robot Interface")

    # Initialisation de la connexion ROS et lancement des outils ROS
    connexion = Connexion(root, None, None)

    # Initialisation du nœud ROS après le lancement de roscore
    rospy.init_node('robot_interface', anonymous=True)

    mouvement = Mouvement(root)
    commande = Commande(root, mouvement)

    # Définition des callbacks pour démarrer et arrêter le robot
    connexion.start_callback = commande.start_robot
    connexion.stop_callback = connexion.stop_connect_robot

    root.mainloop()
