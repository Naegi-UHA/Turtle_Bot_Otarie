import tkinter as tk
from connexion import Connexion
from mouvement import Mouvement
from commande import Commande
import rospy
from tkinter import ttk

if __name__ == "__main__":
    rospy.init_node('robot_interface', anonymous=True)
    
    root = tk.Tk()
    root.title("Robot Interface")

    mouvement = Mouvement(root)
    commande = Commande(root, mouvement)

    connexion = Connexion(root, commande.start_robot, commande.stop_robot)

    root.mainloop()