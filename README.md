# Turtle_Bot_Otarie
Projet python du groupe des Otaries composé de :

Ligne ajoutée pour tester git (commit pull ...)

Objectif: visualisr les mouvements du robot et commander ses déplacmeents

- Interfa ce graphique ( TKinter )
    bouton start
    icone qui indique que la réception est actif
    Subscriber
    visualiser ( position, vitesse ) -> point x,y
    bouton stop ( icone qui indique la fin de la réception ) -> position 0,0

-Interface utilisateur
    Zone 1 : connexion avec robot
           champ de texte pour se connecter ( ROS Master Uri , ROS Hostname)
           bouton start / stop 
           icone

    Zone 2 : Mouvement du robot
          Vitesse linéaire et angulaire sur des graphiques ( package matplotlib )
          Valeur s'actualise 2 fois par secondes

    Zone 3 : Commande
        Anvancer
        Reculer
        Gauche
        Droite
        Slider ( vitessa angulaire et linéaire ) -> vitesse à 0 quand on relache le bouton
        
        
           
        
    
- ROS, connexion robot
- 
