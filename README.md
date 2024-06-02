# Turtle_Bot_Otarie
Projet python du groupe des Otaries composé de :

Ligne ajoutée pour tester git (commit pull ...)

Objectif: visualiser les mouvements du robot et commander ses déplacments

- Interface graphique ( TKinter )
    bouton start
    icone qui indique que la réception est active
    Subscriber ROS et publisher
    visualiser ( position, vitesse ) -> point x,y
    bouton stop ( icone qui indique la fin de la réception ) -> position 0,0

-Interface utilisateur
   
    Zone 1 : connexion avec robot
           champ de texte pour se connecter ( ROS Master Uri , ROS Hostname)
           bouton start / stop 
           icone
           
    Zone 2 : Mouvement du robot
          Vitesse linéaire et angulaire sur des graphiques ( package matplotlib )
          Position du robot et conservation de trajectoire sur un plan 2D
          Valeur s'actualise 2 fois par secondes

    Zone 3 : Commande (la pression sur les boutons à la priorité sur les commandes des sliders.)
        Boutons :
            Arrêter (replace les sliders à 0)
            Anvancer 
            Reculer
            Gauche
            Droite

        Slider :
            vitesse angulaire
            Vitesse linéaire 
        
    
- ROS, connexion robot
