#!/usr/bin/env python3

import pytest
from unittest import mock
import tkinter as tk
from geometry_msgs.msg import Twist
import rospy
from commande import Commande
from mouvement import Mouvement

# Fixture pour configurer l'environnement de test
@pytest.fixture
def setup():
    # Crée une fenêtre tkinter root
    root = tk.Tk()
    # Crée une instance Mock de Mouvement
    mouvement = mock.Mock(spec=Mouvement)
    # Crée une instance de Commande
    commande = Commande(root, mouvement)
    # Retourne la fenêtre root, l'objet commande et le mock de mouvement
    return root, commande, mouvement

# Test de la fonction move_forward
def test_move_forward(setup):
    root, commande, mouvement = setup

    # Mock de la fonction publish du publisher
    with mock.patch.object(commande.velocity_publisher, 'publish') as mock_publish:
        # Appelle la fonction move_forward
        commande.move_forward()
        # Vérifie que la vitesse linéaire est 0.5 et la vitesse angulaire est 0
        assert commande.current_twist.linear.x == 0.5
        assert commande.current_twist.angular.z == 0
        
        # Vérifie si le dernier appel à publish est avec les valeurs attendues
        last_call = mock_publish.call_args_list[-1]
        assert last_call == mock.call(commande.current_twist)

# Test de la fonction move_backward
def test_move_backward(setup):
    root, commande, mouvement = setup

    with mock.patch.object(commande.velocity_publisher, 'publish') as mock_publish:
        # Appelle la fonction move_backward
        commande.move_backward()
        # Vérifie que la vitesse linéaire est -0.5 et la vitesse angulaire est 0
        assert commande.current_twist.linear.x == -0.5
        assert commande.current_twist.angular.z == 0
        
        # Vérifie si le dernier appel à publish est avec les valeurs attendues
        last_call = mock_publish.call_args_list[-1]
        assert last_call == mock.call(commande.current_twist)

# Test de la fonction turn_left
def test_turn_left(setup):
    root, commande, mouvement = setup

    with mock.patch.object(commande.velocity_publisher, 'publish') as mock_publish:
        # Appelle la fonction turn_left
        commande.turn_left()
        # Vérifie que la vitesse linéaire est 0 et la vitesse angulaire est -1
        assert commande.current_twist.linear.x == 0
        assert commande.current_twist.angular.z == -1
        
        # Vérifie si le dernier appel à publish est avec les valeurs attendues
        last_call = mock_publish.call_args_list[-1]
        assert last_call == mock.call(commande.current_twist)

# Test de la fonction turn_right
def test_turn_right(setup):
    root, commande, mouvement = setup

    with mock.patch.object(commande.velocity_publisher, 'publish') as mock_publish:
        # Appelle la fonction turn_right
        commande.turn_right()
        # Vérifie que la vitesse linéaire est 0 et la vitesse angulaire est 1
        assert commande.current_twist.linear.x == 0
        assert commande.current_twist.angular.z == 1
        
        # Vérifie si le dernier appel à publish est avec les valeurs attendues
        last_call = mock_publish.call_args_list[-1]
        assert last_call == mock.call(commande.current_twist)

# Test de la fonction stop_robot
def test_stop_robot(setup):
    root, commande, mouvement = setup

    with mock.patch.object(commande.velocity_publisher, 'publish') as mock_publish:
        # Appelle la fonction stop_robot
        commande.stop_robot()
        # Vérifie que la vitesse linéaire et angulaire sont à 0
        assert commande.current_twist.linear.x == 0
        assert commande.current_twist.angular.z == 0
        # Vérifie que mouvement.running est False
        assert mouvement.running == False
        # Vérifie que mouvement.reset_position est False
        assert mouvement.reset_position == False
        
        # Vérifie si le dernier appel à publish est avec les valeurs attendues
        last_call = mock_publish.call_args_list[-1]
        assert last_call == mock.call(commande.current_twist)
