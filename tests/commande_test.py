#!/usr/bin/env python3


import pytest
from unittest import mock
import tkinter as tk
from geometry_msgs.msg import Twist
import rospy
from commande import Commande
from mouvement import Mouvement

@pytest.fixture
def setup():
    root = tk.Tk()
    mouvement = mock.Mock(spec=Mouvement)
    commande = Commande(root, mouvement)
    return root, commande, mouvement

def test_move_forward(setup):
    root, commande, mouvement = setup

    with mock.patch.object(commande.velocity_publisher, 'publish') as mock_publish:
        commande.move_forward()
        assert commande.current_twist.linear.x == 0.5
        assert commande.current_twist.angular.z == 0
        
        # Vérifiez si le dernier appel à publish est avec les valeurs attendues
        last_call = mock_publish.call_args_list[-1]
        assert last_call == mock.call(commande.current_twist)

def test_move_backward(setup):
    root, commande, mouvement = setup

    with mock.patch.object(commande.velocity_publisher, 'publish') as mock_publish:
        commande.move_backward()
        assert commande.current_twist.linear.x == -0.5
        assert commande.current_twist.angular.z == 0
        
        # Vérifiez si le dernier appel à publish est avec les valeurs attendues
        last_call = mock_publish.call_args_list[-1]
        assert last_call == mock.call(commande.current_twist)

def test_turn_left(setup):
    root, commande, mouvement = setup

    with mock.patch.object(commande.velocity_publisher, 'publish') as mock_publish:
        commande.turn_left()
        assert commande.current_twist.linear.x == 0
        assert commande.current_twist.angular.z == -1
        
        # Vérifiez si le dernier appel à publish est avec les valeurs attendues
        last_call = mock_publish.call_args_list[-1]
        assert last_call == mock.call(commande.current_twist)

def test_turn_right(setup):
    root, commande, mouvement = setup

    with mock.patch.object(commande.velocity_publisher, 'publish') as mock_publish:
        commande.turn_right()
        assert commande.current_twist.linear.x == 0
        assert commande.current_twist.angular.z == 1
        
        # Vérifiez si le dernier appel à publish est avec les valeurs attendues
        last_call = mock_publish.call_args_list[-1]
        assert last_call == mock.call(commande.current_twist)

def test_stop_robot(setup):
    root, commande, mouvement = setup

    with mock.patch.object(commande.velocity_publisher, 'publish') as mock_publish:
        commande.stop_robot()
        assert commande.current_twist.linear.x == 0
        assert commande.current_twist.angular.z == 0
        assert mouvement.running == False
        assert mouvement.reset_position == False
        
        # Vérifiez si le dernier appel à publish est avec les valeurs attendues
        last_call = mock_publish.call_args_list[-1]
        assert last_call == mock.call(commande.current_twist)
