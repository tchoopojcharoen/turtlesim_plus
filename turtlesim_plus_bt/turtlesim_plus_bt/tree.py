#!/usr/bin/python3

import py_trees
import rclpy.qos

from turtlesim_plus_bt.behaviours import (
    PizzaDetected, PizzaInEatRange, CallEatService, DriveTowardPizza, Wander,
)


def create_root(turtle_name: str = 'turtle1') -> py_trees.behaviour.Behaviour:
    pizza_detected = PizzaDetected(
        name="Pizza Detected?",
        topic_name=f'/{turtle_name}/scan',
        qos_profile=rclpy.qos.QoSProfile(depth=10),
    )
    pizza_in_range = PizzaInEatRange(name="In Eat Range?")
    call_eat = CallEatService(name="Eat", turtle_name=turtle_name)
    eat_sequence = py_trees.composites.Sequence(
        "Eat", memory=False, children=[pizza_in_range, call_eat])

    drive = DriveTowardPizza(name="Drive Toward Pizza", turtle_name=turtle_name)
    approach_or_eat = py_trees.composites.Selector(
        "Approach or Eat", memory=False, children=[eat_sequence, drive])

    chase_and_eat = py_trees.composites.Sequence(
        "Chase & Eat", memory=False, children=[pizza_detected, approach_or_eat])

    wander = Wander(name="Wander", turtle_name=turtle_name)

    root = py_trees.composites.Selector(
        "Forage", memory=False, children=[chase_and_eat, wander])
    return root
