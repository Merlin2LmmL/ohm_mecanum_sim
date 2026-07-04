#!/usr/bin/env python3
# ------------------------------------------------------------
# Original Author: Stefan May
# Modifications: Merlin Ortner (2026)
# Date: 01.05.2024 (original), modified 04.07.2026
# Description: Pygame-based robot simulator application for ROS2
# ------------------------------------------------------------
import pygame
import rclpy
from rclpy.node import Node
from ohm_mecanum_sim.ohm_mecanum_simulator import Ohm_Mecanum_Simulator
from rclpy.executors import SingleThreadedExecutor


def main(args=None):
    pygame.init()
    rclpy.init(args=args)

    # Read configuration before the pygame surface and simulator node exist.
    # Uses the same node name as the simulator so a --params-file targets one node.
    param_node = Node("ohm_mecanum_sim")
    param_node.declare_parameter("screen_width", 1600)
    param_node.declare_parameter("screen_height", 900)
    param_node.declare_parameter("outer_border", 5)
    param_node.declare_parameter("inner_border", 300)
    param_node.declare_parameter("robot_names", ["robot1"])
    param_node.declare_parameter("robot_x", [2.0])
    param_node.declare_parameter("robot_y", [2.0])
    param_node.declare_parameter("robot_theta", [0.0])

    width = param_node.get_parameter("screen_width").value
    height = param_node.get_parameter("screen_height").value
    outer_border = param_node.get_parameter("outer_border").value
    inner_border = param_node.get_parameter("inner_border").value
    names = param_node.get_parameter("robot_names").value
    xs = param_node.get_parameter("robot_x").value
    ys = param_node.get_parameter("robot_y").value
    thetas = param_node.get_parameter("robot_theta").value
    param_node.destroy_node()

    if not (len(names) == len(xs) == len(ys) == len(thetas)):
        raise ValueError(
            f"robot_names ({len(names)}), robot_x ({len(xs)}), robot_y ({len(ys)}) "
            f"and robot_theta ({len(thetas)}) must all have the same length"
        )

    size = width, height
    surface = pygame.display.set_mode(size, pygame.HWSURFACE | pygame.DOUBLEBUF)

    sim = Ohm_Mecanum_Simulator(surface, "ohm_mecanum_sim", "Ohm Mecanum Simulator")
    sim.add_rectangle_pixelcoords([outer_border, outer_border], [width - outer_border, height - outer_border])
    sim.add_rectangle_pixelcoords([inner_border, inner_border], [width - inner_border, height - inner_border])
    sim.start_scheduler()

    executor = SingleThreadedExecutor()
    executor.add_node(sim)
    for name, x, y, theta in zip(names, xs, ys, thetas):
        executor.add_node(sim.spawn_robot(x, y, theta, name))

    executor.spin()
    sim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
