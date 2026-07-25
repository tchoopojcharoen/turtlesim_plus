#!/usr/bin/python3

import rclpy
import py_trees_ros

from turtlesim_plus_bt.tree import create_root


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('forage_bt')
    node.declare_parameter('turtle_name', 'turtle1')
    turtle_name = node.get_parameter('turtle_name').value

    root = create_root(turtle_name=turtle_name)
    tree = py_trees_ros.trees.BehaviourTree(root)
    try:
        tree.setup(node=node, timeout=15)
    except Exception as exc:
        node.get_logger().error(f'failed to setup the tree, aborting [{exc}]')
        rclpy.try_shutdown()
        return

    tree.tick_tock(period_ms=100)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
