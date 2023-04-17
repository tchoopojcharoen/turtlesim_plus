#!/usr/bin/python3

# other libraries

# package module
from turtlesim_plus.ros2_plugins import TurtlesimPlusNode

# RCLPY libraries, classes, functions
import rclpy

# ROS Package

# Define the main function that will run when this script is execute
def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimPlusNode()
    try:
        while rclpy.ok(): # while the node isn't shut down
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node has stopped cleanly.')
    except SystemExit:
        node.get_logger().info('Node is complete.')
    except BaseException as exc:
        type = exc.__class__.__name__
        node.get_logger().error(f'{type} exception in node has occured.')
        raise # raise without argument = raise the last exception
    finally:
        node.destroy_node()
        rclpy.shutdown() 

# If this script is being run as the main program, call the main function
if __name__=='__main__':
    main()
