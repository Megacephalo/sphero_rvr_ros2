#! /usr/bin/env python3

import rclpy

from .sphero_rvr import Sphero_RVRP_Node

def main()->None:
    rclpy.init()
    rvr = Sphero_RVRP_Node()

    is_initialized = rvr.initialize_rover()
    if not is_initialized:
        rvr.get_logger().error('Failed to initialize the robot.')
        return
    rvr.get_logger().info('Robot is initialized.')

    rvr.run()
    rvr.destroy_node()

    rvr.get_logger().info('Destroying sphero_rvrp_node.')

if __name__ == '__main__':
    main()
