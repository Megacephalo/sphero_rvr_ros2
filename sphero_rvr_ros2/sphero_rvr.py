#!/usr/bin/env python3

from math import degrees
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import asyncio
from .sphero_sdk import SpheroRvrAsync
from .sphero_sdk import SerialAsyncDal
from .sphero_sdk import SpheroRvrTargets
from .sphero_sdk import ControlSystemTypesEnum
from .sphero_sdk import ControlSystemIdsEnum

class Sphero_RVRP_Node(Node):
    def __init__(self)->None:
        super().__init__('sphero_rvrp_node')
        self.get_logger().info('Launching sphero_rvrp_node.')

        self._t_stop = threading.Event()

        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        self.declare_parameter('port_id', '/dev/ttyUSB0')
        self._port_id = self.get_parameter('port_id').get_parameter_value().string_value

        self._loop = asyncio.get_event_loop()
        self._rvr = SpheroRvrAsync(
            dal=SerialAsyncDal(
                loop=self._loop,
                port_id=self._port_id)
        )
        self._is_stopped: bool = False
        self._cmd_sign: int = 1

        

        # Create subscriber to cmd_vel topic
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, qos_profile=10)


    def initialize_rover(self)->bool:
        try:
            self._loop.run_until_complete(self.rvr_init())  
            return True
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            return False

    async def rvr_init(self)->None:
        await self._rvr.wake()
        await asyncio.sleep(2) # Make sure that the robot really wakes up

        echo_response = await asyncio.wait_for(
            self._rvr.echo(
                data=[3, 1, 4],
                target=SpheroRvrTargets.primary.value
            ), timeout=10.0
        )
        self.get_logger().info(f'Echo response: {echo_response}')

        # Register the handler for the stopped notification
        await self._rvr.on_robot_has_stopped_notify(handler=self.stopped_handler)

        await self._rvr.reset_yaw()
        await self._rvr.reset_locator_x_and_y()
        
        # set timeout for stopping
        control_system_type = ControlSystemTypesEnum.control_system_type_rc_drive
        controller_id = ControlSystemIdsEnum.rc_drive_slew_mode
        await self._rvr.set_default_control_system_for_type(
            control_system_type=control_system_type,
            controller_id=controller_id,
        )
        await self._rvr.set_custom_control_system_timeout(command_timeout=250)

    async def stopped_handler(self, event)->None:
        self._is_stopped = True
    

    def cmd_vel_callback(self, msg)->None:
        # self.get_logger().info(f'Received a Twist message: linear.x={msg.linear.x}, angular.z={msg.angular.z}')
        angular_vel_deg = degrees(msg.angular.z)

        self.get_logger().info(f'linear velocity={msg.linear.x} m/s, angular velocity ={angular_vel_deg} deg/sec')

        asyncio.create_task(self._rvr.drive_rc_si_units(
            linear_velocity=msg.linear.x,
            yaw_angular_velocity=angular_vel_deg,
            flags=0
        ))

        self._cmd_sign = 1 if msg.linear.x >= 0 else -1
    
    # ========================================================
    # Thread for processing serial communication with Sphero
    # ========================================================
    def run(self)->None:
        asyncio.ensure_future(self.setup_listener())
        self._logger.info('Ready to roll')
        while not self._t_stop.is_set():
            try:
                self._loop.run_forever()
            except KeyboardInterrupt:
                self._logger.info('Sphero thread detected keyboard interruption')
                self._t_stop.set()
            except Exception as e:
                self._logger.error(f'Exception processing Sphero message: {e}')
        if self._loop.is_running():
            self._loop.close()
        self._loop.run_until_complete(
            asyncio.gather(
                self._rvr.drive_stop(),
                self._rvr.restore_initial_default_control_systems(),
                self._rvr.restore_default_control_system_timeout(),
                self._rvr.sensor_control_clear(),
                self._rvr.close()
            )
        )
        self._logger.info('Sphero thread is terminated')

    async def setup_listener(self)->None:
        try:
            await self._rvr.sensor_control.start(interval=75)

            asyncio.create_task(self.ros2_spin_once())
            self._logger.info('Listener in place')
        except Exception as e:
            err_msg = f'Error setting up listener: {e}'
            self.get_logger().error(err_msg)
            raise Exception(err_msg)

    async def ros2_spin_once(self)->None:
        while self._loop.is_running():
            try:
                rclpy.spin_once(self, timeout_sec=0)
                await asyncio.sleep(0.05)
            except Exception as e:
                self._logger.warn(f'Exception pumping ROS messaages: {e}')