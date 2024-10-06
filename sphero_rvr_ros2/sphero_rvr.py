#!/usr/bin/env python3

from math import degrees, radians, sqrt
import threading
import asyncio

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry

from .sphero_sdk import SpheroRvrAsync
from .sphero_sdk import SerialAsyncDal
from .sphero_sdk import SpheroRvrTargets
from .sphero_sdk import ControlSystemTypesEnum
from .sphero_sdk import ControlSystemIdsEnum
from sphero_sdk import RvrStreamingServices

class Sphero_RVRP_Node(Node):
    def __init__(self)->None:
        super().__init__('sphero_rvrp_node')
        self.get_logger().info('Launching sphero_rvrp_node.')

        self._t_stop = threading.Event()
        self._received_msg_types = set()

        self.declare_parameter('cmd_vel topic', 'cmd_vel')
        self.cmd_vel_topic = self.get_parameter('cmd_vel topic').get_parameter_value().string_value

        self.declare_parameter('port ID', '/dev/ttyUSB0',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='Port ID for the Sphero RVR'))
        self._port_id = self.get_parameter('port ID').get_parameter_value().string_value

        self.declare_parameter('IMU frame ID', 'imu_link',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='Frame ID for the IMU'))
        self._imu_frame_id = self.get_parameter('IMU frame ID').get_parameter_value().string_value

        self.declare_parameter('IMU topic name', 'sphero/imu',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='IMU topic name'))
        self._imu_topic_name = self.get_parameter('IMU topic name').get_parameter_value().string_value

        self.declare_parameter('Odom topic name', 'sphero/odom/unfiltered',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='Odometry topic name'))
        self._odom_topic_name = self.get_parameter('Odom topic name').get_parameter_value().string_value

        self.declare_parameter('Odom frame id', 'odom',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='Frame ID for the odometry'))
        self._odom_frame_id = self.get_parameter('Odom frame id').get_parameter_value().string_value

        self.declare_parameter('Odom child frame id', 'base_link',
                               ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='Child frame ID for the odometry'))
        self._odom_child_frame_id = self.get_parameter('Odom child frame id').get_parameter_value().string_value

        self.declare_parameter('Sphero pose topic', '/sphero/pose',
                               ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='Topic name for Sphero pose'))
        self.pose_topic_name = self.get_parameter('Sphero pose topic').get_parameter_value().string_value

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

        # ========================================
        # MU
        self._imu = Imu(header=Header(frame_id=self._imu_frame_id))
        self._imu.orientation_covariance = [1e-6, .0, .0, .0, 1e-6, .0, .0, .0, 1e-6]
        self._imu.angular_velocity_covariance = [1e-6, .0, .0, .0, 1e-6, .0, .0, .0, 1e-6]
        self._imu.linear_acceleration_covariance = [1e-6, .0, .0, .0, 1e-6, .0, .0, .0, 1e-6]
        self._imu_pub = self.create_publisher(Imu, self._imu_topic_name, 10)
        # ========================================

        # ========================================
        # Odometry
        self._acceleration = Vector3()
        self._position = Point()
        self._orientation = Quaternion()
        self._pose = Pose(
            position=self._position,
            orientation=self._orientation
        )

        self._pose_with_covariance = PoseWithCovariance(pose=self._pose)
        self._linear = Vector3()
        self._angular = Vector3()
        self._twist = Twist(linear=self._linear, angular=self._angular)
        self._twist_with_covariance = TwistWithCovariance(twist=self._twist)
        self._odom = Odometry(
            pose=self._pose_with_covariance,
            twist=self._twist_with_covariance,
            header=Header(frame_id=self._odom_frame_id),
            child_frame_id=self._odom_child_frame_id
        )
        self._odom_pub = self.create_publisher(Odometry, self._odom_topic_name, 10)
        # ========================================

        # ========================================
        # Pose
        self._pose_pub = self.create_publisher(Pose, self.pose_topic_name, 10)
        # ========================================

        self.get_logger().info('Sphero RVRP node ready')


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
        self.get_logger().info('Ready to roll')
        while not self._t_stop.is_set():
            try:
                self._loop.run_forever()
            except KeyboardInterrupt:
                self.get_logger().info('Sphero thread detected keyboard interruption')
                self._t_stop.set()
            except Exception as e:
                self.get_logger().error(f'Exception processing Sphero message: {e}')
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
        self.get_logger().info('Sphero thread is terminated')

    # ========================================================
    # Listener for sensor data
    # ========================================================
    async def setup_listener(self)->None:
        try:
            await self._rvr.sensor_control.add_sensor_data_handler(
                service=RvrStreamingServices.locator,
                handler=self.sphero_handler
            )

            await self._rvr.sensor_control.add_sensor_data_handler(
                service=RvrStreamingServices.quaternion,
                handler=self.sphero_handler
            )

            await self._rvr.sensor_control.add_sensor_data_handler(
                service=RvrStreamingServices.gyroscope,
                handler=self.sphero_handler
            )

            await self._rvr.sensor_control.add_sensor_data_handler(
                service=RvrStreamingServices.velocity,
                handler=self.sphero_handler
            )

            await self._rvr.sensor_control.add_sensor_data_handler(
                service=RvrStreamingServices.accelerometer,
                handler=self.sphero_handler
            )

            await self._rvr.sensor_control.add_sensor_data_handler(
                service=RvrStreamingServices.imu,
                handler=self.sphero_handler
            )

            await self._rvr.on_will_sleep_notify(handler=self.sleep_handler)

            await self._rvr.sensor_control.start(interval=75)

            asyncio.create_task(self.ros2_spin_once())
            self.get_logger().info('Listener in place')
        except Exception as e:
            err_msg = f'Error setting up listener: {e}'
            self.get_logger().error(err_msg)
            raise Exception(err_msg)

    def sphero_handler(self, data)->None:
        try:
            received_msg_types = set()
            if 'locator' in data:
                received_msg_types.add('locator')
                # Sphero see forward towards Y as opposed to ROS,
                # which see forward as X. Rotate 90 deg.
                self._position.x, self._position.y, self._position.z = data['locator']['Y'], data['locator']['X'], 0.0

            if 'Quaternion' in data:
                received_msg_types.add('quaternion')
                # Rotate 90 as per above.
                self._orientation.w = data['Quaternion']['W']
                self._orientation.x = data['Quaternion']['X']
                self._orientation.y = data['Quaternion']['Y']
                self._orientation.z = data['Quaternion']['Z']

            if 'gyroscope' in data:
                received_msg_types.add('gyroscope')
                # convert to radians
                self._angular.x = radians(data['gyroscope']['X'])
                self._angular.y = radians(data['gyroscope']['Y'])
                self._angular.z = radians(data['gyroscope']['Z'])

            if 'velocity' in data:
                received_msg_types.add('velocity')
                linear_x, linear_y = data['Velocity']['X'], data['Velocity']['Y']
                msg = self._cmd_sign * sqrt(linear_x**2 + linear_y**2)

                self._linear.x = msg

            if 'Accelerometer' in data:
                received_msg_types.add('accelerometer')
                self._acceleration.x = data['Accelerometer']['X']
                self._acceleration.y = data['Accelerometer']['Y']
                self._acceleration.z = data['Accelerometer']['Z']

            self.check_if_need_to_send_msg(received_msg_types)

        except Exception as e:
            self.get_logger().error(f'Sphero handler exception: {e}')

    def check_if_need_to_send_msg(self, latest_received_msg_types:set[str])->bool:
        self._received_msg_types.update(latest_received_msg_types)
        if self._received_msg_types >= {'locator', 'quaternion', 'gyroscope', 'velocity', 'accelerometer'}:
            self._received_msg_types.clear()
            try:
                self._odom.header.stamp = self.get_clock().now().to_msg()
                self._odom_pub.publish(self._odom)

                self._imu.header.stamp = self._odom.header.stamp
                self._imu.orientation = self._orientation
                self._imu.linear_acceleration = self._acceleration
                self._imu.angular_velocity = self._angular
                self._imu_pub.publish(self._imu)

                self._pose_pub.publish(Pose(position=self._position, orientation=self._orientation))
            except Exception as e:
                self.get_logger().error(f'Transformation publishing error: {e}')

    async def sleep_handler(self)->None:
        # Do not let the RVR sleep while we are connected
        try:
            await self._rvr.wake()
        except Exception as e:
            self.get_logger().error(f'Error waking up the robot: {e}')

    # ========================================================
    # Hook up rclpy with asyncio loop
    # ========================================================
    async def ros2_spin_once(self)->None:
        while self._loop.is_running():
            try:
                rclpy.spin_once(self, timeout_sec=0)
                await asyncio.sleep(0.05)
            except Exception as e:
                self.get_logger().warn(f'Exception pumping ROS messaages: {e}')