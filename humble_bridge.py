#!/usr/bin/env python3

import json
import zmq
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

ROBOT_NAME = "/robot_namespace"
HUMBLE_IP = "192.168.131.5"
JAZZY_IP = "192.168.131.1"

HUMBLE_ODOM_OUT_TOPIC = "/ros2_odom_out"
HUMBLE_ODOM_FILTERED_OUT_TOPIC = "/ros2_odom_filtered_out"
HUMBLE_CMD_VEL_OUT_TOPIC = "/ros2_cmd_vel_out"
HUMBLE_IMU_OUT_TOPIC = "/ros2_imu_data"

CMD_VEL_IN_TOPIC = f"{ROBOT_NAME}/cmd_vel"



def jsonOdomMsgToRos(obj):
    msg = Odometry()
    msg.header.stamp.sec = obj['data']['header']['stamp']['sec']
    msg.header.stamp.nanosec = obj['data']['header']['stamp']['nanosec']
    msg.header.frame_id = obj['data']['header']['frame_id']
    msg.child_frame_id = obj['data']['child_frame_id']
    msg.pose.pose.position.x = obj['data']['pose']['pose']['position']['x']
    msg.pose.pose.position.y = obj['data']['pose']['pose']['position']['y']
    msg.pose.pose.position.z = obj['data']['pose']['pose']['position']['z']
    msg.pose.pose.orientation.x = obj['data']['pose']['pose']['orientation']['x']
    msg.pose.pose.orientation.y = obj['data']['pose']['pose']['orientation']['y']
    msg.pose.pose.orientation.z = obj['data']['pose']['pose']['orientation']['z']
    msg.pose.pose.orientation.w = obj['data']['pose']['pose']['orientation']['w']
    msg.twist.twist.linear.x = obj['data']['twist']['twist']['linear']['x']
    msg.twist.twist.linear.y = obj['data']['twist']['twist']['linear']['y']
    msg.twist.twist.linear.z = obj['data']['twist']['twist']['linear']['z']
    msg.twist.twist.angular.x = obj['data']['twist']['twist']['angular']['x']
    msg.twist.twist.angular.y = obj['data']['twist']['twist']['angular']['y']
    msg.twist.twist.angular.z = obj['data']['twist']['twist']['angular']['z']
    return msg


def jsonImuMsgToRos(obj):
    msg = Imu()
    msg.header.stamp.sec = obj['data']['header']['stamp']['sec']
    msg.header.stamp.nanosec = obj['data']['header']['stamp']['nanosec']
    msg.header.frame_id = obj['data']['header']['frame_id']
    msg.orientation.x = obj['data']['orientation']['x']
    msg.orientation.y = obj['data']['orientation']['y']
    msg.orientation.z = obj['data']['orientation']['z']
    msg.orientation.w = obj['data']['orientation']['w']
    msg.angular_velocity.x = obj['data']['angular_velocity']['x']
    msg.angular_velocity.y = obj['data']['angular_velocity']['y']
    msg.angular_velocity.z = obj['data']['angular_velocity']['z']
    msg.linear_acceleration.x = obj['data']['linear_acceleration']['x']
    msg.linear_acceleration.y = obj['data']['linear_acceleration']['y']
    msg.linear_acceleration.z = obj['data']['linear_acceleration']['z']
    return msg


def jsonCmdVelStampedMsgToTwist(obj):
    """Convert Jazzy's TwistStamped JSON into plain Twist for Humble."""
    msg = Twist()
    msg.linear.x = obj['data']['linear']['x']
    msg.linear.y = obj['data']['linear']['y']
    msg.linear.z = obj['data']['linear']['z']
    msg.angular.x = obj['data']['angular']['x']
    msg.angular.y = obj['data']['angular']['y']
    msg.angular.z = obj['data']['angular']['z']
    return msg

class Ros2Bridge(Node):
    def __init__(self):
        super().__init__('ros2_bridge')

        # Initialize ZMQ
        self.zmq_context = zmq.Context()

        # Subscriber socket (from Jazzy)
        self.zmq_sub = self.zmq_context.socket(zmq.SUB)
        self.zmq_sub.connect(f"tcp://{JAZZY_IP}:5555")
        self.zmq_sub.setsockopt_string(zmq.SUBSCRIBE, "")

        # Publisher socket (to Jazzy)
        self.zmq_pub = self.zmq_context.socket(zmq.PUB)
        self.zmq_pub.bind(f"tcp://{HUMBLE_IP}:5556")

        # ROS 2 publishers
        self.odom_pub = self.create_publisher(Odometry, HUMBLE_ODOM_OUT_TOPIC, 10)
        self.odom_filtered_pub = self.create_publisher(Odometry, HUMBLE_ODOM_FILTERED_OUT_TOPIC, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, HUMBLE_CMD_VEL_OUT_TOPIC, 10)
        self.imu_pub = self.create_publisher(Imu, HUMBLE_IMU_OUT_TOPIC, 10)

        # ROS 2 subscriber (cmd_vel Twist input)
        self.cmd_vel_sub = self.create_subscription(
            Twist, CMD_VEL_IN_TOPIC, self.cmd_vel_callback, 10)

        # Timer to poll ZMQ
        self.create_timer(0.01, self.check_zmq_messages)

        self.get_logger().info("Humble bridge node initialized")

    def cmd_vel_callback(self, msg):
        data = cmdVelMsgToJson(msg, CMD_VEL_IN_TOPIC)
        self.get_logger().debug(f"Publishing cmd_vel JSON: {data}")
        self.zmq_pub.send_string(json.dumps(data))

    def check_zmq_messages(self):
        try:
            message = self.zmq_sub.recv_string(flags=zmq.NOBLOCK)
            data = json.loads(message)

            if data['topic'] == f'{ROBOT_NAME}/platform/odom':
                msg = jsonOdomMsgToRos(data)
                self.odom_pub.publish(msg)

            elif data['topic'] == f'{ROBOT_NAME}/platform/odom/filtered':
                msg = jsonOdomMsgToRos(data)
                self.odom_filtered_pub.publish(msg)

            elif data['topic'] == f'{ROBOT_NAME}/sensors/imu_0/data':
                msg = jsonImuMsgToRos(data)
                self.imu_pub.publish(msg)

            elif data['topic'] == f'{ROBOT_NAME}/cmd_vel':
                msg = jsonCmdVelStampedMsgToTwist(data)  # Convert TwistStamped → Twist
                self.cmd_vel_pub.publish(msg)

        except zmq.Again:
            pass
        except Exception as e:
            self.get_logger().error(f"Error processing ZMQ message: {str(e)}")


def main():
    rclpy.init()
    bridge = Ros2Bridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
