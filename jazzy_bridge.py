#!/usr/bin/env python3

import json
import zmq
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

ROBOT_NAME = "/robot_namespace"
HUMBLE_IP = "192.168.131.5"
JAZZY_IP = "192.168.131.1"

ROS2_ODOM_TOPIC = f"{ROBOT_NAME}/platform/odom"
ROS2_ODOM_FILTERED_TOPIC = f"{ROBOT_NAME}/platform/odom/filtered"
ROS2_IMU_TOPIC = f"{ROBOT_NAME}/sensors/imu_0/data"
CMD_VEL_OUT_TOPIC = f"{ROBOT_NAME}/cmd_vel"


def odomMsgToJson(msg, topic):
    data = {
        'topic': topic,
        'data': {
            'header': {
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec
                },
                'frame_id': msg.header.frame_id
            },
            'child_frame_id': msg.child_frame_id,
            'pose': {
                'pose': {
                    'position': {
                        'x': msg.pose.pose.position.x,
                        'y': msg.pose.pose.position.y,
                        'z': msg.pose.pose.position.z
                    },
                    'orientation': {
                        'x': msg.pose.pose.orientation.x,
                        'y': msg.pose.pose.orientation.y,
                        'z': msg.pose.pose.orientation.z,
                        'w': msg.pose.pose.orientation.w
                    }
                }
            },
            'twist': {
                'twist': {
                    'linear': {
                        'x': msg.twist.twist.linear.x,
                        'y': msg.twist.twist.linear.y,
                        'z': msg.twist.twist.linear.z
                    },
                    'angular': {
                        'x': msg.twist.twist.angular.x,
                        'y': msg.twist.twist.angular.y,
                        'z': msg.twist.twist.angular.z
                    }
                }
            }
        }
    }
    return data


def imuMsgToJson(msg, topic):
    data = {
        'topic': topic,
        'data': {
            'header': {
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec
                },
                'frame_id': msg.header.frame_id
            },
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            }
        }
    }
    return data


class Ros2Bridge(Node):
    def __init__(self):
        super().__init__('ros2_bridge')

        self.zmq_context = zmq.Context()

        # Publisher socket for sending data to Humble
        self.zmq_pub = self.zmq_context.socket(zmq.PUB)
        self.zmq_pub.bind(f"tcp://{JAZZY_IP}:5555")

        # Subscriber socket for receiving commands from Humble
        self.zmq_sub = self.zmq_context.socket(zmq.SUB)
        self.zmq_sub.connect(f"tcp://{HUMBLE_IP}:5556")
        self.zmq_sub.setsockopt_string(zmq.SUBSCRIBE, "")

        # ROS2 Jazzy subscribers
        self.odom_sub = self.create_subscription(
            Odometry, ROS2_ODOM_TOPIC, self.odom_callback, 10)

        self.odom_filtered_sub = self.create_subscription(
            Odometry, ROS2_ODOM_FILTERED_TOPIC, self.odom_filtered_callback, 10)

        self.imu_sub = self.create_subscription(
            Imu, ROS2_IMU_TOPIC, self.imu_callback, 10)

        # ROS2 Jazzy publisher for received commands
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, CMD_VEL_OUT_TOPIC, 10)

        self.create_timer(0.01, self.check_zmq_messages)

        self.get_logger().info('ROS2 bridge node initialized')

    def odom_callback(self, msg):
        self.get_logger().info(
            f'Republishing odom to Humble (x: {round(msg.pose.pose.position.x, 2)}, y: {round(msg.pose.pose.position.y, 2)})')
        data = odomMsgToJson(msg, ROS2_ODOM_TOPIC)
        self.zmq_pub.send_string(json.dumps(data))

    def imu_callback(self, msg):
        data = imuMsgToJson(msg, ROS2_IMU_TOPIC)
        self.zmq_pub.send_string(json.dumps(data))

    def odom_filtered_callback(self, msg):
        data = odomMsgToJson(msg, ROS2_ODOM_FILTERED_TOPIC)
        self.zmq_pub.send_string(json.dumps(data))

    def check_zmq_messages(self):
        try:
            message = self.zmq_sub.recv_string(flags=zmq.NOBLOCK)
            data = json.loads(message)
            print(data)
            if data['topic'] == CMD_VEL_OUT_TOPIC:
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.twist.linear.x = data['data']['linear']['x']
                msg.twist.linear.y = data['data']['linear']['y']
                msg.twist.linear.z = data['data']['linear']['z']
                msg.twist.angular.x = data['data']['angular']['x']
                msg.twist.angular.y = data['data']['angular']['y']
                msg.twist.angular.z = data['data']['angular']['z']
                self.cmd_vel_pub.publish(msg)

        except zmq.Again:
            pass
        except Exception as e:
            self.get_logger().error(f'Error processing ZMQ message: {str(e)}')


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
