#!/usr/bin/env python

from geometry_msgs.msg import TwistStamped, Twist
import json
from nav_msgs.msg import Odometry
import rospy
from sensor_msgs.msg import Imu
import zmq

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
    msg.header.stamp = rospy.Time(
        obj['data']['header']['stamp']['sec'], obj['data']['header']['stamp']['nanosec'])
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
    msg.header.stamp = rospy.Time(
        obj['data']['header']['stamp']['sec'], obj['data']['header']['stamp']['nanosec'])
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


def jsonCmdVelStampedMsgToRos(obj):
    msg = TwistStamped()
    msg.header.stamp = rospy.Time(
        obj['data']['header']['stamp']['sec'], obj['data']['header']['stamp']['nanosec'])
    msg.header.frame_id = obj['data']['header']['frame_id']
    msg.twist.linear.x = obj['data']['linear']['x']
    msg.twist.linear.y = obj['data']['linear']['y']
    msg.twist.linear.z = obj['data']['linear']['z']
    msg.twist.angular.x = obj['data']['angular']['x']
    msg.twist.angular.y = obj['data']['angular']['y']
    msg.twist.angular.z = obj['data']['angular']['z']
    return msg


def cmdVelMsgToJson(msg, topic):
    data = {
        'topic': topic,
        'data': {
            'linear': {
                'x': msg.linear.x,
                'y': msg.linear.y,
                'z': msg.linear.z
            },
            'angular': {
                'x': msg.angular.x,
                'y': msg.angular.y,
                'z': msg.angular.z
            }
        }
    }
    return data


class Ros1Bridge:
    def __init__(self):
        rospy.init_node('ros1_bridge')

        # Initialize ZMQ context and sockets
        self.context = zmq.Context()

        # Subscriber socket for receiving from ROS2
        self.zmq_sub = self.context.socket(zmq.SUB)
        self.zmq_sub.connect(f"tcp://{ROS2_IP}:5555")
        self.zmq_sub.setsockopt_string(zmq.SUBSCRIBE, "")

        # Publisher socket for sending to ROS2
        self.zmq_pub = self.context.socket(zmq.PUB)
        self.zmq_pub.bind(f"tcp://{ROS1_IP}:5556")

        # ROS1 publishers
        self.odom_pub = rospy.Publisher(
            ROS1_ODOM_OUT_TOPIC, Odometry, queue_size=10)
        self.odom_filtered_pub = rospy.Publisher(
            ROS1_ODOM_FILTERED_OUT_TOPIC, Odometry, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher(
            ROS1_CMD_VEL_OUT_TOPIC, TwistStamped, queue_size=10)
        self.imu_pub = rospy.Publisher(ROS1_IMU_OUT_TOPIC, Imu, queue_size=10)

        # ROS1 subscribers
        self.cmd_vel_sub = rospy.Subscriber(
            CMD_VEL_IN_TOPIC, Twist, self.cmd_vel_callback)

        rospy.loginfo('ROS1 bridge node initialized')

    def cmd_vel_callback(self, msg):
        data = cmdVelMsgToJson(msg, CMD_VEL_IN_TOPIC)
        print(f"publishing string: {data}")
        self.zmq_pub.send_string(json.dumps(data))

    def spin(self):
        while not rospy.is_shutdown():
            try:
                message = self.zmq_sub.recv_string(flags=zmq.NOBLOCK)
                data = json.loads(message)
                if data['topic'] == f'{ROBOT_NAME}/platform/odom':
                    msg = jsonOdomMsgToRos(data)
                    self.odom_pub.publish(msg)

                elif data['topic'] == f'{ROBOT_NAME}/platform/odom_filtered':
                    msg = jsonOdomMsgToRos(data)
                    self.odom_filtered_pub.publish(msg)

                elif data['topic'] == f'{ROBOT_NAME}/sensors/imu_0/data':
                    msg = jsonImuMsgToRos(data)
                    self.imu_pub.publish(msg)

                elif data['topic'] == f'{ROBOT_NAME}/cmd_vel':
                    msg = jsonCmdVelStampedMsgToRos(data)
                    self.cmd_vel_pub.publish(msg)

            except zmq.Again:
                rospy.sleep(0.001)
                continue
            except Exception as e:
                rospy.logerr(f'Error processing ZMQ message: {str(e)}')
                rospy.sleep(0.001)
                continue


def main():
    bridge = Ros1Bridge()
    try:
        bridge.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
