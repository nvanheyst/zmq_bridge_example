# zmq_bridge_example

Example Template

Data from ROS2 topics will be mapped to ROS1 as follows:
/robot_namespace/platform/odom -> /ros2_odom_out (nav_msgs/Odometry)
/robot_namespace/sensors/imu_0/data -> /ros2_imu_out (sensor_msgs/Imu)
/robot_namepace/platform/cmd_vel -> /ros2_cmd_vel_out (For debugging)
Data from ROS1 topics will be mapped to ROS2 as follows:
/cmd_vel -> /robot_namespace/cmd_vel (geometry_msgs/Twist)


Instructions

Update ROS1 / ROS2_IP constants to your IP (if everything is on the same computer localhost should work)
Update ROBOT_NAME constant (just do / if you don't want a name)
Update CMD_VEL_OUT_TOPIC in ros2_bridge to be whatever topic you're using to command in ros2 (will publish a TwistStamped)
Update CMD_VEL_IN_TOPIC in ros1_bridge to be whatever ROS1 topic you're publishing to for command (Twist)

You'll need the zmq bindings for Python, which should be sudo apt install python3-zmq on Jazzy and pip3 install pyzmq on noetic
