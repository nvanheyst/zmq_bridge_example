# zmq_bridge_example

**Description**

This is a POC example demonstrating a simple bridge between ROS2 Humble and ROS2 Jazzy. The template is configured for a ROS2 Humble system to publish and subscribe to topics with a ROS2 Jazzy system. The setup is tested and instructions created assuming a ROS2 Humble system is controlling the ROS2 Jazzy system. The Humble bridge assumes Twist messages, while the Jazzy bridge assumes TwistStamped. 

A ROS Noetic ↔ ROS2 Jazzy bridge is also available: [Noetic & Jazzy branch](https://github.com/nvanheyst/zmq_bridge_example/tree/Noetic%26Jazzy)


This was tested with teleoperation over an ethernet connection. This could work with an autonomy stack and additional topics can be added as well. This hasn't been tested with images or point clouds just the topics shown below. 


**Example Template**

Data from ROS2 Jazzy topics will be mapped to ROS2 Humble as follows:
/robot_namespace/platform/odom -> /ros2_odom_out (nav_msgs/Odometry)

/robot_namespace/platform/odom/filtered -> /ros2_odom_filtered_out (nav_msgs/Odometry)

/robot_namespace/sensors/imu_0/data -> /ros2_imu_data (sensor_msgs/Imu)

/robot_namepace/platform/cmd_vel -> /ros2_cmd_vel_out (For debugging)

Data from ROS2 Humble topics will be mapped to ROS2 Jazzy as follows:

/cmd_vel -> /robot_namespace/cmd_vel (geometry_msgs/Twist)


**Instructions**

Update HUMBLE / JAZZY_IP constants to your IP 
Update ROBOT_NAME constant to the robot namespace ("/" can be used if there is no namespace)
Update CMD_VEL_OUT_TOPIC in jazzy_bridge to be whatever topic you're using to command in ROS2 Jazzy (will publish a TwistStamped)
Update CMD_VEL_IN_TOPIC in humble_bridge to be whatever ROS2 Humble topic you're publishing to for command (Twist)

You'll need the zmq bindings for Python: 
sudo apt install python3-zmq on Ubuntu 22.04 and 24.04

ROS2 Humble Node can be run on the 22.04 computer with python3 humble_bridge.py  
ROS2 Jazzy Node can be run on the 24.04 computer with python3 jazzy_bridge.py  
