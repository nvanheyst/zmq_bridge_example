# zmq_bridge_example

**Description**

This is a POC example for a simple bridge between ROS Noetic and ROS2 Jazzy. This example template is setup for a ROS1 system that could publish and subscribe to topics from a ROS2 platform. In this setup the ROS Noetic system is controlling the ROS2 Jazzy system, but the template could be adjusted to work in reverse. This was tested with teleoperation over an ethernet connection. This could work with an autonomy stack and additional topics can be added as well. This hasn't been tested with images or point clouds just the topics shown below. 


**Example Template**

Data from ROS2 topics will be mapped to ROS1 as follows:
/robot_namespace/platform/odom -> /ros2_odom_out (nav_msgs/Odometry)

/robot_namespace/platform/odom/filtered -> /ros2_odom_filtered_out (nav_msgs/Odometry)

/robot_namespace/sensors/imu_0/data -> /ros2_imu_data (sensor_msgs/Imu)

/robot_namepace/platform/cmd_vel -> /ros2_cmd_vel_out (For debugging)

Data from ROS1 topics will be mapped to ROS2 as follows:

/cmd_vel -> /robot_namespace/cmd_vel (geometry_msgs/Twist)


**Instructions**

Update ROS1 / ROS2_IP constants to your IP 
Update ROBOT_NAME constant to the robot namespace ("/" can be used if there is no namespace)
Update CMD_VEL_OUT_TOPIC in ros2_bridge to be whatever topic you're using to command in ros2 (will publish a TwistStamped)
Update CMD_VEL_IN_TOPIC in ros1_bridge to be whatever ROS1 topic you're publishing to for command (Twist)

You'll need the zmq bindings for Python: 
sudo apt install python3-zmq on Ubuntu 24.04
pip3 install pyzmq on Ubuntu 20.04.

ROS1 Node can be run on the 20.04/Noetic computer with python3 ros1_bridge.py  
ROS2 Node can be run on the 24.04/Jazzy computer with python3 ros2_bridge.py  
