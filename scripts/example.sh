#!/bin/sh

ROS_PATH=/opt/ros/${ROS_DISTRO}/share

./tzc_gen_cpp.py ${ROS_PATH}/sensor_msgs/msg/Image.msg \
  -Istd_msgs:${ROS_PATH}/std_msgs/msg \
  -p sensor_msgs -o ../include/sensor_msgs -e ../templates/

./tzc_gen_cpp.py ${ROS_PATH}/sensor_msgs/msg/ChannelFloat32.msg \
  -Istd_msgs:${ROS_PATH}/std_msgs/msg \
  -p sensor_msgs -o ../include/sensor_msgs -e ../templates/

./tzc_gen_cpp.py ${ROS_PATH}/sensor_msgs/msg/PointCloud.msg \
  -Istd_msgs:${ROS_PATH}/std_msgs/msg \
  -Isensor_msgs:${ROS_PATH}/sensor_msgs/msg \
  -Igeometry_msgs:${ROS_PATH}/geometry_msgs/msg \
  -p sensor_msgs -o ../include/sensor_msgs -e ../templates/

