#include "ros/ros.h"
#include "tzc_transport/tzc_topic.hpp"
#include "sensor_msgs/tzc_Image.hpp"

#define TOPIC_NAME "tzc_test_topic"
#define HZ         (30)

void imageCallback(const tzc_transport::sensor_msgs::Image::ConstPtr & msg) {
  ROS_INFO("Image (%dx%dx%d) recieved: [%s] delay: [%5.5fms]",
      msg->width, msg->height, msg->step, (char *)msg->data.data(),
      (ros::Time::now() - msg->header.stamp).toSec() * 1000);
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "tzc_sub", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  tzc_transport::Topic t(n);
  tzc_transport::Subscriber< tzc_transport::sensor_msgs::Image > sub =
      t.subscribe< tzc_transport::sensor_msgs::Image >(TOPIC_NAME, HZ, imageCallback);
  ros::spin();
  return 0;
}

