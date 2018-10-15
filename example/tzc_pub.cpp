#include "ros/ros.h"
#include "tzc_transport/tzc_topic.hpp"
#include "sensor_msgs/tzc_Image.hpp"

#define TOPIC_NAME "tzc_test_topic"
#define IMG_WIDTH  (1920)
#define IMG_HEIGHT (1080)
#define IMG_STEP   (3)
#define HZ         (30)
#define SHM_SIZE   (100 * 1024 * 1024)

int main(int argc, char ** argv) {
  ros::init(argc, argv, "tzc_pub", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  tzc_transport::Topic t(n);
  tzc_transport::Publisher< tzc_transport::sensor_msgs::Image > pub =
      t.advertise< tzc_transport::sensor_msgs::Image >(TOPIC_NAME, HZ, SHM_SIZE);

  ros::Rate loop_rate(HZ);
  int count = 0;
  while (ros::ok()) {
    tzc_transport::sensor_msgs::Image img;
    img.width  = IMG_WIDTH;
    img.height = IMG_HEIGHT;
    img.step   = IMG_STEP;
    img.data.resize(IMG_WIDTH * IMG_HEIGHT * IMG_STEP);

    if (pub.allocate(img)) {
      snprintf((char *)img.data.data(), IMG_WIDTH, "image # %5d ...", count);

      ROS_INFO("info: [%s]", (char *)img.data.data());
      img.header.stamp = ros::Time::now();
      pub.publish(img);
    }

    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }
  return 0;
}

