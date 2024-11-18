//
// Created by meltwin on 18/11/24.
//
#include <ros/ros.h>
#include <std_msgs/UInt16.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "setup_baxter");
  ros::NodeHandle nh;

  // Create objects
  const ros::Publisher pub = nh.advertise<std_msgs::UInt16>("/robot/sonar/head_sonar/set_sonars_enabled", 10);
  std_msgs::UInt16 msg;
  msg.data = 0;
  ROS_INFO("Sending command for stopping the sonar");

  int loop = 0;
  ros::Rate sleep(5);
  while (loop++ < 10 && ros::ok()) {
    pub.publish(msg);
    ros::spinOnce();
    sleep.sleep();
  }
  ROS_INFO("Messages sent!");

  ros::shutdown();
}
