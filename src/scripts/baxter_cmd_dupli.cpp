//
// Created by meltwin on 18/11/24.
//

#include <baxter_core_msgs/JointCommand.h>
#include "dtu_jul24/common/TopicDuplicate.hpp"


int main(int argc, char** argv) {
  ros::init(argc, argv, "duplicate");
  using baxter_core_msgs::JointCommand;
  TopicDuplicate<JointCommand, JointCommand::ConstPtr> node;

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  ros::shutdown();
}
