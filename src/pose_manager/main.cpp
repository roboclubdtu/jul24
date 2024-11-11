//
// Created by meltwin on 08/11/24.
//

#include "dtu_jul24/pose_manager/pose_manager.hpp"

using choreographer::PoseManager;

int main(int argc, char** argv) {
  ros::init(argc, argv, PoseManager::NODE_NAME);
  auto node = PoseManager();

  while (ros::ok())
    ros::spinOnce();
  ros::shutdown();
}