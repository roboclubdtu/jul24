//
// Created by meltwin on 07/11/24.
//

#include "dtu_jul24/choreo/choreograph.hpp"

using choreographer::Choreograph;

int main(int argc, char** argv) {
  ros::init(argc, argv, Choreograph::NODE_NAME);
  auto node = Choreograph();

  while (ros::ok())
    ros::spinOnce();

  ros::shutdown();
}