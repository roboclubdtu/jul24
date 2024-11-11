//
// Created by meltwin on 07/11/24.
//

#include "dtu_jul24/choreo/choreograph.hpp"

using choreographer::Choreograph;

int main(int argc, char** argv) {
  ros::init(argc, argv, Choreograph::NODE_NAME);
  auto node = Choreograph();
  node.launch_node(argc, argv);
  ros::shutdown();
}
