//
// Created by meltwin on 11/11/24.
//
#include <iostream>
#include <dtu_jul24/common/utils.hpp>

#include "dtu_jul24/choreo/choreograph.hpp"

namespace choreographer {

  void Choreograph::file_parsing(const char* file_name) {
    ROS_INFO("Launching choreography from file \"%s\"", file_name);
  }

  void Choreograph::terminal_cmd() {
    ROS_INFO("Entering command line interface for the choreographer");

    // Launching ROS
    ros_thread = std::make_shared<std::thread>([this]() {
      while (ros::ok() && !STOP)
        ros::spinOnce();
    });

    // Running terminal command
    std::string cmd;
    do {
      getline(std::cin, cmd);
      cmd = trim_string(cmd);
      if (cmd == "exit")
        break;
      run_line(cmd);
    }
    while (ros::ok() & !STOP);

    clean();
  }

  void Choreograph::launch_node(const int argc, char** argv) {
    // Test for the presence of a file
    if (argc >= 2) {
      for (int i = 0; i < argc; i++) {
        if (strncmp(argv[i], "-f", 2) == 0 && i != argc - 1) {
          file_parsing(argv[i + 1]);
          return;
        }
      }
    }
    terminal_cmd();
  }


}
