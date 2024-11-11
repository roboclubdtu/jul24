//
// Created by meltwin on 11/11/24.
//
#include "dtu_jul24/choreo/choreograph.hpp"

namespace choreographer {

  Choreograph::Choreograph() {
    _discovery_timer = nh.createWallTimer(ros::WallDuration(0.2), [this](const ros::WallTimerEvent&) {
      discover_servers();
    });
  }

  Choreograph::~Choreograph() {
    clean();
  }

  void Choreograph::clean() {
    // Launch stop signals
    STOP = true;
    ros::shutdown();

    // Wait for the ROS thread
    if (ros_thread != nullptr && ros_thread->joinable())
      ros_thread->join();

    // Wait for child processes
    for (const auto& p : _active_threads) {
      if (p->joinable())
        p->join();
    }
    _active_threads.resize(0);
  }


}
