//
// Created by meltwin on 07/11/24.
//

#ifndef CHOREOGRAPH_H
#define CHOREOGRAPH_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include "dtu_jul24/common/constants.hpp"


namespace choreographer {
  class Choreograph {
    struct ServerInfo {
      ros::Subscriber sub;
      ServerStatus::ConstPtr last_msg = nullptr;
      bool initialized = false;
    };

  public:
    static constexpr auto NODE_NAME{"choreograph"};
    Choreograph();

  private:
    void discover_servers();
    void register_server(const std::string&);
    static std::string get_server_prefix(const std::string&);

  private:
    ros::NodeHandle nh;

    // Server discovery
    std::unordered_map<std::string, ServerInfo> _status_sub;
    std::unordered_map<const char*, std::string> servers;
  };
}

#endif //CHOREOGRAPH_H
