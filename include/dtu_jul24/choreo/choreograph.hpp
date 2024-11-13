//
// Created by meltwin on 07/11/24.
//

#ifndef CHOREOGRAPH_H
#define CHOREOGRAPH_H

#include <atomic>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <thread>

#include "dtu_jul24/common/ServerStatus.hpp"

namespace choreographer {
  class Choreograph {
    enum class ServerState { UNINTIALIZED, IDLE, BUSY };

    struct ServerInfo {
      ros::Subscriber sub;
      ServerStatus::ConstPtr last_msg = nullptr;
      ServerState state = ServerState::UNINTIALIZED;
    };

  public:
    static constexpr auto NODE_NAME{"choreograph"};
    Choreograph();
    ~Choreograph();

    void launch_node(int argc, char** argv);

  private:
    void clean();

    // ========================================================================
    // Discovery
    // ========================================================================
    void discover_servers();
    void register_server(const std::string&);
    static std::string get_server_prefix(const std::string&);

    // ========================================================================
    // Running
    // ========================================================================
    void terminal_cmd();
    void file_parsing(const char*);

    // ========================================================================
    // Resource server management
    // ========================================================================
    void run_line(const std::string&);
    void capture_res(const std::string& ns, const std::vector<std::string>&);
    void play_res(const std::string& ns, const std::vector<std::string>&);
    void sleep_res();
    bool check_if_res_exist(const std::string&, std::string&);

  private:
    ros::NodeHandle nh;
    std::atomic_bool STOP = false;
    std::shared_ptr<std::thread> ros_thread;

    // Time management
    std_msgs::Time time_msg;
    ros::Publisher time_publisher;
    ros::WallTimer time_timer;

    // Server discovery
    ros::WallTimer _discovery_timer;
    std::unordered_map<std::string, ServerInfo> _status_sub;
    std::unordered_map<const char*, std::string> servers;

    // Active resources users
    std::vector<std::shared_ptr<std::thread>> _active_threads;
  };
} // namespace choreographer

#endif // CHOREOGRAPH_H
