//
// Created by meltwin on 11/11/24.
//
#include "dtu_jul24/Capture.h"
#include "dtu_jul24/choreo/choreograph.hpp"
#include "dtu_jul24/common/ActionType.hpp"
#include "dtu_jul24/common/utils.hpp"

using dtu_jul24::Capture;

namespace choreographer {

  // ========================================================================
  // Line parsing
  // ========================================================================

  bool Choreograph::check_if_res_exist(const std::string& name, std::string& ns) {
    const auto it = std::find_if(servers.begin(), servers.end(),
                                 [&name](const std::pair<const char*, std::string>& pair) {
                                   const size_t n = std::min(strlen(pair.first), name.size());
                                   return strncmp(pair.first, name.c_str(), n);
                                 });
    if (it != servers.end()) {
      ns = std::string(it->second);
      return true;
    }
    return false;
  }

  void Choreograph::run_line(const std::string& cmd) {
    // Get cmd args
    std::vector<std::string> args;
    split_string(cmd, ' ', args);

    // Check for minimum of arguments
    if (args.size() < 2) {
      ROS_ERROR("Parsed line does not have to minimum required arguments! Line should start by \"<res_id> <action>\"");
      return;
    }

    const auto& action = args[0];
    std::string resource_name;
    std::string resource_ns;

#define TEST_RES_EXIST resource_name = args[1]; \
    if (!check_if_res_exist(resource_name, resource_ns)) { \
      ROS_ERROR("Passed resource name `%s` is not registered", resource_name.c_str()); \
      return; \
    }

    // Run action
    switch (str2action(action)) {
    case ActionType::CAPTURE:
      TEST_RES_EXIST
      capture_res(resource_ns, args);
      break;
    case ActionType::PLAY:
      TEST_RES_EXIST
      play_res(resource_ns, args);
      break;
    case ActionType::SLEEP:
      sleep_res();
      break;
    default:
      ROS_ERROR("Unknown action %s", action.c_str());
    }
  }

  // ========================================================================
  // Resource server management
  // ========================================================================

  void Choreograph::capture_res(const std::string& ns, const std::vector<std::string>&) {
    ROS_INFO("Calling record service for server %s", ns.c_str());

    // Launch threads
    _active_threads.push_back(std::make_shared<std::thread>([this, ns]() {
      const std::string service_name = add_namespace(ActionTopics::CAPTURE, ns);
      auto client = nh.serviceClient<Capture::Request, Capture::Response>(service_name);
      if (!client.exists()) {
        ROS_ERROR("Trying to call record service for server %s (%s) but does not exist!", ns.c_str(),
                  service_name.c_str());
        return;
      }

      // Make request
      Capture::Request req;
      req.collection_name = Capture::Request::default_collection;
      req.play_time = 1.0;
      Capture::Response res;
      if (!client.call(req, res)) {
        ROS_ERROR("Capture failed, see the PoseManager executable for more information.");
        return;
      }

      ROS_INFO("Captured %s-#%d", res.collection_name.c_str(), res.capture_id);
    }));
  }

  void Choreograph::play_res(const std::string& ns, const std::vector<std::string>&) {
    ROS_INFO("Calling play action for server %s", ns.c_str());
  }

  void Choreograph::sleep_res() {
    ROS_INFO("Waiting for %lu threads to finish ...", _active_threads.size());
    for (const auto& p : _active_threads) {
      if (p->joinable())
        p->join();
    }
    _active_threads.resize(0);
    ROS_INFO("Done ...");
  }


}
