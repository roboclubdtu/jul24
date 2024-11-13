//
// Created by meltwin on 11/11/24.
//
#include <actionlib/client/simple_action_client.h>
#include <dtu_jul24/PlayAction.h>
#include <dtu_jul24/PlayFeedback.h>
#include <dtu_jul24/PlayGoal.h>
#include <dtu_jul24/PlayResult.h>


#include "dtu_jul24/Capture.h"
#include "dtu_jul24/choreo/choreograph.hpp"
#include "dtu_jul24/common/ActionType.hpp"
#include "dtu_jul24/common/utils.hpp"

using dtu_jul24::Capture;
using dtu_jul24::PlayAction;
using dtu_jul24::PlayFeedback;
using dtu_jul24::PlayGoal;
using dtu_jul24::PlayResult;

namespace choreographer {

  // ========================================================================
  // Line parsing
  // ========================================================================

  bool Choreograph::check_if_res_exist(const std::string& name, std::string& ns) {
    const auto it =
      std::find_if(servers.begin(), servers.end(), [&name](const std::pair<const char*, std::string>& pair) {
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
    if (args.empty()) {
      ROS_ERROR("Parsed line does not have to minimum required arguments! Line should start by \"<action> <res_id>\"");
      return;
    }

    const auto& action = args[0];
    std::string resource_name;
    std::string resource_ns;

#define TEST_RES_EXIST                                                                                                 \
  if (args.size() < 2) {                                                                                               \
    ROS_ERROR("No resource name given. Command should be \"%s <res_id>\"", action.c_str());                            \
    return;                                                                                                            \
  }                                                                                                                    \
  else {                                                                                                               \
    resource_name = args[1];                                                                                           \
    if (!check_if_res_exist(resource_name, resource_ns)) {                                                             \
      ROS_ERROR("Passed resource name `%s` is not registered", resource_name.c_str());                                 \
      return;                                                                                                          \
    }                                                                                                                  \
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

  void Choreograph::capture_res(const std::string& ns, const std::vector<std::string>& args) {
    ROS_INFO("Calling record service for server %s", ns.c_str());

    // Make arguments
    Capture::Request req;
    req.collection_name = Capture::Request::default_collection;
    req.play_time = -1.0;
    for (size_t i = 2; i < args.size(); i += 2) {
      if (args[i] == "-r" && i + 1 < args.size()) req.collection_name = args[i];
      if (args[i] == "-t" && i + 1 < args.size()) req.play_time = std::stof(args[i]);
    }

    // Launch threads
    _active_threads.push_back(std::make_shared<std::thread>([this, ns, req]() {
      const std::string service_name = add_namespace(ActionTopics::CAPTURE, ns);
      auto client = nh.serviceClient<Capture::Request, Capture::Response>(service_name);
      if (!client.exists()) {
        ROS_ERROR("Trying to call record service for server %s (%s) but does not exist!", ns.c_str(),
                  service_name.c_str());
        return;
      }

      // Make request
      Capture::Response res;
      if (!client.call(req, res)) {
        ROS_ERROR("Capture failed, see the PoseManager executable for more information.");
        return;
      }

      ROS_INFO("Captured %s-#%d", res.collection_name.c_str(), res.capture_id);
    }));
  }

  void Choreograph::play_res(const std::string& ns, const std::vector<std::string>& args) {
    ROS_INFO("Calling play action for server %s", ns.c_str());

    // Parse arguments
    PlayGoal goal;
    goal.resource = PlayGoal::DEFAULT_RES;
    goal.from = 0;
    goal.to = -1;
    for (size_t i = 2; i < args.size(); i += 2) {
      if (args[i] == "-f" && i + 1 < args.size()) goal.from = std::stoi(args[i]);
      if (args[i] == "-t" && i + 1 < args.size()) goal.to = std::stoi(args[i]);
      if (args[i] == "-r" && i + 1 < args.size()) goal.resource = args[i];
    }

    // Make client request
    _active_threads.push_back(std::make_shared<std::thread>([this, ns, goal]() {
      const std::string action_name = add_namespace(ActionTopics::PLAY, ns);

      auto client = actionlib::SimpleActionClient<PlayAction>(action_name, true);
      if (!client.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("Play action \"%s\" cannot be found after 5s!", action_name.c_str());
        return;
      }
      _status_sub[ns].state = ServerState::BUSY;
      client.sendGoal(
        goal, [](const actionlib::SimpleClientGoalState&, const PlayResult::ConstPtr&) {}, []() {},
        [this, &ns](const PlayFeedback::ConstPtr& feedback) {
          ROS_INFO("Played \"%s\"-#%d", ns.c_str(), feedback->current);
        });

      if (client.waitForResult()) {
        if (const auto result = client.getResult(); result->success)
          ROS_INFO("Done playing resource on server \"%s\"", ns.c_str());
        else
          ROS_ERROR("Could not play resource on server \"%s\"", ns.c_str());
      }
      _status_sub[ns].state = ServerState::IDLE;
    }));
  }

  void Choreograph::sleep_res() {
    ROS_INFO("Waiting for %lu threads to finish ...", _active_threads.size());
    for (const auto& p : _active_threads) {
      if (p->joinable()) p->join();
    }
    _active_threads.resize(0);
    ROS_INFO("Done ...");
  }


} // namespace choreographer
