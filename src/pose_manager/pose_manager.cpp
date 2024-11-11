//
// Created by meltwin on 08/11/24.
//

#include "dtu_jul24/pose_manager/pose_manager.hpp"

namespace choreographer {

  PoseManager::PoseManager() :
    ResourceServer(true, NameSpaces::JOINT_STATE) {
    ROS_INFO("Initializing %s", PoseManager::NODE_NAME);
    // Setup ROS objects
    js_sub = nh.subscribe<JointState>(Topics::JOINT_STATE, 10,
                                      [this](auto& js) {
                                        js_callback(js);
                                      });
  }

  void PoseManager::js_callback(const JointState::ConstPtr& msg) {
    latest_js = msg;
  }

  bool PoseManager::capture_callback(
    [[maybe_unused]] Capture::Request& req,
    Capture::Response& res) {
    const unsigned int index = resources.get_current_stack_size();
    ROS_INFO("Receiving capture command #%d!", index);

    if (latest_js == nullptr) {
      ROS_ERROR("No joint state has been received yet!");
      res.capture_id = -1;
      res.collection_name = "none";
      return false;
    }

    // Save resource
    resources.capture(
      TimedResource<CJointState>{
        static_cast<double>(index),
        CJointState(latest_js)
      });

    // Return response
    res.capture_id = index;
    res.collection_name = resources.get_current_stack_name();
    return true;
  }

  JointCommand PoseManager::play_compute_trajectory(
    const CJointState& latest, const TimedResource<CJointState>& waypoint) {
    auto d_pos = (waypoint.object - latest);
    d_pos /= (waypoint.time - to_double_time(last_time));

    // Reset joint command
    joint_cmd.mode = JointCommand::VELOCITY_MODE;
    joint_cmd.names.resize(0);
    joint_cmd.command.resize(0);

    // Fill joint command
    for (auto& [name, joint] : d_pos.joints) {
      joint_cmd.names.push_back(name);
      joint_cmd.command.push_back(joint.value);
    }

    return joint_cmd;
  }


  void PoseManager::play_callback(const PlayGoal::ConstPtr& goal) {
    ROS_INFO("Playing sequence %s (%d -> %d)", goal->resource.c_str(), goal->from,
             goal->to);

    // Switch to the right resource stack
    if (!resources.select_stack(goal->resource)) {
      play_result.success = false;
      play_server.setSucceeded(play_result,
                               "Resource name is not found or loaded.");
      return;
    }
    const auto stack_size = resources.get_current_stack_size();
    const size_t end = std::min((goal->to == 0)
                                ? stack_size - 1
                                : goal->to, stack_size - 1);

    // Iterate over the resources
    ros::Rate r(play_frequency);
    for (size_t i = 0; i <= end; i++) {
      // Get waypoint to reach
      ROS_INFO("Going towards joint state %lu", i);
      auto& wp = resources.get(i);
      ROS_DEBUG("WP %lu (t=%f) = %s", i, wp.time,
                static_cast<std::string>(wp.object).c_str());

      // Send feedback
      play_feedback.current = i;
      play_server.publishFeedback(play_feedback);

      // Compute the velocity command
      auto latest = CJointState(latest_js);
      while (!latest.close_to(wp.object)) {
        play_pub.publish(play_compute_trajectory(latest, wp));
        r.sleep();
        latest = CJointState(latest_js);
      }
    }

    ROS_INFO("Sequence played!");
    play_result.success = true;
    play_server.setSucceeded(play_result, "Sequence played successfully!");
  }

}
