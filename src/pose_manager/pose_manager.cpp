//
// Created by meltwin on 08/11/24.
//

#include "dtu_jul24/pose_manager/pose_manager.hpp"
#include "dtu_jul24/pose_manager/PoseLoader.hpp"

namespace choreographer {

  PoseManager::PoseManager() : ResourceServer(true, NameSpaces::JOINT_STATE) {
    ROS_INFO("Initializing %s", PoseManager::NODE_NAME);
    // Setup ROS objects
    js_sub = nh.subscribe<JointState>(Topics::JOINT_STATE, 10, [this](auto& js) { js_callback(js); });
  }

  void PoseManager::js_callback(const JointState::ConstPtr& msg) {
    auto joints = std::make_shared<BaxterJoints>(msg);
    if (joints->success) latest_js = joints;
  }

  bool PoseManager::capture_callback([[maybe_unused]] Capture::Request& req, Capture::Response& res) {
    // Select stack
    if (!resources.select_stack(req.collection_name)) {
      ROS_ERROR("Capture request collection name is not loaded");
      res.capture_id = -1;
      res.collection_name = "none";
      return false;
    }
    const unsigned int index = resources.get_current_stack_size();
    const double time = (req.play_time == -1.0) ? static_cast<double>(index) : req.play_time;
    ROS_INFO("Receiving capture command %s-#%d (t=%f)!", req.collection_name.c_str(), index, time);

    if (latest_js == nullptr) {
      ROS_ERROR("No joint state has been received yet!");
      res.capture_id = -1;
      res.collection_name = "none";
      return false;
    }

    // Save resource
    resources.capture(TimedResource<BaxterJoints::SharedPtr>{time, latest_js});
    ROS_DEBUG("Joint = %s", resources.get(0).object->str().c_str());

    // Return response
    res.capture_id = index;
    res.collection_name = resources.get_current_stack_name();
    return true;
  }

  JointCommand PoseManager::play_compute_trajectory(const BaxterJoints::SharedPtr& latest,
                                                    const TimedResource<BaxterJoints::SharedPtr>& waypoint,
                                                    const std_msgs::Time::ConstPtr& start_time) {
    // Reset joint command
    joint_cmd.mode = JointCommand::POSITION_MODE;
    joint_cmd.names.resize(0);
    joint_cmd.command.resize(0);

    // Fill joint command
    // for (auto& [name, joint] : waypoint.object->head_nod) {
    //   joint_cmd.names.push_back(name);
    //   joint_cmd.command.push_back(joint.value);
    // }

    return joint_cmd;
  }


  void PoseManager::play_callback(const PlayGoal::ConstPtr& goal) {
    ROS_INFO("Playing sequence `%s` (%d -> %d)", goal->resource.c_str(), goal->from, goal->to);

    // Switch to the right resource stack
    if (!resources.select_stack(goal->resource)) {
      ROS_ERROR("Resource name \"%s\" is not found or loaded.", goal->resource.c_str());
      play_result.success = false;
      play_server.setAborted(play_result, "Resource name is not found or loaded.");
      return;
    }
    const auto stack_size = resources.get_current_stack_size();
    if (stack_size == 0) {
      ROS_ERROR("Trying to play on empty collection \"%s\"", goal->resource.c_str());
      play_result.success = false;
      play_server.setAborted(play_result, "Trying to play on empty collection");
      return;
    }
    const size_t end = std::min((goal->to == -1) ? stack_size - 1 : goal->to, stack_size - 1);
    ROS_INFO("Playing joint state in the range [%d, %zu]", goal->from, end);

    // Iterate over the resources
    ros::Rate r(play_frequency);
    auto start_time = last_time;
    for (size_t i = 0; i <= end && ros::ok(); i++) {
      // Get waypoint to reach
      ROS_INFO("Going towards joint state %lu", i);
      auto wp = resources.get(i);
      ROS_INFO("WP %lu (t=%f) = %s", i, wp.time, wp.object->str().c_str());

      // Send feedback
      play_feedback.current = i;
      play_server.publishFeedback(play_feedback);

      // Compute the velocity command
      auto latest = latest_js;
      do {
        play_pub.publish(play_compute_trajectory(latest, wp, start_time));
        r.sleep();
        latest = latest_js;
      }
      while (!latest->close_to(wp.object) && ros::ok());
    }

    ROS_INFO("Sequence played!");
    play_result.success = true;
    play_server.setSucceeded(play_result, "Sequence played successfully!");
  }

  bool PoseManager::load_callback(LoadResource::Request& req, LoadResource::Response& res) {
    ROS_INFO("Loading joint trajectory from file \"%s\" into collection \"%s\"", req.file.c_str(),
             req.collection_name.c_str());
    std::vector<TimedResource<BaxterJoints::SharedPtr>> joints_vector(0);
    if (const PoseLoader pose_io; !pose_io.load_from_file(req.file, joints_vector)) {
      ROS_ERROR("Could not load joint trajectory from file!");
      res.success = LoadResource::Response::FAILED;
      return false;
    }
    else {
      resources.load_stack(req.collection_name, joints_vector);
      res.success = LoadResource::Response::SUCCEEDED;
      return true;
    }
  }

  bool PoseManager::save_callback(SaveResource::Request& req, SaveResource::Response& res) {
    ROS_INFO("Saving joint trajectory from collection \"%s\" into file \"%s\"", req.collection_name.c_str(),
             req.file.c_str());
    if (const PoseLoader pose_io; !pose_io.save_to_file(req.file, resources.get_stack(req.collection_name))) {
      res.success = SaveResource::Response::FAILED;
      return false;
    }
    else {
      ROS_INFO("Successfully saved trajectory !");
      res.success = SaveResource::Response::SUCCEEDED;
      return true;
    }
  }

} // namespace choreographer
