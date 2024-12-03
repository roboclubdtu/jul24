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
    head_cmd_pub = nh.advertise<HeadPanCommand>(Topics::HEAD_CMD, 10);
    head_cmd.speed_ratio = 0.3;
  }

  void PoseManager::js_callback(const JointState::ConstPtr& msg) {
    if (const auto joints = std::make_shared<BaxterJoints>(msg); joints->success) latest_js = joints;
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

  JointCommand PoseManager::play_compute_trajectory(const TrajectoryPoint::SharedPtr& previous,
                                                    const TrajectoryPoint::SharedPtr& current,
                                                    const TrajectoryPoint::SharedPtr& next) {
    // Reset joint command
    joint_cmd.mode = JointCommand::POSITION_MODE;
    joint_cmd.names.resize(0);
    joint_cmd.command.resize(0);
    joint_cmd.names = BaxterJoints::header_list();

    // If going towards start of the sequence
    if (previous == nullptr) {
      ROS_INFO_THROTTLE(0.5, "Going to start!");
      joint_cmd.command = next->js->values_list();
      return joint_cmd;
    }

    // Make command
    const double t = std::min(1.0, (current->time - previous->time) / (next->time - previous->time));
    ROS_INFO("tp = %.2f, tc = %.2f, tn = %.2f => dc = %.2f | df = %.2f | t = %.2f", previous->time, current->time,
             next->time, current->time - previous->time, next->time - previous->time, t);
    const auto target = (1 - t) * previous->js + t * next->js;
    joint_cmd.names = BaxterJoints::header_list();
    joint_cmd.command = target->values_list();
    return joint_cmd;
  }

  HeadPanCommand PoseManager::play_compute_head_cmd([[maybe_unused]] const TrajectoryPoint::SharedPtr& previous,
                                                    [[maybe_unused]] const TrajectoryPoint::SharedPtr& current,
                                                    const TrajectoryPoint::SharedPtr& next) {
    head_cmd.target = static_cast<float>(next->js->head_pan);
    return head_cmd;
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
    TrajectoryPoint::SharedPtr last_wp = nullptr;
    TrajectoryPoint::SharedPtr next_wp = nullptr;
    double start_time = 0.0;
    for (size_t i = 0; i <= end && ros::ok(); i++) {
      last_wp = next_wp;

      // If second point of the trajectory, update start time
      // (if going for point 1, we assume we are at point 0)
      if (i == 1) {
        start_time = to_double_time(last_time);
        last_wp->time = start_time;
      }

      // Get waypoint to reach
      ROS_INFO("Going towards joint state %lu", i);
      auto [time, j] = resources.get(i);
      next_wp = TrajectoryPoint::make((i == 0) ? 0.0 : start_time + time, j);
      ROS_INFO("WP %lu (t=%f) = %s", i, time, j->str().c_str());

      // Send feedback
      play_feedback.current = i;
      play_server.publishFeedback(play_feedback);

      // Compute the joints command
      auto latest_time = last_time;
      TrajectoryPoint::SharedPtr current;
      do {
        current = TrajectoryPoint::make(to_double_time(last_time), latest_js);
        play_pub.publish(play_compute_trajectory(last_wp, current, next_wp));
        head_cmd_pub.publish(play_compute_head_cmd(last_wp, current, next_wp));
        r.sleep();
      }
      while (!current->js->close_to(next_wp->js) && ros::ok());
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
      ROS_INFO("Loaded %lu joint states from file", joints_vector.size());
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

  bool PoseManager::info_callback(StackInfo::Request& req, StackInfo::Response& res) {
    res.output.resize(0);

    // Output lines
    res.output.emplace_back(req.collection_name);
    res.output.emplace_back("-------------------------");
    auto& stack = resources.get_stack(req.collection_name);
    std::stringstream ss;
    for (size_t i = 0; i < stack.size(); i++) {
      const auto& [time, object] = stack[i];
      ss << "#" << i << " - " << time << " : " << object->str();
      res.output.emplace_back(ss.str());
      ss.str("");
    }
    return true;
  }

} // namespace choreographer
