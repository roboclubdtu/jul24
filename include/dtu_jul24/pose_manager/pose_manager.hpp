//
// Created by meltwin on 08/11/24.
//

#ifndef POSE_MANAGER_H
#define POSE_MANAGER_H


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "dtu_jul24/common/BaxterJoint.hpp"
#include "dtu_jul24/common/CustomJoints.hpp"
#include "dtu_jul24/common/ResourceServer.hpp"
#include "dtu_jul24/common/types.hpp"

using sensor_msgs::JointState;

namespace choreographer {
  class PoseManager final : public ResourceServer<JointCommand, BaxterJoints::SharedPtr> {
  public:
    constchar NODE_NAME{"pose_manager"};

    struct Topics {
      constchar JOINT_STATE{"joint_states"};
      constchar CAPTURE{"capture"};
    };

    PoseManager();

  private:
    void js_callback(const JointState::ConstPtr&);

    bool capture_callback(Capture::Request&, Capture::Response&) override;
    void play_callback(const PlayGoal::ConstPtr& goal) override;
    bool load_callback(LoadResource::Request&, LoadResource::Response&) override;
    bool save_callback(SaveResource::Request&, SaveResource::Response&) override;

    JointCommand play_compute_trajectory(const BaxterJoints::SharedPtr& latest,
                                         const TimedResource<BaxterJoints::SharedPtr>& waypoint,
                                         const std_msgs::Time::ConstPtr& start_time);

  private:
    ros::NodeHandle nh; //!< Node handle for ROS interactions

    // JointState management
    std::shared_ptr<BaxterJoints> latest_js;
    ros::Subscriber js_sub;
    JointCommand joint_cmd;

    //
  };
} // namespace choreographer


#endif // POSE_MANAGER_H
