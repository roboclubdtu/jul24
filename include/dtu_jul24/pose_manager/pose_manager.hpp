//
// Created by meltwin on 08/11/24.
//

#ifndef POSE_MANAGER_H
#define POSE_MANAGER_H


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "dtu_jul24/common/types.hpp"
#include "dtu_jul24/common/CustomJoints.hpp"
#include "dtu_jul24/common/ResourceServer.hpp"

using sensor_msgs::JointState;

namespace choreographer {
  class PoseManager final : public ResourceServer<JointState, CJointState> {
  public:
    constchar NODE_NAME{"pose_manager"};

    struct Topics {
      constchar JOINT_STATE{"joint_states"};
      constchar CAPTURE{"capture"};
    };

    PoseManager();

  private:
    void js_callback(const JointState::ConstPtr&);

    bool capture_callback(Capture::Request&,
                          Capture::Response&) override;

    void play_callback(const PlayGoal::ConstPtr& goal) override;

    JointCommand play_compute_trajectory(const CJointState& latest, const TimedResource<CJointState>& waypoint);

  private:
    ros::NodeHandle nh; //!< Node handle for ROS interactions

    // JointState management
    JointState::ConstPtr latest_js;
    ros::Subscriber js_sub;
    JointCommand joint_cmd;

    //
  };
}


#endif //POSE_MANAGER_H
