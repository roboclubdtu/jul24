//
// Created by meltwin on 08/11/24.
//

#ifndef POSE_MANAGER_H
#define POSE_MANAGER_H


#include <baxter_core_msgs/HeadPanCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "dtu_jul24/common/BaxterJoint.hpp"
#include "dtu_jul24/common/CustomJoints.hpp"
#include "dtu_jul24/common/ResourceServer.hpp"
#include "dtu_jul24/common/types.hpp"

using baxter_core_msgs::HeadPanCommand;
using sensor_msgs::JointState;

namespace choreographer {
  class PoseManager final : public ResourceServer<JointCommand, BaxterJoints::SharedPtr> {
  public:
    constchar NODE_NAME{"pose_manager"};

    struct Topics {
      constchar JOINT_STATE{"joint_states"};
      constchar HEAD_CMD{"command_head_pan"};
    };

    PoseManager();

  private:
    void js_callback(const JointState::ConstPtr&);

    bool capture_callback(Capture::Request&, Capture::Response&) override;
    void play_callback(const PlayGoal::ConstPtr& goal) override;
    bool load_callback(LoadResource::Request&, LoadResource::Response&) override;
    bool save_callback(SaveResource::Request&, SaveResource::Response&) override;
    bool info_callback(StackInfo::Request&, StackInfo::Response&) override;

    /**
     * Trajectory point structure for easier reading of the arguments
     */
    struct TrajectoryPoint {
      typedef std::shared_ptr<TrajectoryPoint> SharedPtr;

      double time;
      BaxterJoints::SharedPtr js;

      explicit TrajectoryPoint(const double& t, const BaxterJoints::SharedPtr& _js) : time(t), js(_js) {}

      static SharedPtr make(const double& time, const BaxterJoints::SharedPtr& js) {
        return std::make_shared<TrajectoryPoint>(time, js);
      }
    };

    JointCommand play_compute_trajectory(const TrajectoryPoint::SharedPtr& previous,
                                         const TrajectoryPoint::SharedPtr& current,
                                         const TrajectoryPoint::SharedPtr& next);

    HeadPanCommand play_compute_head_cmd(const TrajectoryPoint::SharedPtr& previous,
                                         const TrajectoryPoint::SharedPtr& current,
                                         const TrajectoryPoint::SharedPtr& next);

  private:
    ros::NodeHandle nh; //!< Node handle for ROS interactions

    // JointState management
    std::shared_ptr<BaxterJoints> latest_js;
    ros::Subscriber js_sub;
    ros::Publisher head_cmd_pub;
    JointCommand joint_cmd;
    HeadPanCommand head_cmd;

    //
  };
} // namespace choreographer


#endif // POSE_MANAGER_H
