//
// Created by meltwin on 11/11/24.
//

#ifndef CUSTOM_JOINTS_HPP
#define CUSTOM_JOINTS_HPP

#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include "dtu_jul24/common/constants.hpp"

using sensor_msgs::JointState;
using baxter_core_msgs::JointCommand;

namespace choreographer
{
  // ==========================================================================
  // Custom fields
  // ==========================================================================
  struct CJoint
  {
    std::string name;
    double value;

    CJoint operator-(const CJoint& v2) const;
  };

  struct CJointState
  {
    std::unordered_map<std::string, CJoint> joints;

    explicit CJointState() : joints(std::unordered_map<std::string, CJoint>(0))
    {
    }

    explicit CJointState(const std::unordered_map<std::string, CJoint>& map) : joints(map)
    {
    }

    explicit CJointState(const JointState::ConstPtr&);

    bool close_to(const CJointState&, double tolerance = JS_TOLERANCE) const;

    CJointState operator-(const CJointState& v2) const;
    CJointState& operator/=(const double);
    CJointState operator/(const double) const;
    explicit operator std::string() const;
  };
}

#endif //CUSTOM_JOINTS_HPP
