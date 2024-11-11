//
// Created by meltwin on 11/11/24.
//

#include "dtu_jul24/common/CustomJoints.hpp"

#include <ros/ros.h>

namespace choreographer {

  CJoint CJoint::operator-(const CJoint& v2) const {
    return CJoint{name, value - v2.value};
  }


  CJointState::CJointState(const JointState::ConstPtr& js) {
    for (size_t i = 0; i < js->name.size(); i++) {
      auto& name = js->name[i];
      joints.emplace(name, CJoint{name, js->position[i]});
    }
  }

  CJointState CJointState::operator-(const CJointState& v2) const {
    CJointState state{std::unordered_map<std::string, CJoint>(0)};

    // Iterate over elements
    for (const auto& [name, joint] : joints) {
      if (auto it = v2.joints.find(name); it != v2.joints.end())
        state.joints.emplace(name, joint - it->second);
    }

    return state;
  }

  CJointState& CJointState::operator/=(const double divider) {
    for (auto& [name, val] : joints) {
      val.value = val.value / divider;
    }
    return *this;
  }

  CJointState CJointState::operator/(const double divider) const {
    CJointState newer{joints};
    newer /= divider;
    return newer;
  }

  bool CJointState::close_to(const CJointState& v2, double tolerance) const {
    auto diff = *this - v2;
    return std::all_of(diff.joints.begin(), diff.joints.end(),
                       [tolerance](const std::pair<std::string, CJoint>& pair) {
                         return std::fabs(pair.second.value) < tolerance;
                       });
  }

  CJointState::operator std::string() const {
    std::stringstream ss;
    ss << "[";
    for (auto& [name, joint] : joints)
      ss << name << "= " << joint.value << ", ";
    ss << "]";
    return ss.str();
  }


}
