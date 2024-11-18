//
// Created by meltwin on 14/11/24.
//
#include <sstream>
#include "dtu_jul24/common/BaxterJoint.hpp"

namespace choreographer {
  BaxterJoints::BaxterJoints(const JointState::ConstPtr& state) {
    for (size_t i = 0; i < state->name.size() && success; i++) {
      success &= set_joint_val(state->name[i], state->position[i]);
    }
  }

#define LIMB_FUNCTION(joint_func, name, val) joint_func(LEFT_##name, left.val) joint_func(RIGHT_##name, right.val)

#define FOR_ALL_JOINTS(joint_func)                                                                                     \
  joint_func(HEAD_NOD, head_nod);                                                                                      \
  joint_func(HEAD_PAN, head_pan);                                                                                      \
  joint_func(TORSO, torso_0);                                                                                          \
  LIMB_FUNCTION(joint_func, SHOULDER_0, shoulder_0);                                                                   \
  LIMB_FUNCTION(joint_func, SHOULDER_1, shoulder_1);                                                                   \
  LIMB_FUNCTION(joint_func, ELBOW_0, elbow_0);                                                                         \
  LIMB_FUNCTION(joint_func, ELBOW_1, elbow_0);                                                                         \
  LIMB_FUNCTION(joint_func, WRIST_0, wrist_0);                                                                         \
  LIMB_FUNCTION(joint_func, WRIST_1, wrist_1);                                                                         \
  LIMB_FUNCTION(joint_func, WRIST_2, wrist_2);

  bool BaxterJoints::set_joint_val(const std::string& name, const double val) {
#define MAKE_JOINT(txt, var)                                                                                           \
  if (name == BaxterJointNames::txt) {                                                                                 \
    var = val;                                                                                                         \
    return true;                                                                                                       \
  }
    FOR_ALL_JOINTS(MAKE_JOINT)
    return false;
  }


  bool BaxterJoints::close_to(const SharedPtr& v2, const double tolerance) const {
    // Macro to make it cleaner
#define TEST_JOINT(j, v)                                                                                               \
  if (std::fabs(v - v2->v) > tolerance) return false;
    if (!v2->success) return false;
    FOR_ALL_JOINTS(TEST_JOINT)
    return true;
  }

  std::string BaxterJoints::str() const {
    std::stringstream ss;
#define OUTPUT_JOINT(txt, field) ss << BaxterJointNames::txt << "=" << field << ",";
    ss << "BaxterJoint[";
    FOR_ALL_JOINTS(OUTPUT_JOINT)
    ss << "]";
    return ss.str();
  }

  std::vector<std::string> BaxterJoints::header_list() {
    std::vector<std::string> list(0);
#define ADD_TO_LIST(j, v) list.emplace_back(BaxterJointNames::j);
    FOR_ALL_JOINTS(ADD_TO_LIST)
    return list;
  }

  std::vector<double> BaxterJoints::values_list() const {
    std::vector<double> list(0);
#define ADD_TO_LIST(j, v) list.emplace_back(v);
    FOR_ALL_JOINTS(ADD_TO_LIST)
    return list;
  }


} // namespace choreographer
