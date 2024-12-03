//
// Created by meltwin on 14/11/24.
//
#include <ros/ros.h>
#include <sstream>

#include "dtu_jul24/common/BaxterJoint.hpp"

namespace choreographer {
  BaxterJoints::BaxterJoints(const JointState::ConstPtr& state) : BaxterFullJoints() {
    for (size_t i = 0; i < state->name.size() && success; i++) {
      success &= set_joint_val(state->name[i], state->position[i]);
      if (!success) {
        ROS_DEBUG("Invalid joint %s", state->name[i].c_str());
      }
    }
  }

  /*
   * ==========================================================================
   * Defining utils macro
   * ==========================================================================
   */

#define LIMB_FUNCTION(joint_func, name, val) joint_func(LEFT_##name, left.val) joint_func(RIGHT_##name, right.val)

#define FOR_ALL_JOINTS(joint_func)                                                                                     \
  joint_func(HEAD_NOD, head_nod);                                                                                      \
  joint_func(HEAD_PAN, head_pan);                                                                                      \
  joint_func(TORSO, torso_0);                                                                                          \
  LIMB_FUNCTION(joint_func, SHOULDER_0, shoulder_0);                                                                   \
  LIMB_FUNCTION(joint_func, SHOULDER_1, shoulder_1);                                                                   \
  LIMB_FUNCTION(joint_func, ELBOW_0, elbow_0);                                                                         \
  LIMB_FUNCTION(joint_func, ELBOW_1, elbow_1);                                                                         \
  LIMB_FUNCTION(joint_func, WRIST_0, wrist_0);                                                                         \
  LIMB_FUNCTION(joint_func, WRIST_1, wrist_1);                                                                         \
  LIMB_FUNCTION(joint_func, WRIST_2, wrist_2);

  /*
   * ==========================================================================
   * Make Joint function & macro
   * ==========================================================================
   */
#define MAKE_JOINT(txt, var)                                                                                           \
  if (name == BaxterJointNames::txt) {                                                                                 \
    var = val;                                                                                                         \
    return true;                                                                                                       \
  }

  // Function
  bool BaxterJoints::set_joint_val(const std::string& name, const double val) {
    FOR_ALL_JOINTS(MAKE_JOINT)
    return false;
  }
#undef MAKE_JOINT


  /*
   * ==========================================================================
   * Close to function & macro
   * ==========================================================================
   */
#define TEST_JOINT(j, v)                                                                                               \
  if (std::fabs(v - v2->v) > tolerance.v) return false;

  // Function
  bool BaxterJoints::close_to(const SharedPtr& v2, const BaxterFullJoints& tolerance) const {
    if (!v2->success) return false;
    FOR_ALL_JOINTS(TEST_JOINT)
    return true;
  }
#undef TEST_JOINT

  /*
   * ==========================================================================
   * Serialization function & macro
   * ==========================================================================
   */
#define OUTPUT_JOINT(txt, field) ss << BaxterJointNames::txt << "=" << field << ",";

  // Function
  std::string BaxterJoints::str() const {
    std::stringstream ss;
    ss << "BaxterJoint[";
    FOR_ALL_JOINTS(OUTPUT_JOINT)
    ss << "]";
    return ss.str();
  }
#undef OUTPUT_JOINT

  /*
   * ==========================================================================
   * Name & Values vectors export
   * ==========================================================================
   */
#define ADD_NAMES_TO_LIST(j, v) list.emplace_back(BaxterJointNames::j);
#define ADD_VALUES_TO_LIST(j, v) list.emplace_back(v);

  // Function
  std::vector<std::string> BaxterJoints::header_list() {
    std::vector<std::string> list(0);
    FOR_ALL_JOINTS(ADD_NAMES_TO_LIST)
    return list;
  }

  // Function
  std::vector<double> BaxterJoints::values_list() const {
    std::vector<double> list(0);
    FOR_ALL_JOINTS(ADD_VALUES_TO_LIST)
    return list;
  }
#undef ADD_NAMES_TO_LIST
#undef ADD_VALUES_TO_LIST

  /*
   * ==========================================================================
   * Mathematical operators
   * ==========================================================================
   */
#define DOUBLE_FACTOR(j, v) out.v = b * v;
#define SUBSTRACT(j, v) out.v = v - b.v;
#define ADD(j, v) out.v = v + b.v;

  BaxterJoints BaxterJoints::operator*(const double& b) const {
    BaxterJoints out;
    FOR_ALL_JOINTS(DOUBLE_FACTOR)
    return out;
  }
  BaxterJoints BaxterJoints::operator-(const BaxterJoints& b) const {
    BaxterJoints out;
    FOR_ALL_JOINTS(SUBSTRACT);
    return out;
  }
  BaxterJoints BaxterJoints::operator+(const BaxterJoints& b) const {
    BaxterJoints out;
    FOR_ALL_JOINTS(ADD);
    return out;
  }
#undef DOUBLE_FACTOR
#undef SUBSTRACT
#undef ADD

  /*
   * ==========================================================================
   * Mathematical operators for shared pointer
   * ==========================================================================
   */

#define DOUBLE_FACTOR(j, v) out->v = b * a->v;
#define SUBSTRACT(j, v) out->v = a->v - b->v;
#define ADD(j, v) out->v = a->v + b->v;

  BaxterJoints::SharedPtr operator*(const double& b, const BaxterJoints::SharedPtr& a) {
    auto out = std::make_shared<BaxterJoints>();
    FOR_ALL_JOINTS(DOUBLE_FACTOR);
    return out;
  }

  BaxterJoints::SharedPtr operator-(const BaxterJoints::SharedPtr& a, const BaxterJoints::SharedPtr& b) {
    auto out = std::make_shared<BaxterJoints>();
    FOR_ALL_JOINTS(SUBSTRACT);
    return out;
  }

  BaxterJoints::SharedPtr operator+(const BaxterJoints::SharedPtr& a, const BaxterJoints::SharedPtr& b) {
    auto out = std::make_shared<BaxterJoints>();
    FOR_ALL_JOINTS(ADD);
    return out;
  }

#undef DOUBLE_FACTOR
#undef SUBSTRACT
#undef ADD


} // namespace choreographer
