//
// Created by meltwin on 14/11/24.
//

#ifndef BAXTER_JOINT_HPP
#define BAXTER_JOINT_HPP

#include "constants.hpp"

#include <sensor_msgs/JointState.h>
#include "dtu_jul24/common/types.hpp"

using sensor_msgs::JointState;

namespace choreographer {

  struct BaxterJointNames {
    constchar HEAD_NOD{"head_nod"};
    constchar HEAD_PAN{"head_pan"};
    constchar TORSO{"torso_t0"};
    constchar LEFT_SHOULDER_0{"left_s0"};
    constchar LEFT_SHOULDER_1{"left_s1"};
    constchar LEFT_ELBOW_0{"left_e0"};
    constchar LEFT_ELBOW_1{"left_e1"};
    constchar LEFT_WRIST_0{"left_w0"};
    constchar LEFT_WRIST_1{"left_w1"};
    constchar LEFT_WRIST_2{"left_w2"};
    constchar RIGHT_SHOULDER_0{"right_s0"};
    constchar RIGHT_SHOULDER_1{"right_s1"};
    constchar RIGHT_ELBOW_0{"right_e0"};
    constchar RIGHT_ELBOW_1{"right_e1"};
    constchar RIGHT_WRIST_0{"right_w0"};
    constchar RIGHT_WRIST_1{"right_w1"};
    constchar RIGHT_WRIST_2{"right_w2"};
  };

  /**
   * Container for the joints of one arm
   */
  struct BaxterArmJoints {
    double shoulder_0 = 0, shoulder_1 = 0;
    double elbow_0 = 0, elbow_1 = 0;
    double wrist_0 = 0, wrist_1 = 0, wrist_2 = 0;

    // Constructors
    explicit BaxterArmJoints() {}
    explicit BaxterArmJoints(const double value) :
        shoulder_0(value), shoulder_1(value), elbow_0(value), elbow_1(value), wrist_0(value), wrist_1(value),
        wrist_2(value) {}

    // Default tolerance for the arm
    static BaxterArmJoints getDefaultTolerance() { return BaxterArmJoints{JS_TOLERANCE}; }
  };

  /**
   * Full body joint container
   */
  struct BaxterFullJoints {
    double head_nod = 0, head_pan = 0;
    double torso_0 = 0;
    BaxterArmJoints left = BaxterArmJoints();
    BaxterArmJoints right = BaxterArmJoints();

    // Constructors
    explicit BaxterFullJoints(){};
    explicit BaxterFullJoints(const double _nod, const double _pan, const double _torso, const BaxterArmJoints& _left,
                              const BaxterArmJoints& _right) :
        head_nod(_nod), head_pan(_pan), torso_0(_torso), left(_left), right(_right) {}

    // Default tolerance for the whole robot
    static BaxterFullJoints getDefaultTolerance() {
      auto tolerance = BaxterFullJoints();
      tolerance.head_nod = JS_HEAD_TOLERANCE;
      tolerance.head_pan = JS_HEAD_TOLERANCE;
      tolerance.torso_0 = JS_TOLERANCE;
      tolerance.left = BaxterArmJoints::getDefaultTolerance();
      tolerance.right = BaxterArmJoints::getDefaultTolerance();
      return tolerance;
    }
  };

  /**
   * Wrapper around the joint container with some utils functions
   */
  struct BaxterJoints : BaxterFullJoints {
    typedef std::shared_ptr<BaxterJoints> SharedPtr;
    bool success = true;

    // Constructor
    explicit BaxterJoints() {}
    explicit BaxterJoints(const JointState::ConstPtr&);
    explicit BaxterJoints(const double _nod, const double _pan, const double _torso, const BaxterArmJoints& _left,
                          const BaxterArmJoints& _right) : BaxterFullJoints(_nod, _pan, _torso, _left, _right) {}

    // Manipulation
    bool close_to(const SharedPtr&, const BaxterFullJoints& tolerance = getDefaultTolerance()) const;
    std::string str() const;
    bool set_joint_val(const std::string& name, double val);

    // Export functions
    static std::vector<std::string> header_list();
    std::vector<double> values_list() const;

    // Math functions
    BaxterJoints operator*(const double& b) const;
    BaxterJoints operator-(const BaxterJoints& b) const;
    BaxterJoints operator+(const BaxterJoints& b) const;
  };

  // Operators on the shared_ptr
  BaxterJoints::SharedPtr operator*(const double& b, const BaxterJoints::SharedPtr& a);
  BaxterJoints::SharedPtr operator-(const BaxterJoints::SharedPtr& a, const BaxterJoints::SharedPtr& b);
  BaxterJoints::SharedPtr operator+(const BaxterJoints::SharedPtr& a, const BaxterJoints::SharedPtr& b);

} // namespace choreographer

#endif // BAXTER_JOINT_HPP
