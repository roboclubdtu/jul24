//
// Created by meltwin on 14/11/24.
//

#ifndef BAXTERJOINT_HPP
#define BAXTERJOINT_HPP

#include "constants.hpp"


#include <sensor_msgs/JointState.h>
#include "dtu_jul24/common/types.hpp"

using sensor_msgs::JointState;

namespace choreographer {

  struct BaxterJointNames {
    constchar HEAD_NOD{"head_node"};
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
  ;

  struct BaxterArmJoints {
    double shoulder_0 = 0, shoulder_1 = 0;
    double elbow_0 = 0, elbow_1 = 0;
    double wrist_0 = 0, wrist_1 = 0, wrist_2 = 0;
  };
  struct BaxterJoints {
    typedef std::shared_ptr<BaxterJoints> SharedPtr;

    double head_nod = 0, head_pan = 0;
    double torso_0 = 0;
    BaxterArmJoints left, right;

    bool success = true;

    explicit BaxterJoints() {}
    explicit BaxterJoints(const JointState::ConstPtr&);
    bool close_to(const SharedPtr&, double tolerance = JS_TOLERANCE) const;
    std::string str() const;
    bool set_joint_val(const std::string& name, double val);

    static std::vector<std::string> header_list();
    std::vector<double> values_list() const;
  };

} // namespace choreographer

#endif // BAXTERJOINT_HPP
