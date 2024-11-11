//
// Created by meltwin on 11/11/24.
//

#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <std_msgs/String.h>
#include "dtu_jul24/common/ServerStatus.hpp"
#include "dtu_jul24/common/types.hpp"


namespace choreographer {
    struct NameSpaces {
        constchar JOINT_STATE{"joint_state"};
    };


#define JS_TOLERANCE 0.1
}

#endif //CONSTANTS_HPP
