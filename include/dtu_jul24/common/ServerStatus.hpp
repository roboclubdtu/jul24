//
// Created by meltwin on 11/11/24.
//

#ifndef SERVER_STATUS_HPP
#define SERVER_STATUS_HPP

#include <std_msgs/String.h>
#include "dtu_jul24/common/types.hpp"

namespace choreographer {
#define RESOURCE_SERVER_STATUS "ressrv_status"
#define SERVER_STATUS_PERIOD 0.5
    using ServerStatus = std_msgs::String;

    struct ServerType_V {
        constchar JOINT_STATE{"js"};
    };

    enum class ServerType {
        JOINT_STATE
    };

    inline ServerStatus init_status(const ServerType t) {
        ServerStatus status;
        switch (t) {
        case ServerType::JOINT_STATE:
            status.data = ServerType_V::JOINT_STATE;
        }
        return status;
    }
}


#endif //SERVER_STATUS_HPP
