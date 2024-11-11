//
// Created by meltwin on 11/11/24.
//

#ifndef ACTION_TYPE_HPP
#define ACTION_TYPE_HPP

#include <string>
#include "dtu_jul24/common/types.hpp"

namespace choreographer {
  struct ActionTopics {
    constchar CAPTURE{"capture"};
    constchar PLAY{"play"};
  };

  enum class ActionType {
    UNKNOWN,
    CAPTURE,
    PLAY,
    SLEEP
  };

  inline ActionType str2action(const std::string& action) {
    if (action == "capture")
      return ActionType::CAPTURE;
    if (action == "play")
      return ActionType::PLAY;
    if (action == "sleep")
      return ActionType::SLEEP;
    return ActionType::UNKNOWN;
  }
}

#endif //ACTION_TYPE_HPP
