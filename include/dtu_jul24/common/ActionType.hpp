//
// Created by meltwin on 11/11/24.
//

#ifndef ACTION_TYPE_HPP
#define ACTION_TYPE_HPP

#include <string>
#include "dtu_jul24/common/types.hpp"

namespace choreographer {
  struct ActionTopics {
    constchar TIME{"time"};
    constchar CAPTURE{"capture"};
    constchar PLAY{"play"};
    constchar LOAD{"load"};
    constchar SAVE{"save"};
    constchar INFO{"info"};
  };

  enum class ActionType { UNKNOWN, CAPTURE, PLAY, SLEEP, LOAD, SAVE, LIST };

  inline ActionType str2action(const std::string& action) {
    if (action == "capture") return ActionType::CAPTURE;
    if (action == "play") return ActionType::PLAY;
    if (action == "sleep") return ActionType::SLEEP;
    if (action == "load") return ActionType::LOAD;
    if (action == "save") return ActionType::SAVE;
    if (action == "info") return ActionType::LIST;
    return ActionType::UNKNOWN;
  }
} // namespace choreographer

#endif // ACTION_TYPE_HPP
