//
// Created by meltwin on 09/11/24.
//

#ifndef TYPES_HPP
#define TYPES_HPP

#include <std_msgs/Time.h>

namespace choreographer {
#define constchar static constexpr const char*

  template <typename T>
  struct TimedResource {
    double time;
    T object;
  };

  inline double to_double_time(const std_msgs::Time::ConstPtr& time) {
    double t = 0;
    t += time->data.nsec * 1E-9;
    t += time->data.sec;
    return t;
  }
}

#endif //TYPES_HPP
