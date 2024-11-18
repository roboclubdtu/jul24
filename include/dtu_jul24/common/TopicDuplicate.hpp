//
// Created by meltwin on 18/11/24.
//

#ifndef TOPIC_DUPLICATE_HPP
#define TOPIC_DUPLICATE_HPP

#include <ros/ros.h>

template <typename T, typename S>
class TopicDuplicate {
public:
  TopicDuplicate() {
    sub = nh.subscribe<T>("input", 10, [this](const S& msg) { callback(msg); });
    pub1 = nh.advertise<T>("output1", 10);
    pub2 = nh.advertise<T>("output2", 10);
  }

private:
  void callback(const S& msg) {
    pub1.publish(msg);
    pub2.publish(msg);
  }

  ros::Subscriber sub;
  ros::Publisher pub1, pub2;
  ros::NodeHandle nh;
};

#endif // TOPIC_DUPLICATE_HPP
