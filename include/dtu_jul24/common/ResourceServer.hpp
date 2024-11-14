//
// Created by meltwin on 09/11/24.
//

#ifndef RESOURCE_SERVER_HPP
#define RESOURCE_SERVER_HPP

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Time.h>

#include "dtu_jul24/Capture.h"
#include "dtu_jul24/LoadResource.h"
#include "dtu_jul24/PlayAction.h"
#include "dtu_jul24/SaveResource.h"
#include "dtu_jul24/common/ActionType.hpp"
#include "dtu_jul24/common/ResourceContainer.hpp"
#include "dtu_jul24/common/ServerStatus.hpp"
#include "dtu_jul24/common/types.hpp"
#include "dtu_jul24/common/utils.hpp"

using dtu_jul24::Capture;
using dtu_jul24::LoadResource;
using dtu_jul24::PlayAction;
using dtu_jul24::PlayFeedback;
using dtu_jul24::PlayGoal;
using dtu_jul24::PlayResult;
using dtu_jul24::SaveResource;

namespace choreographer {
  template <typename T, typename S>
  class ResourceServer {
  public:
    struct Names {
      constchar TIME_TOPIC{"time"};
      constchar OUTPUT_TOPIC{"value"};
    };

    struct Defaults {
      static constexpr int DEFAULT_FREQUENCY{5};
    };

    virtual ~ResourceServer() { play_server.shutdown(); };

  protected:
    explicit ResourceServer(const bool allow_capture, const char* node_namespace,
                            const char* output_topic = Names::OUTPUT_TOPIC) :
        node_ns(node_namespace), play_server(nh, add_namespace(ActionTopics::PLAY, node_ns),
                                             [this](const PlayGoal::ConstPtr& goal) { this->play_callback(goal); }) {
      // Record objects
      if (allow_capture) {
        record_server = nh.advertiseService<Capture::Request, Capture::Response>(
          put_ns(ActionTopics::CAPTURE),
          [this](Capture::Request& req, Capture::Response& res) { return capture_callback(req, res); });
      }

      // Play objects
      play_pub = nh.advertise<T>(output_topic, 10);
      play_server.start();
      if (std::string param_name; nh.searchParam("play_freq", param_name)) nh.getParam(param_name, play_frequency);

      // Load and save services
      load_server = nh.advertiseService<LoadResource::Request, LoadResource::Response>(
        put_ns(ActionTopics::LOAD),
        [this](LoadResource::Request& req, LoadResource::Response& res) { return load_callback(req, res); });
      save_server = nh.advertiseService<SaveResource::Request, SaveResource::Response>(
        put_ns(ActionTopics::SAVE),
        [this](SaveResource::Request& req, SaveResource::Response& res) { return save_callback(req, res); });

      // Resource status
      status_pub = nh.advertise<ServerStatus>(put_ns(RESOURCE_SERVER_STATUS), 10);
      status_timer = nh.createWallTimer(ros::WallDuration(SERVER_STATUS_PERIOD),
                                        [this](const ros::WallTimerEvent&) { status_pub.publish(status_msg); });

      // Time management object
      time_sub = nh.subscribe<std_msgs::Time>(ActionTopics::TIME, 10,
                                              [this](const std_msgs::Time::ConstPtr& time) { last_time = time; });
    }

    /**
     * Add the namespace of the node to the given text
     */
    std::string put_ns(const char* txt) const { return add_namespace(txt, node_ns); }

    /**
     * Request to capture
     */
    virtual bool capture_callback(Capture::Request&, Capture::Response&) = 0;
    virtual void play_callback(const PlayGoal::ConstPtr& goal) = 0;
    virtual bool load_callback(LoadResource::Request&, LoadResource::Response&) = 0;
    virtual bool save_callback(SaveResource::Request&, SaveResource::Response&) = 0;

  private:
    void time_callback(const std_msgs::Time::ConstPtr& time_msg) { last_time = time_msg; }

  protected:
    ros::NodeHandle nh;
    const char* node_ns;
    TimedResourceContainer<S> resources;
    ros::Subscriber time_sub;
    std_msgs::Time::ConstPtr last_time;

    // Status
    ros::Publisher status_pub;
    ServerStatus status_msg = init_status(ServerType::JOINT_STATE);
    ros::WallTimer status_timer;

    // Services
    ros::ServiceServer record_server;
    ros::ServiceServer save_server;
    ros::ServiceServer load_server;

    // Play
    actionlib::SimpleActionServer<PlayAction> play_server;
    PlayFeedback play_feedback;
    PlayResult play_result;
    ros::Publisher play_pub;
    int play_frequency = Defaults::DEFAULT_FREQUENCY;
  };
} // namespace choreographer

#endif // RESOURCE_SERVER_HPP
