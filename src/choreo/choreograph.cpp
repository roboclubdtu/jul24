//
// Created by meltwin on 07/11/24.
//
#include <dtu_jul24/choreo/choreograph.hpp>

using XmlRpc::XmlRpcValue;

namespace choreographer {
    Choreograph::Choreograph() {
        discover_servers();
    }

    std::string Choreograph::get_server_prefix(const std::string& topic) {
        const auto to_compare = std::string(RESOURCE_SERVER_STATUS);
        std::string found;
        // Check if enough length to compare
        if (topic.size() <= to_compare.size())
            return found;

        if (topic.compare(topic.size() - to_compare.size(), to_compare.size(),
                          to_compare) != 0)
            return found;
        found = topic.substr(0, topic.size() - to_compare.size() - 1);
        return found;
    }

    std::string make_name(const std::string& s, int index) {
        std::stringstream ss;
        ss << s << index;
        return ss.str();
    }

    void Choreograph::register_server(const std::string& s) {
        // Get last message
        if (const auto it = _status_sub.find(s); it != _status_sub.end()) {
            int index = 0;
            std::string server_name;
            auto& info = it->second;
            do {
                server_name = make_name(info.last_msg->data, index++);
            }
            while (std::any_of(servers.begin(), servers.end(),
                               [&server_name](const std::pair<const char*, std::string>& p) {
                                   return std::string(p.first) == server_name;
                               }) and index < 256);
            if (index >= 256) {
                ROS_ERROR("More than 256 servers of type %s are in the list, something is wrong",
                          info.last_msg->data.c_str());
                return;
            }
            servers.emplace(server_name.c_str(), s);
            info.initialized = true;

            ROS_INFO("Registered server %s as %s", s.c_str(), server_name.c_str());
        }
        else {
            ROS_WARN("Trying to register server `%s` without a proper status message", s.c_str());
        }
    }

    void Choreograph::discover_servers() {
        ros::master::V_TopicInfo topics;

        getTopics(topics);
        for (auto& topic : topics) {
            ROS_DEBUG("Found topic %s", topic.name.c_str());

            // Test if it is a status topic
            if (auto s = get_server_prefix(topic.name); !s.empty()) {
                // Has it been discovered already ?
                if (!std::any_of(_status_sub.begin(), _status_sub.end(),
                                 [&s](const std::pair<std::string, ServerInfo>& srv) {
                                     return srv.first.compare(s);
                                 })) {
                    ROS_INFO("Found new server %s", s.c_str());
                    servers.emplace(s.c_str(), s);

                    // Setup status sub
                    _status_sub.emplace(s, ServerInfo{nh.subscribe<ServerStatus>(
                                                          topic.name, 10,
                                                          [this,s](const ServerStatus::ConstPtr msg) {
                                                              auto& info = _status_sub[s];
                                                              info.last_msg = msg;
                                                              if (!info.initialized)
                                                                  register_server(s);

                                                          }),
                                                      nullptr});
                }
            }
        }
    }

}
