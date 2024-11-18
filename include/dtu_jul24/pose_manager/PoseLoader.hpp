//
// Created by meltwin on 14/11/24.
//

#ifndef POSE_LOADER_HPP
#define POSE_LOADER_HPP

#include "dtu_jul24/common/BaxterJoint.hpp"
#include "dtu_jul24/common/CustomJoints.hpp"
#include "dtu_jul24/common/ResourceContainer.hpp"
#include "dtu_jul24/common/types.hpp"

namespace choreographer {

  class PoseLoader {
  public:
    static constexpr char COMMA_SEPARATOR = ',';

    explicit PoseLoader(char delimiter = COMMA_SEPARATOR);

    bool load_from_file(const std::string& filename,
                        std::vector<TimedResource<BaxterJoints::SharedPtr>>& container) const;
    bool save_to_file(const std::string& filename,
                      const std::vector<TimedResource<BaxterJoints::SharedPtr>>& container) const;

  protected:
    const char delimiter;
  };

} // namespace choreographer

#endif // POSE_LOADER_HPP
