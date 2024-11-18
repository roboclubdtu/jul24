//
// Created by meltwin on 14/11/24.
//
#include <filesystem>
#include <fstream>
#include <iomanip>
#include "dtu_jul24/common/utils.hpp"
#include "dtu_jul24/pose_manager/PoseLoader.hpp"


namespace choreographer {

  PoseLoader::PoseLoader(const char delimiter) : delimiter(delimiter) {}

  bool PoseLoader::load_from_file(const std::string& filename,
                                  std::vector<TimedResource<BaxterJoints::SharedPtr>>& container) const {
    // Initializing
    container.resize(0);
    std::ifstream infile(filename);
    if (!infile.is_open()) {
      ROS_ERROR("Could not open file \"%s\" to save the joint states", filename.c_str());
      return false;
    }

    // Reading header
    std::string line;
    std::getline(infile, line);
    std::vector<std::string> sliced_line;
    split_string(line, delimiter, sliced_line);
    sliced_line.erase(sliced_line.begin());
    const std::vector<std::string> names = sliced_line;
    const size_t n_joints = names.size();

    // Reading lines
    size_t line_index = 2;
    std::unordered_map<std::string, CJoint> joint_state;
    while (infile.peek() != EOF) {
      // Get line
      sliced_line.resize(0);
      std::getline(infile, line);
      split_string(line, delimiter, sliced_line);

      // Create joint
      size_t n_values = sliced_line.size() - 1;
      if (n_values != n_joints) {
        ROS_ERROR("Inconsistent number of values between header and line %lu (%lu names vs %lu values)", line_index,
                  n_joints, n_values);
        infile.close();
        return false;
      }
      auto joints = std::make_shared<BaxterJoints>();
      for (size_t i = 1; i <= n_values; i++) {
        joints->set_joint_val(names[i - 1], std::stod(sliced_line[i]));
      }
      container.push_back(TimedResource<BaxterJoints::SharedPtr>{std::stod(sliced_line[0]), joints});
      line_index++;
    }
    infile.close();
    return true;
  }

  bool PoseLoader::save_to_file(const std::string& filename,
                                const std::vector<TimedResource<BaxterJoints::SharedPtr>>& container) const {
    // Test if no joints to export
    const size_t n_to_export = container.size();
    if (n_to_export == 0) {
      ROS_ERROR("Trying to export empty joint state list!");
      return false;
    }

    // Open file
    auto p = std::filesystem::absolute(filename);
    std::ofstream outfile(p);
    if (!outfile.is_open()) {
      ROS_ERROR("Could not open file \"%s\" to save the joint states", p.c_str());
      return false;
    }

    // Save header
    outfile << "t (s)" << delimiter;
    std::vector<std::string> names = BaxterJoints::header_list();
    const size_t n_joints = names.size();
    size_t index = 0;
    for (const auto& name : names) {
      outfile << name;
      if (index++ < n_joints - 1) outfile << delimiter;
    }
    outfile << std::endl;

    // Save lines
    outfile << std::setprecision(6) << std::setfill('0') << std::fixed;
    for (size_t pt_n = 0; pt_n < n_to_export; pt_n++) {
      const auto& [time, object] = container[pt_n];
      outfile << time << delimiter;
      index = 0;
      for (const auto& joint : object->values_list()) {
        outfile << joint;
        if (index++ < n_joints - 1) outfile << delimiter;
      }
      outfile << std::endl;
    }

    outfile.close();
    return true;
  }


} // namespace choreographer
