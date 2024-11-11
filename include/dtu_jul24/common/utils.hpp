//
// Created by meltwin on 11/11/24.
//

#ifndef UTILS_HPP
#define UTILS_HPP

#include <string>
#include <vector>

namespace choreographer {
  void split_string(const std::string& input, char delimiter, std::vector<std::string>& out);
  std::string trim_string(const std::string& input);

  std::string add_namespace(const std::string&, const std::string&);
  std::string add_namespace(const char*, const char*);
}

#endif //UTILS_HPP
