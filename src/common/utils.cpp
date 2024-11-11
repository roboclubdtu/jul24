//
// Created by meltwin on 11/11/24.
//
#include "dtu_jul24/common/utils.hpp"
#include <sstream>

void choreographer::split_string(const std::string& input, const char delimiter, std::vector<std::string>& out) {
  out.resize(0);

  // Split the string
  const char* p = input.c_str();
  bool inside = false;
  size_t start_index = 0, current_index = 0;
  const size_t str_len = input.size();
  do {
    if (!inside && p[current_index] != delimiter) {
      inside = true;
      start_index = current_index;
    }
    else if (inside && (p[current_index] == delimiter || current_index == str_len - 1)) {
      out.push_back(input.substr(start_index, current_index - start_index + ((current_index == str_len - 1) ? 1 : 0)));
      inside = false;
    }
  }
  while (++current_index < str_len);
}

std::string choreographer::trim_string(const std::string& input) {
  const char* p = input.c_str();
  std::string out;
  const size_t str_len = input.size();
  size_t start = 0, end = str_len - 1;

  // Find start
  while (p[start] == ' ' && start < str_len)
    start++;

  // Find end
  while (p[end] == ' ' && end >= start)
    end--;

  return input.substr(start, end - start + 1);
}

std::string choreographer::add_namespace(const std::string& txt, const std::string& ns) {
  std::stringstream ss;
  ss << ns << "/" << txt;
  return ss.str();
}

std::string choreographer::add_namespace(const char* txt, const char* ns) {
  std::stringstream ss;
  ss << ns << "/" << txt;
  return ss.str();
}



