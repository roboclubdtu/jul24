//
// Created by meltwin on 09/11/24.
//

#ifndef RESOURCE_CONTAINER_HPP
#define RESOURCE_CONTAINER_HPP

#include <ros/ros.h>
#include <string>
#include <vector>
#include "dtu_jul24/common/types.hpp"

namespace choreographer {
  template <typename T>
  struct ResourceContainer {
    // ==========================================================================
    // Initializer
    // ==========================================================================
    ResourceContainer() {
      stacks.emplace(current, std::vector<T>(0));
      current_stack = &stacks[current];
      current_index = 0;
    }

    // ========================================================================
    // Container manipulation
    // ========================================================================

    /**
     * Get the current resource stack name
     */
    [[nodiscard]] const std::string& get_current_stack_name() const { return current; }

    /**
     * Get the current resource stack size
     */
    [[nodiscard]] unsigned int get_current_stack_size() { return stacks[current].size(); }

    size_t get_current_index() const { return current_index; }

    /**
     * Select the stack to iterate on
     */
    bool select_stack(const std::string stack_name) {
      if (auto it = stacks.find(current); it == stacks.end()) {
        ROS_ERROR("Trying to load un-existing stack %s", stack_name.c_str());
        return false;
      }
      current = stack_name;
      current_stack = &stacks[current];
      current_index = 0;
      return true;
    }

    void set_stack_cursor(const size_t pos) { current_index = pos; }

    // ========================================================================
    // Stack manipulation
    // ========================================================================
    void capture(const T& obj) {
      if (auto it = stacks.find(current); it != stacks.end()) {
        stacks.emplace(current, std::vector<T>(0));
      }

      stacks[current].push_back(obj);
    }

    T& get(const std::string& stack, const size_t index) {
      if (auto it = stacks.find(stack); it == stacks.end())
        throw std::runtime_error("Trying to access unknown stack !");
      if (stacks[stack].size() <= index)
        throw std::runtime_error("Trying to reach element in stack farther than its end");
      return stacks[stack][index];
    }

    T& get(const size_t index) {
      if (current_stack == nullptr) throw std::runtime_error("Trying to read current stack of value nullptr");
      if (current_stack->size() <= index)
        throw std::runtime_error("Trying to reach after the end of the current stack");
      return (*current_stack)[index];
    }

    T& get_next() {
      if (current_stack == nullptr) throw std::runtime_error("Trying to read current stack of value nullptr");
      if (current_stack->size() <= ++current_index) throw std::runtime_error("Reached the end of the stack");
      return (*current_stack)[current_index];
    }

    const std::vector<T>& get_stack(const std::string& stack_name) {
      if (auto it = stacks.find(stack_name); it == stacks.end()) return _empty;
      return stacks[stack_name];
    }

    void load_stack(const std::string& stack_name, std::vector<T>& res_stack) {
      if (auto it = stacks.find(stack_name); it != stacks.end()) {
        ROS_WARN("Stack %s already exist. This action is overwriting it.", stack_name.c_str());
        stacks.erase(stack_name);
      }

      stacks.emplace(stack_name, res_stack);
      select_stack(stack_name);
    }

  private:
    std::string current = "current";
    std::unordered_map<std::string, std::vector<T>> stacks;
    const std::vector<T> _empty;

    std::vector<T>* current_stack;
    size_t current_index;
  };

  template <typename T>
  using TimedResourceContainer = ResourceContainer<TimedResource<T>>;
} // namespace choreographer

#endif // RESOURCE_CONTAINER_HPP
