#pragma once

#include <string>
#include <vector>

namespace legged {

struct JointGroups {
  std::vector<bool> leg;
  std::vector<bool> arm;
  std::vector<bool> head;
};

JointGroups classifyJointGroups(const std::vector<std::string>& joint_names);

}  // namespace legged
