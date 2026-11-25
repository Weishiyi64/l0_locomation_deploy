#include "legged_controllers/JointGroups.h"

namespace legged {

JointGroups classifyJointGroups(const std::vector<std::string>& joint_names) {
  JointGroups groups;
  groups.leg.assign(joint_names.size(), false);
  groups.arm.assign(joint_names.size(), false);
  groups.head.assign(joint_names.size(), false);

  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    const std::string& n = joint_names[i];
    groups.leg[i] = (n.find("hip") != std::string::npos) || (n.find("knee") != std::string::npos) ||
                    (n.find("ankle") != std::string::npos);
    groups.arm[i] = (n.find("shoulder") != std::string::npos) || (n.find("elbow") != std::string::npos) ||
                    (n.find("wrist") != std::string::npos);
    groups.head[i] = (n.find("neck") != std::string::npos) || (n.find("head") != std::string::npos);
  }

  return groups;
}

}  // namespace legged
