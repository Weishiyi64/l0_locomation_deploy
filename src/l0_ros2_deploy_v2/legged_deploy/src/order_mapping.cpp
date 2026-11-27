#include "order_mapping.h"

#include <unordered_map>
#include <unordered_set>

namespace order_mapping {

void OrderMapper::initialize(const std::vector<std::string>& joint_order_mj,
                             const std::vector<std::string>& joint_order_isaac) {
  if (joint_order_mj.empty() || joint_order_mj.size() > 64U || joint_order_mj.size() != joint_order_isaac.size())
  {
    RCLCPP_ERROR(logger_, "OrderMapper: joint order lists must have the same size in range 1..64");
    throw std::runtime_error("OrderMapper: invalid joint order list sizes");
  }

  const std::unordered_set<std::string> mj_names(joint_order_mj.begin(), joint_order_mj.end());
  const std::unordered_set<std::string> isaac_names(joint_order_isaac.begin(), joint_order_isaac.end());
  if (mj_names.size() != joint_order_mj.size() || isaac_names.size() != joint_order_isaac.size() ||
      mj_names != isaac_names || mj_names.count("") != 0U)
    throw std::runtime_error("OrderMapper: joint order lists must contain the same unique, non-empty names");

  mj_to_isaac_.clear();
  isaac_to_mj_.clear();

  // Build MJ to Isaac mapping
  std::unordered_map<std::string, int> isaac_map;
  for (size_t i = 0; i < joint_order_isaac.size(); ++i)
  {
    isaac_map[joint_order_isaac[i]] = static_cast<int>(i);
  }

  for (size_t i = 0; i < joint_order_mj.size(); ++i)
  {
    const std::string& name = joint_order_mj[i];
    auto it = isaac_map.find(name);
    if (it != isaac_map.end())
    {
      mj_to_isaac_.push_back(it->second);
    }
    else
    {
      mj_to_isaac_.push_back(-1);
    }
  }

  // Build Isaac to MJ mapping
  std::unordered_map<std::string, int> mj_map;
  for (size_t i = 0; i < joint_order_mj.size(); ++i)
  {
    mj_map[joint_order_mj[i]] = static_cast<int>(i);
  }

  for (size_t i = 0; i < joint_order_isaac.size(); ++i)
  {
    const std::string& name = joint_order_isaac[i];
    auto it = mj_map.find(name);
    if (it != mj_map.end())
    {
      isaac_to_mj_.push_back(it->second);
    }
    else
    {
      isaac_to_mj_.push_back(-1);
    }
  }
}

void OrderMapper::transformMjToIsaac(const Eigen::VectorXd& mj_vec, Eigen::VectorXd& isaac_vec) const {
  if (mj_vec.size() != static_cast<int>(mj_to_isaac_.size()))
  {
    RCLCPP_ERROR(logger_, "OrderMapper: MJ vector size (%ld) != expected size (%zu)", mj_vec.size(),
                 mj_to_isaac_.size());
    throw std::runtime_error("OrderMapper: MJ vector size mismatch");
  }

  isaac_vec.resize(isaac_to_mj_.size());

  for (size_t mj_i = 0; mj_i < mj_to_isaac_.size(); ++mj_i)
  {
    int isaac_index = mj_to_isaac_[mj_i];
    if (isaac_index >= 0 && isaac_index < static_cast<int>(isaac_to_mj_.size()))
    {
      isaac_vec[isaac_index] = mj_vec[mj_i];
    }
    else
    {
      RCLCPP_ERROR(logger_, "OrderMapper: MJ index %zu -> Isaac index %d is out of range", mj_i, isaac_index);
    }
  }
}

void OrderMapper::mapJointObservation(const std::vector<double>& positions, const std::vector<double>& velocities,
                                      const std::vector<double>& default_angles, Eigen::VectorXd& q,
                                      Eigen::VectorXd& dq) const {
  const size_t count = isaac_to_mj_.size();
  if (count == 0U || positions.size() != count || velocities.size() != count || default_angles.size() != count)
  {
    RCLCPP_ERROR(logger_, "OrderMapper: joint observation/default angle sizes must match the initialized mapping");
    throw std::runtime_error("OrderMapper: joint observation size mismatch");
  }

  q.resize(count);
  dq.resize(count);
  // initialize() guarantees a complete permutation; write directly into the resident state vectors.
  for (size_t isaac_i = 0; isaac_i < count; ++isaac_i)
  {
    const int mj_i = isaac_to_mj_[isaac_i];
    q[isaac_i] = positions[mj_i] - default_angles[mj_i];
    dq[isaac_i] = velocities[mj_i];
  }
}

void OrderMapper::transformIsaacToMj(const Eigen::VectorXd& isaac_vec, Eigen::VectorXd& mj_vec) const {
  if (isaac_vec.size() != static_cast<int>(isaac_to_mj_.size()))
  {
    RCLCPP_ERROR(logger_, "OrderMapper: Isaac vector size (%ld) != expected size (%zu)", isaac_vec.size(),
                 isaac_to_mj_.size());
    throw std::runtime_error("OrderMapper: Isaac vector size mismatch");
  }

  mj_vec.resize(mj_to_isaac_.size());

  for (size_t isaac_i = 0; isaac_i < isaac_to_mj_.size(); ++isaac_i)
  {
    int mj_index = isaac_to_mj_[isaac_i];
    if (mj_index >= 0 && mj_index < static_cast<int>(mj_to_isaac_.size()))
    {
      mj_vec[mj_index] = isaac_vec[isaac_i];
    }
    else
    {
      RCLCPP_ERROR(logger_, "OrderMapper: Isaac index %zu -> MJ index %d is out of range", isaac_i, mj_index);
    }
  }
}

}  // namespace order_mapping
