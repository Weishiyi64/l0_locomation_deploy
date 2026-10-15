#pragma once

#include <socketcan_wrapper/socketcan_manager.h>  // defines can::Frame

#include <functional>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "actuator_bridge/config_types.h"
#include "actuator_bridge/driver_base.h"
#include "actuator_bridge/driver_types.h"

namespace actuator_bridge {

class ActuatorBridge {
 public:
  explicit ActuatorBridge(std::shared_ptr<rclcpp::Node> node);
  ~ActuatorBridge();

  void setShutdownDisableAll(bool enabled) { shutdown_disable_all_ = enabled; }

  bool loadFromParams();

  bool enableAll();
  void disableAll();
  void resetAll();
  void zeroAll();

  void enable(size_t idx);
  void disable(size_t idx);
  void reset(size_t idx);
  void zero(size_t idx);

  // ========== 状态查询 ==========

  /**
   * @brief 获取电机总数
   * @return 电机数量
   */
  size_t motorNum() const { return flat_.size(); }

  /**
   * @brief 获取所有电机名称
   * @return 电机名称列表
   */
  std::vector<std::string> motorNames() const;

  /**
   * @brief 根据名称查找电机索引
   * @param name 电机名称
   * @return 电机索引，不存在时返回 -1
   */
  int indexOf(const std::string& name) const;

  /**
   * @brief 根据tag查找所有匹配的关节名称
   * @param tag 关节标签
   * @return 匹配的关节名称列表
   */
  std::vector<std::string> getJointNamesByTag(const std::string& tag) const;

  JointState getJointState(size_t idx) const {
    std::lock_guard<std::mutex> lock(states_mutex_);
    return states_.at(idx);
  }
  float getZeroOffset(size_t idx) const { return zero_offset_[idx]; }

  // 获取关节限位信息
  float getJointLowerLimit(size_t idx) const { return joint_lower_limit_[idx]; }
  float getJointUpperLimit(size_t idx) const { return joint_upper_limit_[idx]; }

  // ========== 状态标志 ==========

  // 发送标志（flush() 后置 true）
  bool getSendFlag(size_t idx) const {
    std::lock_guard<std::mutex> lock(sent_mutex_);
    return sent_.at(idx);
  }
  void setSendFlag(size_t idx, bool v) {
    std::lock_guard<std::mutex> lock(sent_mutex_);
    sent_.at(idx) = v;
  }

  // 接收标志（收到新反馈后置 true; 非真实电机永远返回 true）
  bool getReceiveFlag(size_t idx) const {
    if (mode_.at(idx) != ActuatorMode::ENABLE) return true;
    std::lock_guard<std::mutex> lock(receive_mutex_);
    return receive_.at(idx);
  }
  void setReceiveFlag(size_t idx, bool v) {
    std::lock_guard<std::mutex> lock(receive_mutex_);
    receive_.at(idx) = v;
  }

  // ========== 命令控制 ==========

  /**
   * @brief 设置单电机期望命令（延迟发送，需调用 flush() 才会实际发送）
   * @param idx 电机索引
   * @param cmd 关节命令
   */
  void setCommand(size_t idx, const JointCmd& cmd);

  /**
   * @brief 将所有缓存的命令编码为 CAN 帧并发送
   */
  void flush();

 private:
  // pointer to ROS2 node for logging/params
  std::shared_ptr<rclcpp::Node> node_;
  bool shutdown_disable_all_{true};

  // ========== 内部结构 ==========

  /**
   * @brief 总线结构：管理单个 CAN 总线及其上的所有执行器
   */
  struct Bus {
    std::string can_device;
    std::shared_ptr<socketcan_wrapper::SocketCANManager> can_manager;
    std::vector<ActuatorSpec> actuators;
    std::unordered_map<std::string, std::shared_ptr<DriverBase>> drivers;
  };

  /**
   * @brief 扁平索引：将电机映射到总线和 ID
   */
  struct FlatIndex {
    std::string name;
    std::string vendor;
    int id{0};
    int bus{-1};
  };

  // ========== 内部方法 ==========

  /**
   * @brief 获取或创建总线
   * @param vendor 供应商名称
   * @param can_device CAN 设备名称
   * @return 总线索引
   */
  int ensureBus(const std::string& vendor, const std::string& can_device);

  /**
   * @brief 从配置构建总线
   * @param buses_config 总线配置列表
   * @return 成功返回 true，失败返回 false
   */
  bool buildFromBusesConfig(const std::vector<BusConfig>& buses_config);

  /**
   * @brief CAN 接收回调
   * @param frame CAN 帧
   * @param bus_idx 总线索引
   */
  void CANRxCallback(const can::Frame& frame, int bus_idx);

  /**
   * @brief 检查索引是否有效
   * @param idx 索引
   * @return 有效返回 true，无效返回 false
   */
  bool isValidIndex(size_t idx) const { return idx < flat_.size(); }

  /**
   * @brief 获取扁平索引
   * @param idx 索引
   * @return 有效返回指针，无效返回 nullptr
   */
  const FlatIndex* getFlatIndex(size_t idx) const { return isValidIndex(idx) ? &flat_[idx] : nullptr; }

  /**
   * @brief 发送 CAN 帧列表
   * @param bus 总线引用
   * @param frames 帧列表
   */
  void sendFrame(Bus& bus, const can_msgs::msg::Frame& frames);

  // ========== 成员变量 ==========

  // ros::NodeHandle nh_;  // removed for ROS2
  // ros::NodeHandle pnh_; // removed for ROS2

  std::vector<Bus> buses_;
  std::unordered_map<std::string, int> bus_index_by_key_;
  std::unordered_map<std::string, int> name_to_bus_;
  std::unordered_map<std::string, int> name_to_id_;

  // 扁平索引映射与运行态缓存
  std::vector<FlatIndex> flat_;
  std::unordered_map<std::string, int> name_to_flat_;
  std::unordered_map<std::string, int> alias_to_flat_;
  std::vector<JointState> states_;
  std::vector<JointCmd> pending_cmds_;
  std::vector<bool> has_pending_cmd_;
  std::vector<bool> sent_;
  std::vector<bool> receive_;

  // 校准参数（按扁平索引对齐）
  std::vector<float> zero_offset_;
  std::vector<float> motor_direction_;
  std::vector<float> reduction_;
  std::vector<float> rotation_direction_;

  // 关节限位参数（按扁平索引对齐）
  std::vector<float> joint_lower_limit_;
  std::vector<float> joint_upper_limit_;

  // 启动时反馈判圈状态（按扁平索引对齐）
  std::vector<float> feedback_wrap_offset_;
  std::vector<bool> feedback_wrap_initialized_;

  // 关节标签（按扁平索引对齐）
  std::vector<std::string> tags_;

  // 电机模式标志（按扁平索引对齐）
  std::vector<ActuatorMode> mode_;

  // ========== 线程安全 ==========
  mutable std::mutex states_mutex_;   // 保护 states_ 数组
  mutable std::mutex sent_mutex_;     // 保护 sent_ 数组
  mutable std::mutex receive_mutex_;  // 保护 receive_ 数组
  mutable std::mutex cmd_mutex_;      // 保护 pending_cmds_ 和 has_pending_cmd_ 数组
};

}  // namespace actuator_bridge
