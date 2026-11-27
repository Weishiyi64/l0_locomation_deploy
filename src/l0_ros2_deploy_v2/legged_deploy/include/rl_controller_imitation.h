#ifndef RL_SDK_IMITATION_H
#define RL_SDK_IMITATION_H

#include "inference_model.h"

#include <Eigen/Dense>
#include <atomic>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <thread>
#include <vector>

#include "base/base_rl_controller.h"
#include "base/rl_controller_registrar.h"
#include "order_mapping.h"
#include "robot_interfaces/srv/switch_policy.hpp"
#include "rl_data.h"
#include "rl_data_imitation.h"

// Per-output buffer for the imitation (motion) model.
// Named output index and validated CPU buffer.
struct ImitationOutputBuffer {
  int output_index = -1;
  std::vector<float> host_buf;
};

class RL_imitation : public BaseRLController {
 public:
  RL_imitation(rclcpp::Node::SharedPtr nh);
  ~RL_imitation();

  void initTopic(rclcpp::Node::SharedPtr nh, bool create_switch_service = true);
  void initModel();
  bool getParam();
  void initData();

  bool updateObservation();
  bool updateAction(Robot_Imitation_Output& output);
  void updateImitationData();

  void update();
  void stateCallback(const robot_interfaces::msg::PolicyObservation::ConstSharedPtr msg) override;
  void imitationResetCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void imitationActiveCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);

  RL_IMITATION_ModelParams params;
  LeggedDeployParams deploy_params;

 protected:
  std::string rlCmdPublishTopic() const override;

  std::atomic_bool loopRunning_{};
  std::thread loopThread_;

 private:
  void set_anchor_ori(const Eigen::Quaterniond& robot_initial_quat, const Eigen::Quaterniond& q_robot,
                      const Eigen::Quaterniond& imitation_initial_quat, const Eigen::Quaterniond& q_imitation);

  Eigen::Quaterniond get_imitation_anchor_ori_b_cpp(const Eigen::Quaterniond& robot_initial_quat,
                                                    const Eigen::Quaterniond& q_robot,
                                                    const Eigen::Quaterniond& imitation_initial_quat,
                                                    const Eigen::Quaterniond& q_imitation);

  struct MotionData {
    Eigen::Quaterniond body_quat;
    Eigen::VectorXd joint_pos;
    Eigen::VectorXd joint_vel;
    Eigen::VectorXd contact_mask;
    Eigen::VectorXd root_pos;
    Eigen::VectorXd root_quat;
    Eigen::VectorXd joint_pos_policy;
  };

  struct CachedPolicyRuntime {
    CachedPolicyRuntime(rclcpp::Logger logger, rcl_clock_type_t clock_type)
        : rbt_state(logger),
          rbt_output(logger),
          obs(Observations_Imitation(1, 1)),
          imitation_start_time(0, 0, clock_type) {}

    std::string policy_name;
    std::string robot_cfg_path;
    RL_IMITATION_ModelParams params;
    bool pub_imitationref_joint_states = false;
    Robot_Imitation_State rbt_state;
    Robot_Imitation_Output rbt_output;
    Observations_Imitation obs;
    std::unique_ptr<ObservationManager_Imitation> obs_manager;
    rclcpp::Time imitation_start_time;
    bool previous_rl_inference_enabled = false;
    bool initial_quats_logged = false;
    Eigen::Quaterniond robot_initial_quat;
    Eigen::Quaterniond imitation_initial_quat;
    int time_step_counter = 0;
    std::string policy_output_name;
    bool motion_done_sent = false;
    std::vector<MotionData> precomputed_motion_data;
    int motion_data_size = 0;
    bool has_body_quat_output = false;

    std::unique_ptr<InferenceModel> model_policy;
    std::unique_ptr<InferenceModel> model_imitation;
    int model_policy_output_idx = 0;
    std::map<std::string, ImitationOutputBuffer> imitation_outputs;
  };

  // Auto-detect motion duration by checking if 5 consecutive frames are
  // identical
  int detectMaxTimeSteps(const std::map<std::string, size_t>& output_dims);

  void createSwitchPolicyService();
  void resetImitationState();
  void resetObservationState();
  void releaseModelResourcesLocked();
  void releaseCachedRuntime(CachedPolicyRuntime& runtime);
  CachedPolicyRuntime takeCurrentRuntimeLocked();
  void storeCurrentRuntimeLocked();
  void restoreRuntimeLocked(CachedPolicyRuntime&& runtime);
  bool loadPolicyIntoCurrentLocked(const std::string& policy_name, std::string& robot_cfg_path, std::string& message);
  bool loadPolicyLocked(const std::string& policy_name, std::string& robot_cfg_path, std::string& message);
  void preloadPoliciesLocked(const std::vector<std::string>& policies);
  void refreshInferenceEnabledLocked();
  void switchImitationPolicyCallback(
      const std::shared_ptr<robot_interfaces::srv::SwitchPolicy::Request> request,
      std::shared_ptr<robot_interfaces::srv::SwitchPolicy::Response> response);

  // Run one imitation model inference step.
  // Runs the motion model and validates its outputs before use.
  bool runImitationInference(float timestep);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subResetCmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subImitationActive_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pubImitationJointStates_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubImitationMotionDone_;
  rclcpp::Service<robot_interfaces::srv::SwitchPolicy>::SharedPtr switchPolicySrv_;

  bool motion_done_sent_ = false;

  bool pub_imitationref_joint_states_ = false;

  Robot_Imitation_State rbt_state;
  Robot_Imitation_Output rbt_output;
  Observations_Imitation obs;

  std::unique_ptr<ObservationManager_Imitation> obs_manager_;

  rclcpp::Time imitation_start_time;
  bool previous_rl_inference_enabled_ = false;
  bool initial_quats_logged_ = false;

  Eigen::Quaterniond robot_initial_quat, imitation_initial_quat;

  // Order mapper instance
  order_mapping::OrderMapper order_mapper_;

  // time step counter and motion flag
  int time_step_counter_ = 0;

  // optional override for the policy model output name (lowercased)
  std::string policy_output_name_;
  std::string policy_root_path_;
  std::string robot_type_;
  std::string current_policy_name_;
  std::string current_robot_cfg_path_;
  bool dynamic_policy_switch_enabled_ = false;
  bool imitation_active_ = false;
  std::atomic_bool model_loaded_{false};
  std::atomic<int> loop_frequency_hz_{100};
  mutable std::mutex model_mutex_;
  std::vector<std::string> available_imitation_policies_;
  std::vector<std::string> preload_imitation_policies_;
  std::map<std::string, CachedPolicyRuntime> cached_policies_;

  std::vector<MotionData> precomputed_motion_data_;
  int motion_data_size_ = 0;
  bool has_body_quat_output_ = false;

  std::unique_ptr<InferenceModel> model_policy_;
  std::unique_ptr<InferenceModel> model_imitation_;
  int model_policy_output_idx_ = 0;

  // Named model outputs on the host.
  std::map<std::string, ImitationOutputBuffer> imitation_outputs_;

  rclcpp::Node::SharedPtr nh_;
};

#endif
