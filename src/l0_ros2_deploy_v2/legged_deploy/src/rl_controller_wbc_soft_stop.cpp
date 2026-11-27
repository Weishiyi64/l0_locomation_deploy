#include "rl_controller_wbc.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr double kPlanWindowSeconds = .12;
constexpr double kStopWindowSeconds = 8.;
}

void RLWbc::initializeSoftStop() {
  try {
    const std::string path = std::string(LEGGED_POLICY_PATH) + "/l0_v5/stop_decoder_fp16/";
    auto model = std::make_shared<StopReferenceFp16>(path+"stop_decoder.rknn",
                                                      path+"stand_reference.json",joint_names_);
    StopReferenceFp16::Row row{};
    row[2] = .45;
    row[3] = 1.;
    const auto& stand = model->standingJoints();
    std::copy(stand.begin(),stand.end(),row.begin()+7);
    const auto warmup = model->plan({row,row,row},true);
    if (warmup.rows.empty()) throw std::runtime_error("no STOP warmup rows");
    stop_decoder_ = std::move(model);
    RCLCPP_INFO(node_->get_logger(),"Soft STOP RKNN FP16 decoder preloaded and warmed");
  } catch (const std::exception& e) {
    RCLCPP_WARN(node_->get_logger(),"Soft STOP decoder unavailable; requests will return to AMP (%s)",e.what());
  }
}

void RLWbc::resetSoftStop() {
  stop_requested_at_ = {};
  stop_plan_started_ = {};
  stop_anchor_ = -1;
  stop_tick_ = 0;
  stop_rows_.clear();
}

void RLWbc::updateSoftStop(double elapsed, const RobotFrame& state) {
  if (done_published_) return;
  const auto now = SteadyClock::now();
  const auto return_to_amp = [&](const char* reason) {
    RCLCPP_INFO(node_->get_logger(),"Soft STOP %s; requesting a fresh zero-command AMP result",reason);
    publishDone(true,reason,true);
  };

  if (stop_anchor_ < 0) {
    stop_anchor_ = std::min(current_->reference_num_frames_-1,
        current_->first_reference_frame_ + static_cast<int>(
            std::max(0.,elapsed-current_->start_hold_sec_)/current_->control_dt_));
    if (!stop_decoder_) { return_to_amp("decoder unavailable"); return; }
    try {
      if (stop_future_.valid()) {
        if (stop_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
          return_to_amp("older decoder request still running");
          return;
        }
        // A cancelled or late result belongs to an earlier execution.
        try { (void)stop_future_.get(); } catch (const std::exception&) {}
      }
      StopReferenceFp16::Entry entry{};
      for (int slot=0; slot<3; ++slot) {
        const auto frame = inferReferenceFrame(stop_anchor_+slot-1);
        std::copy(frame.root_position.begin(),frame.root_position.end(),entry[slot].begin());
        std::copy(frame.root_orientation.begin(),frame.root_orientation.end(),entry[slot].begin()+3);
        std::copy(frame.joint_position.begin(),frame.joint_position.end(),entry[slot].begin()+7);
      }
      bool stationary = -state.projected_gravity[2] > .985;
      for (const auto omega : state.angular_velocity) stationary &= std::abs(omega) < .25f;
      const auto& stand = stop_decoder_->standingJoints();
      for (int j=0; j<12; ++j)
        stationary &= std::abs(state.joint_velocity[j]) < .20f &&
            std::abs(state.joint_position[j]+current_->default_joint_positions_[j]-stand[j]) <= .12 &&
            std::abs(entry[1][7+j]-stand[j]) <= .12 &&
            std::abs(entry[2][7+j]-entry[0][7+j]) <= .0008;
      stop_plan_started_ = SteadyClock::now();
      stop_future_ = std::async(std::launch::async,[decoder=stop_decoder_,entry,stationary] {
        return decoder->plan(entry,stationary);
      });
      RCLCPP_INFO(node_->get_logger(),"Soft STOP request=%s reference=%d stationary=%d",
                  posCmd_.source_id.c_str(),stop_anchor_,stationary);
    } catch (const std::exception& e) {
      RCLCPP_WARN(node_->get_logger(),"Soft STOP could not start planning (%s)",e.what());
      return_to_amp("planning unavailable");
    }
    return;
  }

  if (std::chrono::duration<double>(now-stop_requested_at_).count() >= kStopWindowSeconds) {
    return_to_amp("8 second observation window elapsed");
    return;
  }
  if (stop_rows_.empty()) {
    const double seconds = std::chrono::duration<double>(now-stop_plan_started_).count();
    if (!stop_future_.valid() || seconds >= kPlanWindowSeconds) {
      return_to_amp("planning late or unavailable");
      return;
    }
    if (stop_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
    try {
      auto plan = stop_future_.get();
      if (plan.rows.empty()) throw std::runtime_error("empty STOP output");
      stop_rows_ = std::move(plan.rows);
      stop_tick_ = std::min<std::size_t>(1,stop_rows_.size()-1);
      RCLCPP_INFO(node_->get_logger(),
                  "Soft STOP accepted %zu frames (Decoder %d), ready %.2f ms; projection %.6f rad, original peak step %.6f rad",
                  stop_rows_.size(),plan.decoder_frames,seconds*1000.,
                  plan.max_leg_projection,plan.max_joint_step);
    } catch (const std::exception& e) {
      RCLCPP_WARN(node_->get_logger(),"Soft STOP planner failed (%s)",e.what());
      return_to_amp("planning unavailable");
    }
    return;
  }
  if (stop_tick_+1 >= stop_rows_.size())
    return_to_amp("reference reached last frame");
}
