#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <luvbot_control_interfaces/action/follow_navigation_path.hpp>

namespace legged_controllers {

// Navigation supplies the remaining polyline; sample a forward 0.4 m window for the model.
class FollowGoalTracker {
 public:
  using Goal = luvbot_control_interfaces::action::FollowNavigationPath::Goal;
  static constexpr std::size_t kPlannerInputSize = 21;

  struct Output {
    std::array<float, kPlannerInputSize> data{};
    const char* mode{"Hold"};
    double remaining_distance{0.0};
    double heading_error{0.0};
    bool reached{false};
  };

  struct Pose2D {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  // Generation stamps are provenance, not execution content. A newer stamp may revalidate a revision.
  static bool sameReference(const Goal& a, const Goal& b) {
    if (a.task_id != b.task_id || a.path_revision != b.path_revision || a.use_heading != b.use_heading ||
        (a.use_heading && a.goal_yaw != b.goal_yaw) || a.speed != b.speed || a.turn_speed != b.turn_speed ||
        a.reference_timeout != b.reference_timeout || a.path.header.frame_id != b.path.header.frame_id)
      return false;
    return a.path.points == b.path.points;
  }

  void setReference(std::shared_ptr<const Goal> goal, bool new_task) {
    if (new_task || !goal_ || goal->path_revision != goal_->path_revision) {
      segment_ = 0;
      station_ = .4;
    }
    bool reset_arrival = new_task || !goal_;
    if (goal_) {
      const auto& old_end = goal_->path.points.back();
      const auto& new_end = goal->path.points.back();
      reset_arrival = reset_arrival || old_end.x != new_end.x || old_end.y != new_end.y ||
          goal_->use_heading != goal->use_heading || (goal->use_heading && goal_->goal_yaw != goal->goal_yaw);
    }
    if (reset_arrival) position_locked_ = false;
    goal_ = std::move(goal);
    arc_.assign(goal_->path.points.size(), 0.0);
    for (std::size_t i = 1; i < goal_->path.points.size(); ++i) {
      const auto& a = goal_->path.points[i-1];
      const auto& b = goal_->path.points[i];
      arc_[i] = arc_[i-1]+std::hypot(b.x-a.x, b.y-a.y);
    }
  }

  Output update(const Pose2D& pose) {
    Output output;
    if (!goal_) return output;
    const auto& points = goal_->path.points;
    const auto& first = points.front();
    const auto& end = points.back();
    const double dx = end.x-pose.x, dy = end.y-pose.y;
    const double distance = std::hypot(dx, dy);
    output.remaining_distance = std::hypot(first.x-pose.x, first.y-pose.y)+arc_.back();
    output.heading_error = goal_->use_heading ? wrap(goal_->goal_yaw-pose.yaw) : 0.0;
    if (!std::isfinite(distance) || !std::isfinite(output.remaining_distance) ||
        !std::isfinite(output.heading_error)) return output;
    // Arrival and slowdown use endpoint distance; remaining_distance stays path-based feedback.
    if (position_locked_ && distance > .5) position_locked_ = false;
    if (!position_locked_ && distance <= .25) position_locked_ = true;
    if (!position_locked_) {
      const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
      output.data[0] = static_cast<float>(c*dx+s*dy);
      output.data[1] = static_cast<float>(-s*dx+c*dy);
      const double speed = goal_->speed == 0.0 ? .35 : clamp(goal_->speed, .3, .8);
      const double slowdown = clamp((1.5-distance)/1.3, 0.0, 1.0);
      output.data[2] = static_cast<float>(speed-(speed-.3)*std::expm1(3.0*slowdown)/std::expm1(3.0));
      samplePath(pose, output.data);
      output.mode = "Nav";
    } else if (goal_->use_heading && std::abs(output.heading_error) > kHeadingTolerance) {
      output.data[3] = static_cast<float>(output.heading_error);
      output.data[4] = static_cast<float>(goal_->turn_speed == 0.0 ? 1.57 : clamp(std::abs(goal_->turn_speed), 0.0, 1.57));
      output.mode = "Face";
    } else {
      // Enter and exit use the same 10 degree threshold; terminal latching belongs to the Action server.
      output.reached = true;
    }
    return output;
  }

  void reset() {
    goal_.reset();
    position_locked_ = false;
    arc_.clear();
    segment_ = 0;
    station_ = .4;
  }

 private:
  void samplePath(const Pose2D& pose, std::array<float, kPlannerInputSize>& data) {
    const auto& points = goal_->path.points;
    if (points.size() > 1) {
      const double anchor = arc_[std::min(segment_, points.size()-2)];
      double nearest = std::numeric_limits<double>::infinity(), progress = 0.0;
      for (std::size_t i = 0; i+1 < points.size(); ++i) {
        if (arc_[i] < std::max(0.0, anchor-1.6) || arc_[i] > anchor+5.2) continue;
        const double length = arc_[i+1]-arc_[i];
        if (length <= 1e-6) continue;
        const double x = points[i+1].x-points[i].x, y = points[i+1].y-points[i].y;
        const double fraction = clamp(((pose.x-points[i].x)*x+(pose.y-points[i].y)*y)/(x*x+y*y), 0.0, 1.0);
        const double dx = points[i].x+fraction*x-pose.x, dy = points[i].y+fraction*y-pose.y;
        if (dx*dx+dy*dy < nearest) {
          nearest = dx*dx+dy*dy;
          progress = arc_[i]+fraction*length;
          segment_ = i;
        }
      }
      if (std::isfinite(nearest)) station_ = std::max(station_, (std::floor(progress/.4)+1.0)*.4);
    }
    const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
    for (std::size_t i = 0; i < 8; ++i) {
      // Single-point and zero-length references remain valid direct goals.
      double x = points.back().x, y = points.back().y;
      const double sample = std::min(station_+.4*i, arc_.back());
      if (points.size() > 1 && sample < arc_.back()) {
        const auto upper = std::upper_bound(arc_.begin(), arc_.end(), sample);
        const auto index = static_cast<std::size_t>(upper-arc_.begin()-1);
        const double fraction = (sample-arc_[index])/(arc_[index+1]-arc_[index]);
        x = points[index].x+fraction*(points[index+1].x-points[index].x);
        y = points[index].y+fraction*(points[index+1].y-points[index].y);
      }
      x -= pose.x; y -= pose.y;
      data[5+2*i] = static_cast<float>(c*x+s*y);
      data[6+2*i] = static_cast<float>(-s*x+c*y);
    }
  }

  static constexpr double kHeadingTolerance = 3.14159265358979323846/18.0;
  static double clamp(double value, double lower, double upper) { return std::max(lower, std::min(value, upper)); }
  static double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
  std::shared_ptr<const Goal> goal_;
  bool position_locked_{false};
  std::vector<double> arc_;
  std::size_t segment_{0};
  double station_{.4};
};

}  // namespace legged_controllers
