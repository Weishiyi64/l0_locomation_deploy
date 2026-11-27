#include "stop_reference_fp16.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace {
constexpr int kDecoderFrames = 228;
constexpr std::size_t kMaximumFrames = 400;
constexpr double kStep = .14;
constexpr double kBounds[12][2] = {
  {-2.618,1.047},{-.349,1.745},{-1.047,1.047},{0,2.35619449},{-.6981317,.87266463},{-.436,.436},
  {-2.618,1.047},{-1.745,.349},{-1.047,1.047},{0,2.35619449},{-.6981317,.87266463},{-.436,.436}
};

Eigen::Quaterniond quaternion(const StopReferenceFp16::Row& row) {
  const Eigen::Quaterniond q(row[3], row[4], row[5], row[6]);
  if (!q.coeffs().allFinite() || q.norm() < 1e-9)
    throw std::runtime_error("STOP invalid root quaternion");
  return q.normalized();
}

Eigen::Matrix3d inverseHeading(const Eigen::Matrix3d& rotation) {
  return Eigen::AngleAxisd(-std::atan2(rotation(1,0),rotation(0,0)),
                           Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Matrix3d rotation6(const float* row) {
  Eigen::Vector3d a(row[3], row[4], row[5]);
  Eigen::Vector3d b(row[6], row[7], row[8]);
  if (a.norm() < 1e-6) throw std::runtime_error("STOP invalid first rotation axis");
  a.normalize();
  b -= a.dot(b)*a;
  if (b.norm() < 1e-6) throw std::runtime_error("STOP invalid second rotation axis");
  b.normalize();
  Eigen::Matrix3d rotation;
  rotation.row(0) = a;
  rotation.row(1) = b;
  rotation.row(2) = a.cross(b);
  return rotation;
}

void copyQuaternion(StopReferenceFp16::Row& row, const Eigen::Quaterniond& q) {
  row[3] = q.w();
  row[4] = q.x();
  row[5] = q.y();
  row[6] = q.z();
}
}  // namespace

StopReferenceFp16::StopReferenceFp16(const std::string& model, const std::string& profile,
                                   const std::vector<std::string>& joint_names)
    : decoder_(model) {
  if (decoder_.inputCount() != 3 || decoder_.outputCount() != 2)
    throw std::runtime_error("STOP RKNN tensor count");
  const auto input = [&](const char* name, std::size_t size) {
    const int index = decoder_.inputIndex(name);
    if (index < 0 || decoder_.inputElements(index) != size)
      throw std::runtime_error(std::string("STOP RKNN input ") + name);
    return index;
  };
  const auto output = [&](const char* name, std::size_t size) {
    const int index = decoder_.outputIndex(name);
    if (index < 0 || decoder_.outputElements(index) != size)
      throw std::runtime_error(std::string("STOP RKNN output ") + name);
    return index;
  };
  first_ = input("first",21);
  last_ = input("last",21);
  requested_ = input("requested_frames",1);
  motion_ = output("motion",kDecoderFrames*21);
  length_ = output("lengths",1);
  const auto calibrated = YAML::LoadFile(profile);
  if (calibrated["policy_name"].as<std::string>() != "l0_v5_amp_1124" ||
      calibrated["joint_names"].as<std::vector<std::string>>() != joint_names ||
      joint_names.size() != stand_.size())
    throw std::runtime_error("STOP AMP calibration/joint order mismatch");
  const auto pose = calibrated["representative_frame"];
  stand_ = pose["q_stand_rad"].as<std::array<double, 21>>();
  target_[2] = pose["pelvis_pos_m"].as<std::array<double, 3>>()[2];
  const auto q = pose["pelvis_quat_xyzw"].as<std::array<double, 4>>();
  const auto rotation = Eigen::Quaterniond(q[3],q[0],q[1],q[2]).normalized().toRotationMatrix();
  const Eigen::Matrix3d local = inverseHeading(rotation)*rotation;
  for (int i=0; i<6; ++i) target_[3+i] = local(i/3,i%3);
  std::copy_n(stand_.begin(),12,target_.begin()+9);
}

StopReferenceFp16::Plan StopReferenceFp16::plan(Entry entry, bool stationary) {
  double max_projection = 0.;
  for (auto& row : entry) {
    if (!std::all_of(row.begin(),row.end(),[](double v){ return std::isfinite(v); }))
      throw std::runtime_error("STOP nonfinite input");
    copyQuaternion(row,quaternion(row));
    for (int j=0; j<12; ++j) {
      const double bounded = std::clamp(row[7+j],kBounds[j][0],kBounds[j][1]);
      max_projection = std::max(max_projection,std::abs(bounded-row[7+j]));
      row[7+j] = bounded;
    }
  }
  const auto& previous = entry[0];
  const auto& first = entry[1];
  const auto& next = entry[2];
  const Eigen::Matrix3d rotation = quaternion(first).toRotationMatrix();
  const Eigen::Matrix3d yaw = inverseHeading(rotation);
  const Eigen::Matrix3d local = yaw*rotation;
  const Eigen::Vector3d origin(first[0],first[1],0.);
  std::array<float,21> endpoint{};
  endpoint[2] = first[2];
  for (int i=0; i<6; ++i) endpoint[3+i] = local(i/3,i%3);
  std::copy_n(first.begin()+7,12,endpoint.begin()+9);
  const float auto_duration = 0.f;
  std::vector<const float*> inputs(3);
  std::vector<std::size_t> sizes(3,21);
  inputs[first_] = endpoint.data();
  inputs[last_] = target_.data();
  inputs[requested_] = &auto_duration;
  sizes[requested_] = 1;
  if (!decoder_.run(inputs,sizes)) throw std::runtime_error("STOP RKNN: " + decoder_.lastError());
  const float predicted = decoder_.output(length_).at(0);
  if (!std::isfinite(predicted) || predicted < 10 || predicted > kDecoderFrames ||
      predicted != std::round(predicted))
    throw std::runtime_error("STOP RKNN invalid duration");
  const int count = static_cast<int>(predicted);
  const auto& output = decoder_.output(motion_);

  double gap = 0.;
  for (int j=0; j<21; ++j) gap = std::max(gap,std::abs(first[7+j]-stand_[j]));
  const int upper_ticks = stationary ? 30 :
      static_cast<int>(std::min(394.,std::ceil(std::max(.8,gap*.85)/.02)));
  const int count_full = std::max(count,stationary ? 31 : upper_ticks+6);
  std::vector<Row> base(count_full);
  for (int i=0; i<count_full; ++i) {
    auto& row = base[i];
    const float* lower = i >= count-1 ? target_.data() : output.data()+21*i;
    const Eigen::Vector3d pos = yaw.transpose()*Eigen::Vector3d(lower[0],lower[1],lower[2])+origin;
    for (int k=0; k<3; ++k) row[k] = pos[k];
    copyQuaternion(row,Eigen::Quaterniond(yaw.transpose()*rotation6(lower)).normalized());
    for (int j=0; j<12; ++j) {
      const double bounded = std::clamp<double>(lower[9+j],kBounds[j][0],kBounds[j][1]);
      max_projection = std::max(max_projection,std::abs(lower[9+j]-bounded));
      row[7+j] = bounded;
    }
    for (int j=19; j<28; ++j) {
      const double c = first[j], delta = next[j]-previous[j];
      if (!stationary && i<5) {
        const double u = i/4., a = 4*(u-6*std::pow(u,3)+8*std::pow(u,4)-3*std::pow(u,5));
        row[j] = c+.5*a*delta+8*u*u*std::pow(1-u,3)*(previous[j]+next[j]-2*c);
      } else {
        const double u = std::min(1.,(i-(stationary ? 0. : 5.))/upper_ticks);
        const double w = stationary ? u*u*(3-2*u) : u*u*u*(10+u*(6*u-15));
        row[j] = c+w*(stand_[j-7]-c)+(stationary ? u*(1-u)*(1-u)*.6*delta/.04 : 0.);
      }
    }
    if (!std::all_of(row.begin(),row.end(),[](double v){return std::isfinite(v);}))
      throw std::runtime_error("STOP nonfinite composed reference");
  }
  base.front() = first;
  Plan result{{first},count,max_projection,0.};
  for (int i=1; i<count_full; ++i) {
    const auto& a = base[i-1];
    const auto& b = base[i];
    double step = 0.;
    for (int j=7; j<28; ++j) step = std::max(step,std::abs(b[j]-a[j]));
    result.max_joint_step = std::max(result.max_joint_step,step);
    const auto room = kMaximumFrames-result.rows.size()-static_cast<std::size_t>(count_full-i-1);
    // Use spare frames for speed repair. A large speed diagnostic still yields a plan.
    const int divisions = step > .14001 ?
        static_cast<int>(std::min<double>(room,std::ceil(step/kStep))) : 1;
    for (int k=1; k<divisions; ++k) {
      const double t = static_cast<double>(k)/divisions;
      Row row;
      for (int j=0; j<28; ++j) row[j] = a[j]+t*(b[j]-a[j]);
      copyQuaternion(row,quaternion(a).slerp(t,quaternion(b)).normalized());
      result.rows.push_back(row);
    }
    result.rows.push_back(b);
  }
  return result;
}
