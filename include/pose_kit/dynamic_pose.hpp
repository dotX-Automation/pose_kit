/**
 * Dynamic pose library, based on Eigen geometry types.
 *
 * February 13, 2026
 */

/**
 * Copyright 2026 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <pose_kit/kinematic_pose.hpp>

namespace pose_kit
{

/**
 * Represents position, orientation, velocity, and acceleration of an autonomous agent.
 */
class POSE_KIT_PUBLIC DynamicPose : public KinematicPose
{
public:
  using SharedPtr = std::shared_ptr<DynamicPose>;
  using ConstSharedPtr = std::shared_ptr<const DynamicPose>;
  using WeakPtr = std::weak_ptr<DynamicPose>;
  using ConstWeakPtr = std::weak_ptr<const DynamicPose>;
  using UniquePtr = std::unique_ptr<DynamicPose>;
  using ConstUniquePtr = std::unique_ptr<const DynamicPose>;

  using Matrix6d = Pose::Matrix6d;
  using PoseCovariance = Pose::PoseCovariance;
  using TwistCovariance = KinematicPose::TwistCovariance;
  using AccelCovariance = std::array<double, 36>;

  // ========== Constructors and destructors ==========

  // ---------- Special members ----------

  DynamicPose() = default;
  DynamicPose(const DynamicPose &) = default;
  DynamicPose(DynamicPose &&) noexcept = default;
  DynamicPose & operator=(const DynamicPose &) = default;
  DynamicPose & operator=(DynamicPose &&) noexcept = default;
  ~DynamicPose() override = default;

  // ---------- Canonical constructors ----------

  DynamicPose(
    const tf2::Vector3 & pos,
    const tf2::Quaternion & att,
    const tf2::Vector3 & lin_vel,
    const tf2::Vector3 & ang_vel,
    const tf2::Vector3 & lin_accel,
    const tf2::Vector3 & ang_accel,
    const std_msgs::msg::Header & header = std_msgs::msg::Header(),
    const std::string & child_frame_id = std::string(),
    const PoseCovariance & pose_cov = PoseCovariance{},
    const TwistCovariance & twist_cov = TwistCovariance{},
    const AccelCovariance & accel_cov = AccelCovariance{});

  DynamicPose(
    const Eigen::Vector3d & pos,
    const Eigen::Quaterniond & att,
    const Eigen::Vector3d & lin_vel,
    const Eigen::Vector3d & ang_vel,
    const Eigen::Vector3d & lin_accel,
    const Eigen::Vector3d & ang_accel,
    const std_msgs::msg::Header & header = std_msgs::msg::Header(),
    const std::string & child_frame_id = std::string(),
    const PoseCovariance & pose_cov = PoseCovariance{},
    const TwistCovariance & twist_cov = TwistCovariance{},
    const AccelCovariance & accel_cov = AccelCovariance{});

  // ========== Getters and setters ==========

  // ---------- Linear acceleration ----------

  inline void set_linear_acceleration(const tf2::Vector3 & lin_accel) {lin_accel_ = lin_accel;}

  inline void set_linear_acceleration(const Eigen::Vector3d & lin_accel)
  {
    lin_accel_.setX(lin_accel.x());
    lin_accel_.setY(lin_accel.y());
    lin_accel_.setZ(lin_accel.z());
  }

  [[nodiscard]] inline const tf2::Vector3 & linear_acceleration() const {return lin_accel_;}

  inline void get_linear_acceleration(Eigen::Vector3d & lin_accel) const
  {
    lin_accel.x() = lin_accel_.x();
    lin_accel.y() = lin_accel_.y();
    lin_accel.z() = lin_accel_.z();
  }

  // ---------- Angular acceleration ----------

  inline void set_angular_acceleration(const tf2::Vector3 & ang_accel) {ang_accel_ = ang_accel;}

  inline void set_angular_acceleration(const Eigen::Vector3d & ang_accel)
  {
    ang_accel_.setX(ang_accel.x());
    ang_accel_.setY(ang_accel.y());
    ang_accel_.setZ(ang_accel.z());
  }

  [[nodiscard]] inline const tf2::Vector3 & angular_acceleration() const {return ang_accel_;}

  inline void get_angular_acceleration(Eigen::Vector3d & ang_accel) const
  {
    ang_accel.x() = ang_accel_.x();
    ang_accel.y() = ang_accel_.y();
    ang_accel.z() = ang_accel_.z();
  }

  // ---------- Acceleration covariance ----------

  inline void set_acceleration_covariance(const AccelCovariance & accel_cov)
  {
    accel_cov_ = accel_cov;
  }

  [[nodiscard]] inline const AccelCovariance & acceleration_covariance() const {return accel_cov_;}

  // ========== Main methods ==========

  void change_parent_frame(const Pose & pose) override;

  void change_child_frame_inverse(const Pose & pose) override;

  void change_frames(const Pose & pose_pre, const Pose & pose_post) override;

protected:
  // ========== Internal variables  ==========

  tf2::Vector3 lin_accel_{0.0, 0.0, 0.0}; // [m/s^2]
  tf2::Vector3 ang_accel_{0.0, 0.0, 0.0}; // [rad/s^2]
  AccelCovariance accel_cov_{};
};

} // namespace pose_kit
