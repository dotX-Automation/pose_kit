/**
 * Kinematic pose library.
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

#include <pose_kit/pose.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

namespace pose_kit
{

/**
 * Represents position, orientation, and velocity of an autonomous agent.
 */

class POSE_KIT_PUBLIC KinematicPose : public Pose
{
public:
  using SharedPtr = std::shared_ptr<KinematicPose>;
  using ConstSharedPtr = std::shared_ptr<const KinematicPose>;
  using WeakPtr = std::weak_ptr<KinematicPose>;
  using ConstWeakPtr = std::weak_ptr<const KinematicPose>;
  using UniquePtr = std::unique_ptr<KinematicPose>;
  using ConstUniquePtr = std::unique_ptr<const KinematicPose>;

  using Matrix6d = Pose::Matrix6d;
  using PoseCovariance = Pose::PoseCovariance;
  using TwistCovariance = std::array<double, 36>;

  // ========== Constructors and destructors ==========

  // ---------- Special members ----------

  KinematicPose() = default;
  KinematicPose(const KinematicPose &) = default;
  KinematicPose(KinematicPose &&) noexcept = default;
  KinematicPose & operator=(const KinematicPose &) = default;
  KinematicPose & operator=(KinematicPose &&) noexcept = default;
  ~KinematicPose() override = default;

  // ---------- Canonical constructors ----------

  KinematicPose(
    const tf2::Vector3 & pos,
    const tf2::Quaternion & att,
    const tf2::Vector3 & lin_vel,
    const tf2::Vector3 & ang_vel,
    const std_msgs::msg::Header & header = std_msgs::msg::Header(),
    const std::string & child_frame_id = std::string(),
    const PoseCovariance & pose_cov = PoseCovariance{},
    const TwistCovariance & twist_cov = TwistCovariance{});

  KinematicPose(
    const Eigen::Vector3d & pos,
    const Eigen::Quaterniond & att,
    const Eigen::Vector3d & lin_vel,
    const Eigen::Vector3d & ang_vel,
    const std_msgs::msg::Header & header = std_msgs::msg::Header(),
    const std::string & child_frame_id = std::string(),
    const PoseCovariance & pose_cov = PoseCovariance{},
    const TwistCovariance & twist_cov = TwistCovariance{});

  // ---------- ROS message constructors ----------

  explicit KinematicPose(
    const geometry_msgs::msg::PoseStamped & pose_msg,
    const geometry_msgs::msg::TwistStamped & twist_msg,
    const std::string & child_frame_id = std::string(),
    const PoseCovariance & pose_cov = PoseCovariance{},
    const TwistCovariance & twist_cov = TwistCovariance{});

  explicit KinematicPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose_msg,
    const geometry_msgs::msg::TwistWithCovarianceStamped & twist_msg,
    const std::string & child_frame_id = std::string());

  // ========== ROS message converters ==========

  void to_twist(geometry_msgs::msg::Twist & msg) const;

  void to_twist_stamped(geometry_msgs::msg::TwistStamped & msg) const;

  void to_twist_with_covariance(geometry_msgs::msg::TwistWithCovariance & msg) const;

  void to_twist_with_covariance_stamped(geometry_msgs::msg::TwistWithCovarianceStamped & msg) const;

  // ========== Getters and setters ==========

  // ---------- Linear velocity ----------

  inline void set_linear_velocity(const tf2::Vector3 & lin_vel) {lin_vel_ = lin_vel;}

  inline void set_linear_velocity(const Eigen::Vector3d & lin_vel)
  {
    lin_vel_.setX(lin_vel.x());
    lin_vel_.setY(lin_vel.y());
    lin_vel_.setZ(lin_vel.z());
  }

  [[nodiscard]] inline const tf2::Vector3 & linear_velocity() const {return lin_vel_;}

  inline void get_linear_velocity(Eigen::Vector3d & lin_vel) const
  {
    lin_vel.x() = lin_vel_.x();
    lin_vel.y() = lin_vel_.y();
    lin_vel.z() = lin_vel_.z();
  }

  // ---------- Angular velocity ----------

  inline void set_angular_velocity(const tf2::Vector3 & w) {ang_vel_ = w;}

  inline void set_angular_velocity(const Eigen::Vector3d & w)
  {
    ang_vel_.setX(w.x());
    ang_vel_.setY(w.y());
    ang_vel_.setZ(w.z());
  }

  [[nodiscard]] inline const tf2::Vector3 & angular_velocity() const {return ang_vel_;}

  inline void get_angular_velocity(Eigen::Vector3d & w) const
  {
    w.x() = ang_vel_.x();
    w.y() = ang_vel_.y();
    w.z() = ang_vel_.z();
  }

  // ---------- Twist covariance ----------

  inline void set_twist_covariance(const TwistCovariance & c) {twist_cov_ = c;}

  [[nodiscard]] inline const TwistCovariance & twist_covariance() const {return twist_cov_;}

  // ========== Main methods ==========

  void change_parent_frame(const Pose & pose) override;

  void change_child_frame_inverse(const Pose & pose) override;

  void change_frames(const Pose & pose_pre, const Pose & pose_post) override;

protected:
  // ========== Internal variables  ==========

  tf2::Vector3 lin_vel_{0.0, 0.0, 0.0}; // [m/s]
  tf2::Vector3 ang_vel_{0.0, 0.0, 0.0}; // [rad/s]
  TwistCovariance twist_cov_{};
};

} // namespace pose_kit
