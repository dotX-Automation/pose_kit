/**
 * Pose library.
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

#include "visibility_control.h"

#include <array>
#include <memory>
#include <stdexcept>
#include <string>

#include <dua_math/dua_math.hpp>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/utils.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>

namespace pose_kit
{

/**
 * Represents position and orientation of an autonomous agent.
 */
class POSE_KIT_PUBLIC Pose
{
public:
  using SharedPtr = std::shared_ptr<Pose>;
  using ConstSharedPtr = std::shared_ptr<const Pose>;
  using WeakPtr = std::weak_ptr<Pose>;
  using ConstWeakPtr = std::weak_ptr<const Pose>;
  using UniquePtr = std::unique_ptr<Pose>;
  using ConstUniquePtr = std::unique_ptr<const Pose>;

  using Matrix6d = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;
  using PoseCovariance = std::array<double, 36>;

  // ========== Constructors and destructors ==========

  // ---------- Special members ----------

  Pose() = default;
  Pose(const Pose &) = default;
  Pose(Pose &&) noexcept = default;
  Pose & operator=(const Pose &) = default;
  Pose & operator=(Pose &&) noexcept = default;
  virtual ~Pose() = default;

  // ---------- Canonical constructors ----------

  Pose(
    const tf2::Vector3 & pos,
    const tf2::Quaternion & att,
    const std_msgs::msg::Header & header = std_msgs::msg::Header(),
    const std::string & child_frame_id = std::string(),
    const PoseCovariance & pose_cov = PoseCovariance{});

  Pose(
    const Eigen::Vector3d & pos,
    const Eigen::Quaterniond & att,
    const std_msgs::msg::Header & header = std_msgs::msg::Header(),
    const std::string & child_frame_id = std::string(),
    const PoseCovariance & pose_cov = PoseCovariance{});

  Pose(
    const tf2::Vector3 & pos,
    const tf2::Vector3 & rpy,
    const std_msgs::msg::Header & header = std_msgs::msg::Header(),
    const std::string & child_frame_id = std::string(),
    const PoseCovariance & pose_cov = PoseCovariance{});

  Pose(
    const Eigen::Vector3d & pos,
    const Eigen::Vector3d & rpy,
    const std_msgs::msg::Header & header = std_msgs::msg::Header(),
    const std::string & child_frame_id = std::string(),
    const PoseCovariance & pose_cov = PoseCovariance{});

  // ---------- Factories ----------

  static Pose identity(
    const std_msgs::msg::Header & header = std_msgs::msg::Header(),
    const std::string & child_frame_id = std::string(),
    const PoseCovariance & pose_cov = PoseCovariance{})
  {
    return Pose(
      tf2::Vector3(0.0, 0.0, 0.0),
      tf2::Quaternion::getIdentity(),
      header, child_frame_id, pose_cov);
  }

  static Pose from_position(
    const tf2::Vector3 & pos,
    const std_msgs::msg::Header & header = std_msgs::msg::Header(),
    const std::string & child_frame_id = std::string(),
    const PoseCovariance & pose_cov = PoseCovariance{})
  {
    return Pose(
      pos,
      tf2::Quaternion::getIdentity(),
      header, child_frame_id, pose_cov);
  }

  static Pose from_position(
    const Eigen::Vector3d & pos,
    const std_msgs::msg::Header & header = std_msgs::msg::Header(),
    const std::string & child_frame_id = std::string(),
    const PoseCovariance & pose_cov = PoseCovariance{})
  {
    return Pose(
      pos,
      Eigen::Quaterniond::Identity(),
      header, child_frame_id, pose_cov);
  }

  static Pose from_attitude(
    const tf2::Quaternion & att,
    const std_msgs::msg::Header & header = std_msgs::msg::Header(),
    const std::string & child_frame_id = std::string(),
    const PoseCovariance & pose_cov = PoseCovariance{})
  {
    return Pose(
      tf2::Vector3(0.0, 0.0, 0.0),
      att,
      header, child_frame_id, pose_cov);
  }

  static Pose from_attitude(
    const Eigen::Quaterniond & att,
    const std_msgs::msg::Header & header = std_msgs::msg::Header(),
    const std::string & child_frame_id = std::string(),
    const PoseCovariance & pose_cov = PoseCovariance{})
  {
    return Pose(
      Eigen::Vector3d::Zero(),
      att,
      header, child_frame_id, pose_cov);
  }

  static Pose from_isometry(
    const Eigen::Isometry3d & iso,
    const std_msgs::msg::Header & header = std_msgs::msg::Header(),
    const std::string & child_frame_id = std::string(),
    const PoseCovariance & pose_cov = PoseCovariance{})
  {
    return Pose(
      iso.translation(),
      Eigen::Quaterniond(iso.rotation()),
      header, child_frame_id, pose_cov);
  }

  // ---------- ROS message constructors ----------

  explicit Pose(
    const geometry_msgs::msg::PoseStamped & pose_msg,
    const std::string & child_frame_id = std::string(),
    const PoseCovariance & pose_cov = PoseCovariance{});

  explicit Pose(
    const geometry_msgs::msg::PoseWithCovariance & pose_msg,
    const std_msgs::msg::Header & header = std_msgs::msg::Header(),
    const std::string & child_frame_id = std::string());

  explicit Pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose_msg,
    const std::string & child_frame_id = std::string());

  explicit Pose(
    const geometry_msgs::msg::TransformStamped & tf_msg,
    const PoseCovariance & pose_cov = PoseCovariance{});

  // ========== ROS message converters ==========

  void to_point(geometry_msgs::msg::Point & msg) const;

  void to_quaternion(geometry_msgs::msg::Quaternion & msg) const;

  void to_pose(geometry_msgs::msg::Pose & msg) const;

  void to_pose_stamped(geometry_msgs::msg::PoseStamped & msg) const;

  void to_pose_with_covariance(geometry_msgs::msg::PoseWithCovariance & msg) const;

  void to_pose_with_covariance_stamped(geometry_msgs::msg::PoseWithCovarianceStamped & msg) const;

  void to_transform_stamped(geometry_msgs::msg::TransformStamped & msg) const;

  // ========== Getters and setters ==========

  // ---------- Position ----------

  inline void set_x(double x) {pos_.setX(x);}
  inline void set_y(double y) {pos_.setY(y);}
  inline void set_z(double z) {pos_.setZ(z);}

  [[nodiscard]] inline double x() const {return pos_.x();}
  [[nodiscard]] inline double y() const {return pos_.y();}
  [[nodiscard]] inline double z() const {return pos_.z();}

  inline void set_position(const tf2::Vector3 & pos) {pos_ = pos;}

  inline void set_position(const Eigen::Vector3d & pos)
  {
    set_x(pos.x());
    set_y(pos.y());
    set_z(pos.z());
  }

  [[nodiscard]] inline const tf2::Vector3 & position() const {return pos_;}

  inline void get_position(Eigen::Vector3d & pos) const
  {
    pos.x() = pos_.x();
    pos.y() = pos_.y();
    pos.z() = pos_.z();
  }

  // ---------- Attitude ----------

  inline void set_attitude(const tf2::Quaternion & att) {att_ = att;}

  inline void set_attitude(const Eigen::Quaterniond & att)
  {
    att_.setX(att.x());
    att_.setY(att.y());
    att_.setZ(att.z());
    att_.setW(att.w());
  }

  [[nodiscard]] inline const tf2::Quaternion & attitude() const {return att_;}

  inline void get_attitude(Eigen::Quaterniond & att) const
  {
    att.x() = att_.x();
    att.y() = att_.y();
    att.z() = att_.z();
    att.w() = att_.w();
  }

  // ---------- RPY ----------

  inline void set_rpy(const tf2::Vector3 & rpy)
  {
    tf2::Quaternion att;
    att.setRPY(rpy.x(), rpy.y(), rpy.z());
    set_attitude(att);
  }

  inline void set_rpy(const Eigen::Vector3d & rpy)
  {
    set_rpy(tf2::Vector3(rpy.x(), rpy.y(), rpy.z()));
  }

  inline void get_rpy(tf2::Vector3 & rpy) const
  {
    double roll, pitch, yaw;
    tf2::getEulerYPR(att_, yaw, pitch, roll);
    rpy.setX(roll);
    rpy.setY(pitch);
    rpy.setZ(yaw);
  }

  inline void get_rpy(Eigen::Vector3d & rpy) const
  {
    tf2::Vector3 rpy_vec;
    get_rpy(rpy_vec);
    rpy.x() = rpy_vec.x();
    rpy.y() = rpy_vec.y();
    rpy.z() = rpy_vec.z();
  }

  inline void set_roll(double roll)
  {
    Eigen::Vector3d rpy;
    get_rpy(rpy);
    set_rpy(tf2::Vector3(roll, rpy.y(), rpy.z()));
  }

  inline void set_pitch(double pitch)
  {
    Eigen::Vector3d rpy;
    get_rpy(rpy);
    set_rpy(tf2::Vector3(rpy.x(), pitch, rpy.z()));
  }

  inline void set_yaw(double yaw)
  {
    Eigen::Vector3d rpy;
    get_rpy(rpy);
    set_rpy(tf2::Vector3(rpy.x(), rpy.y(), yaw));
  }

  [[nodiscard]] inline double get_roll() const
  {
    tf2::Vector3 rpy;
    get_rpy(rpy);
    return rpy.x();
  }

  [[nodiscard]] inline double get_pitch() const
  {
    tf2::Vector3 rpy;
    get_rpy(rpy);
    return rpy.y();
  }

  [[nodiscard]] inline double get_yaw() const
  {
    tf2::Vector3 rpy;
    get_rpy(rpy);
    return rpy.z();
  }

  [[nodiscard]] inline double get_heading() const
  {
    tf2::Matrix3x3 m(att_);
    const tf2::Vector3 x_axis = m.getColumn(0);
    return std::atan2(x_axis.y(), x_axis.x());
  }

  [[nodiscard]] inline double get_forward_tilt() const
  {
    tf2::Matrix3x3 m(att_);
    const tf2::Vector3 x_axis = m.getColumn(0);
    return std::atan2(-x_axis.z(), std::hypot(x_axis.x(), x_axis.y()));
  }

  [[nodiscard]] inline double get_lateral_elevation() const
  {
    tf2::Matrix3x3 m(att_);
    const tf2::Vector3 y_axis = m.getColumn(1);
    return std::atan2(-y_axis.z(), std::hypot(y_axis.x(), y_axis.y()));
  }

  // ---------- Isometry ----------

  inline void set_isometry(const Eigen::Isometry3d & iso)
  {
    set_position(iso.translation());
    set_attitude(Eigen::Quaterniond(iso.rotation()));
  }

  inline void get_isometry(Eigen::Isometry3d & iso) const
  {
    iso = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond att = Eigen::Quaterniond::Identity();
    get_attitude(att);
    iso.rotate(att);
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    get_position(pos);
    iso.pretranslate(pos);
  }

  // ---------- Pose Covariance ----------

  inline void set_pose_covariance(const PoseCovariance & pose_cov) {pose_cov_ = pose_cov;}

  [[nodiscard]] inline const PoseCovariance & pose_covariance() const {return pose_cov_;}

  // ---------- Header ----------

  inline void set_header(const std_msgs::msg::Header & header) {header_ = header;}

  [[nodiscard]] inline const std_msgs::msg::Header & header() const {return header_;}

  // ---------- Frame ids ----------

  inline void set_parent_frame_id(const std::string & frame_id) {header_.set__frame_id(frame_id);}

  inline void set_child_frame_id(const std::string & frame_id) {child_frame_id_ = frame_id;}

  [[nodiscard]] inline const std::string & child_frame_id() const {return child_frame_id_;}

  [[nodiscard]] inline const std::string & parent_frame_id() const {return header_.frame_id;}

  // ---------- Time ----------

  inline void set_timestamp(const rclcpp::Time & time) {header_.stamp = time;}

  inline void get_timestamp(rclcpp::Time & time) const {time = header_.stamp;}

  inline void set_timestamp_s(uint64_t timestamp_s)
  {
    header_.stamp.set__sec(static_cast<int32_t>(timestamp_s));
    header_.stamp.set__nanosec(0u);
  }

  inline uint64_t get_timestamp_s() const {return static_cast<uint64_t>(header_.stamp.sec);}

  inline void set_timestamp_ms(uint64_t timestamp_ms)
  {
    constexpr uint64_t K = 1000ULL;
    header_.stamp.set__sec(static_cast<int32_t>(timestamp_ms / K));
    header_.stamp.set__nanosec(static_cast<uint32_t>((timestamp_ms % K) * 1000000ULL));
  }

  inline uint64_t get_timestamp_ms() const
  {
    return static_cast<uint64_t>(header_.stamp.sec) * 1000ULL +
           static_cast<uint64_t>(header_.stamp.nanosec) / 1000000ULL;
  }

  inline void set_timestamp_us(uint64_t timestamp_us)
  {
    constexpr uint64_t K = 1000000ULL;
    header_.stamp.set__sec(static_cast<int32_t>(timestamp_us / K));
    header_.stamp.set__nanosec(static_cast<uint32_t>((timestamp_us % K) * 1000ULL));
  }

  inline uint64_t get_timestamp_us() const
  {
    return static_cast<uint64_t>(header_.stamp.sec) * 1000000ULL +
           static_cast<uint64_t>(header_.stamp.nanosec) / 1000ULL;
  }

  inline void set_timestamp_ns(uint64_t timestamp_ns)
  {
    constexpr uint64_t K = 1000000000ULL;
    header_.stamp.set__sec(static_cast<int32_t>(timestamp_ns / K));
    header_.stamp.set__nanosec(static_cast<uint32_t>(timestamp_ns % K));
  }

  inline uint64_t get_timestamp_ns() const
  {
    return static_cast<uint64_t>(header_.stamp.sec) * 1000000000ULL +
           static_cast<uint64_t>(header_.stamp.nanosec);
  }

  // ========== Main methods ==========

  /**
   * @brief Compute the inverse pose transformation.
   *
   * If this pose represents T_parent_child, the inverse represents T_child_parent.
   */
  [[nodiscard]] Pose inverse() const;

  /**
   * @brief Change the parent frame of the pose (left-multiply by a transform).
   *
   * The stored pose is interpreted as the rigid transform:
   *   T_parent_child
   *
   * The input TF is interpreted as:
   *   T_target_source
   * with:
   *   - source == current parent frame
   *   - target == new parent frame
   */
  virtual void change_parent_frame(const Pose & pose);

  void apply_pre_transform(const geometry_msgs::msg::TransformStamped & tf)
  {
    change_parent_frame(Pose(tf));
  }

  /**
   * @brief Change the child frame of the pose (right-multiply by the inverse of a transform).
   *
   * The stored pose is interpreted as:
   *   T_parent_child
   *
   * The input TF is interpreted as:
   *   T_target_source
   * with:
   *   - source == current child frame
   *   - target == new child frame
   *
   * Pose update:
   *   T_parent_target = T_parent_child · (T_target_source)^{-1}
   */
  virtual void change_child_frame_inverse(const Pose & pose);

  void apply_post_inverse_transform(const geometry_msgs::msg::TransformStamped & tf)
  {
    change_child_frame_inverse(Pose(tf));
  }

  /**
   * @brief Simultaneously change parent frame and child frame.
   *
   * The stored pose is interpreted as:
   *   T_parent_child
   *
   * Two transforms are provided:
   *   - tf_pre  : Tpre_target_source, with presource  == current parent frame
   *   - tf_post : Tpost_target_source, with postsource == current child frame
   *
   * Pose update:
   *   T_target_pre_target_post = Tpre_target_source · T_parent_child · (Tpost_target_source)^{-1}
   */
  virtual void change_frames(const Pose & pose_pre, const Pose & pose_post);

  void apply_transform_chain(
    const geometry_msgs::msg::TransformStamped & tf_pre,
    const geometry_msgs::msg::TransformStamped & tf_post)
  {
    change_frames(Pose(tf_pre), Pose(tf_post));
  }

protected:
  // ========== Internal variables  ==========

  tf2::Vector3 pos_{0.0, 0.0, 0.0}; // [m]
  tf2::Quaternion att_{0.0, 0.0, 0.0, 1.0};
  std_msgs::msg::Header header_{};
  std::string child_frame_id_{};
  PoseCovariance pose_cov_{};
};

} // namespace pose_kit
