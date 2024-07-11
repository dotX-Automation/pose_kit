/**
 * Pose library, based on Eigen geometry types.
 *
 * Roberto Masocco <r.masocco@dotxautomation.com>
 *
 * April 14, 2023
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
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

#ifndef POSE_KIT__POSE_HPP_
#define POSE_KIT__POSE_HPP_

#include "visibility_control.h"

#include <array>
#include <stdexcept>

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

#include <rclcpp/rclcpp.hpp>

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
  /* Constructors. */
  Pose();
  Pose(const Pose & p);
  Pose(
    double x, double y, double z,
    const std_msgs::msg::Header & header);
  Pose(
    const Eigen::Quaterniond & q,
    const std_msgs::msg::Header & header);
  Pose(
    const Eigen::EulerAnglesXYZd & rpy_angles,
    const std_msgs::msg::Header & header);
  Pose(
    double x, double y, double z, double heading,
    const std_msgs::msg::Header & header,
    const std::array<double, 36> & cov = std::array<double, 36>{});
  Pose(
    const Eigen::Vector3d & pos,
    const Eigen::Quaterniond & q,
    const std_msgs::msg::Header & header,
    const std::array<double, 36> & cov = std::array<double, 36>{});

  /* Constructors from ROS messages. */
  Pose(const geometry_msgs::msg::PoseStamped & msg);
  Pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);

  /* Destructor. */
  virtual ~Pose();

  /* ROS interfaces conversion methods. */
  geometry_msgs::msg::PoseStamped to_pose_stamped();
  geometry_msgs::msg::PoseWithCovarianceStamped to_pose_with_covariance_stamped();

  /* Getters. */
  inline Eigen::Vector3d get_position() const
  {
    return position_;
  }
  inline Eigen::Quaterniond get_attitude() const
  {
    return attitude_;
  }
  inline Eigen::EulerAnglesXYZd get_rpy() const
  {
    return rpy_;
  }
  inline Eigen::AngleAxisd get_rotation() const
  {
    return Eigen::AngleAxisd(attitude_);
  }
  inline Eigen::Translation3d get_translation() const
  {
    return Eigen::Translation3d(position_);
  }
  inline Eigen::Isometry3d get_isometry() const
  {
    Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
    isometry.rotate(this->get_rotation());
    isometry.pretranslate(this->get_translation().vector());
    return isometry;
  }
  inline std::array<double, 36> get_pose_covariance() const
  {
    return pose_covariance_;
  }
  inline std_msgs::msg::Header get_header() const
  {
    return header_;
  }
  inline uint64_t get_timestamp_ns() const
  {
    return header_.stamp.sec * 1e9 + header_.stamp.nanosec;
  }
  inline uint64_t get_timestamp_us() const
  {
    return header_.stamp.sec * 1e6 + header_.stamp.nanosec / 1e3;
  }
  inline uint64_t get_timestamp_ms() const
  {
    return header_.stamp.sec * 1e3 + header_.stamp.nanosec / 1e6;
  }
  inline uint64_t get_timestamp_s() const
  {
    return header_.stamp.sec;
  }
  inline std::string get_frame_id() const
  {
    return header_.frame_id;
  }

  /* Setters. */
  inline void set_position(const Eigen::Vector3d & pos)
  {
    position_ = pos;
  }
  inline void set_attitude(const Eigen::Quaterniond & q)
  {
    attitude_ = q;
    rpy_ = Eigen::EulerAnglesXYZd(q);
  }
  inline void set_rpy(const Eigen::EulerAnglesXYZd & rpy_angles)
  {
    rpy_ = rpy_angles;
    attitude_ = Eigen::Quaterniond(rpy_angles);
  }
  inline void set_pose_covariance(const std::array<double, 36> & cov)
  {
    pose_covariance_ = cov;
  }
  inline void set_header(const std_msgs::msg::Header & header)
  {
    header_ = header;
  }
  inline void set_timestamp(const rclcpp::Time & time_point)
  {
    header_.set__stamp(time_point);
  }
  inline void set_timestamp_ns(uint64_t timestamp_ns)
  {
    header_.stamp.set__sec(timestamp_ns / 1e9);
    header_.stamp.set__nanosec(timestamp_ns % static_cast<uint64_t>(1e9));
  }
  inline void set_timestamp_us(uint64_t timestamp_us)
  {
    header_.stamp.set__sec(timestamp_us / 1e6);
    header_.stamp.set__nanosec((timestamp_us % static_cast<uint64_t>(1e6)) * 1e3);
  }
  inline void set_timestamp_ms(uint64_t timestamp_ms)
  {
    header_.stamp.set__sec(timestamp_ms / 1e3);
    header_.stamp.set__nanosec((timestamp_ms % static_cast<uint64_t>(1e3)) * 1e6);
  }
  inline void set_timestamp_s(uint64_t timestamp_s)
  {
    header_.stamp.set__sec(timestamp_s);
    header_.stamp.set__nanosec(0);
  }
  inline void set_frame_id(const std::string & frame_id)
  {
    header_.set__frame_id(frame_id);
  }

  /* Geometric operations. */
  virtual void rigid_transform(
    const geometry_msgs::msg::TransformStamped & tf,
    const std::string & new_frame_id = "");

  /* Assignment operators. */
  Pose & operator=(const Pose & p);
  Pose & operator=(Pose && p);

protected:
  /* Internal data. */
  Eigen::Vector3d position_ = {0.0, 0.0, 0.0}; // [m]
  Eigen::Quaterniond attitude_ = Eigen::Quaterniond::Identity();
  Eigen::EulerAnglesXYZd rpy_ = {0.0, 0.0, 0.0}; // [rad] in [-PI +PI]
  std::array<double, 36> pose_covariance_{};
  std_msgs::msg::Header header_{};
};

} // namespace pose_kit

#endif // POSE_KIT__POSE_HPP_
