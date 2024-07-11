/**
 * Dynamic pose library, based on Eigen geometry types.
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

#ifndef POSE_KIT__DYNAMIC_POSE_HPP_
#define POSE_KIT__DYNAMIC_POSE_HPP_

#include "visibility_control.h"

#include <array>

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>

#include <pose_kit/kinematic_pose.hpp>

namespace pose_kit
{

/**
 * Represents position, orientation, velocity, and acceleration of an autonomous agent.
 */
class POSE_KIT_PUBLIC DynamicPose : public KinematicPose
{
public:
  /* Constructors. */
  DynamicPose() {}
  DynamicPose(const DynamicPose & dp);
  DynamicPose(
    double x, double y, double z,
    double vx, double vy, double vz,
    double ax, double ay, double az,
    const std_msgs::msg::Header & header);
  DynamicPose(
    const Eigen::Quaterniond & q,
    const Eigen::Vector3d & angular_vel,
    const Eigen::Vector3d & angular_accel,
    const std_msgs::msg::Header & header);
  DynamicPose(
    const Eigen::EulerAnglesXYZd & rpy_angles,
    const Eigen::Vector3d & angular_vel,
    const Eigen::Vector3d & angular_accel,
    const std_msgs::msg::Header & header);
  DynamicPose(
    double x, double y, double z,
    double vx, double vy, double vz,
    double ax, double ay, double az,
    double heading,
    const std_msgs::msg::Header & header,
    const std::array<double, 36> & cov);
  DynamicPose(
    const Eigen::Vector3d & pos,
    const Eigen::Quaterniond & q,
    const Eigen::Vector3d & vel,
    const Eigen::Vector3d & angular_vel,
    const Eigen::Vector3d & accel,
    const Eigen::Vector3d & angular_accel,
    const std_msgs::msg::Header & header,
    const std::array<double, 36> & cov = std::array<double, 36>{},
    const std::array<double, 36> & twist_cov = std::array<double, 36>{},
    const std::array<double, 36> & accel_cov = std::array<double, 36>{});
  DynamicPose(Pose p)
  : KinematicPose(std::move(p)) {}
  DynamicPose(KinematicPose kp)
  : KinematicPose(std::move(kp)) {}

  /* Destructor. */
  virtual ~DynamicPose() {}

  /* ROS interfaces conversion methods. */
  sensor_msgs::msg::Imu to_imu() const;

  /* Getters. */
  inline Eigen::Vector3d get_acceleration() const
  {
    return acceleration_;
  }
  inline Eigen::Vector3d get_angular_acceleration() const
  {
    return angular_acceleration_;
  }
  inline std::array<double, 36> get_acceleration_covariance() const
  {
    return acceleration_cov_;
  }

  /* Setters. */
  inline void set_acceleration(const Eigen::Vector3d & accel)
  {
    acceleration_ = accel;
  }
  inline void set_angular_acceleration(const Eigen::Vector3d & angular_accel)
  {
    angular_acceleration_ = angular_accel;
  }
  inline void set_acceleration_covariance(const std::array<double, 36> & accel_cov)
  {
    acceleration_cov_ = accel_cov;
  }

  /* Assignment operators. */
  DynamicPose & operator=(const DynamicPose & dp);
  DynamicPose & operator=(DynamicPose && dp);

  /* Geometric operations. */
  // TODO

protected:
  /* Internal data. */
  Eigen::Vector3d acceleration_ = {0.0, 0.0, 0.0}; // [m/s^2]
  Eigen::Vector3d angular_acceleration_ = {0.0, 0.0, 0.0}; // [rad/s^2]
  std::array<double, 36> acceleration_cov_{};
};

} // namespace pose_kit

#endif // POSE_KIT__DYNAMIC_POSE_HPP_
