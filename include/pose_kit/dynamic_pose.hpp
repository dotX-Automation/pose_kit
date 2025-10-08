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
#include <memory>

#include <Eigen/Geometry>

#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/utils.hpp>

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
  using SharedPtr = std::shared_ptr<DynamicPose>;
  using WeakPtr = std::weak_ptr<DynamicPose>;
  using UniquePtr = std::unique_ptr<DynamicPose>;
  using ConstSharedPtr = std::shared_ptr<const DynamicPose>;
  using ConstWeakPtr = std::weak_ptr<const DynamicPose>;

  /**
   * @brief Default constructor.
   */
  DynamicPose();

  /**
   * @brief Copy constructor.
   *
   * @param dp Pose to copy.
   */
  DynamicPose(const DynamicPose & dp);

  /**
   * @brief Constructor with initial position, linear velocity, and linear acceleration.
   *
   * @param x Initial X position [m].
   * @param y Initial Y position [m].
   * @param z Initial Z position [m].
   * @param vx Initial X linear velocity [m/s].
   * @param vy Initial Y linear velocity [m/s].
   * @param vz Initial Z linear velocity [m/s].
   * @param header ROS header.
   */
  DynamicPose(
    double x, double y, double z,
    double vx, double vy, double vz,
    double ax, double ay, double az,
    const std_msgs::msg::Header & header);

  /**
   * @brief Constructor with initial attitude, angular velocity, and angular acceleration.
   *
   * @param q Initial attitude quaternion.
   * @param angular_v Initial angular velocity [rad/s].
   * @param angular_a Initial angular acceleration [rad/s^2].
   * @param header ROS header.
   */
  DynamicPose(
    const tf2::Quaternion & q,
    const tf2::Vector3 & angular_v,
    const tf2::Vector3 & angular_a,
    const std_msgs::msg::Header & header);

  /**
   * @brief Constructor with initial euler angles, angular velocity, and angular acceleration.
   *
   * @param rpy Initial euler angles [rad].
   * @param angular_v Initial angular velocity [rad/s].
   * @param angular_a Initial angular acceleration [rad/s^2].
   * @param header ROS header.
   */
  DynamicPose(
    const tf2::Vector3 & rpy,
    const tf2::Vector3 & angular_v,
    const tf2::Vector3 & angular_a,
    const std_msgs::msg::Header & header);

  /**
   * @brief Constructor with initial position, linear velocity, linear acceleration, and heading.
   *
   * @param x Initial X position [m].
   * @param y Initial Y position [m].
   * @param z Initial Z position [m].
   * @param vx Initial X linear velocity [m/s].
   * @param vy Initial Y linear velocity [m/s].
   * @param vz Initial Z linear velocity [m/s].
   * @param ax Initial X linear acceleration [m/s^2].
   * @param ay Initial Y linear acceleration [m/s^2].
   * @param az Initial Z linear acceleration [m/s^2].
   * @param heading Initial heading [rad].
   * @param header ROS header.
   * @param cov Initial covariance matrix.
   */
  DynamicPose(
    double x, double y, double z,
    double vx, double vy, double vz,
    double ax, double ay, double az,
    double heading,
    const std_msgs::msg::Header & header,
    const std::array<double, 36> & cov);

  /**
   * @brief Constructor with initial position, attitude, linear and angular velocity and acceleration.
   *
   * @param p Initial position [m].
   * @param q Initial attitude quaternion.
   * @param v Initial linear velocity [m/s].
   * @param angular_v Initial angular velocity [rad/s].
   * @param a Initial linear acceleration [m/s^2].
   * @param angular_a Initial angular acceleration [rad/s^2].
   * @param header ROS header.
   * @param cov Initial covariance matrix.
   * @param twist_cov Initial twist covariance matrix.
   * @param accel_cov Initial acceleration covariance matrix.
   */
  DynamicPose(
    const tf2::Vector3 & p,
    const tf2::Quaternion & q,
    const tf2::Vector3 & v,
    const tf2::Vector3 & angular_v,
    const tf2::Vector3 & a,
    const tf2::Vector3 & angular_a,
    const std_msgs::msg::Header & header,
    const std::array<double, 36> & cov = std::array<double, 36>{},
    const std::array<double, 36> & twist_cov = std::array<double, 36>{},
    const std::array<double, 36> & accel_cov = std::array<double, 36>{});

  /**
   * @brief Constructor with initial position, attitude, linear and angular velocity and acceleration, in Eigen format.
   *
   * @param p Initial position [m].
   * @param q Initial attitude quaternion.
   * @param v Initial linear velocity [m/s].
   * @param angular_v Initial angular velocity [rad/s].
   * @param a Initial linear acceleration [m/s^2].
   * @param angular_a Initial angular acceleration [rad/s^2].
   * @param header ROS header.
   * @param cov Initial covariance matrix.
   * @param twist_cov Initial twist covariance matrix.
   * @param accel_cov Initial acceleration covariance matrix.
   */
  DynamicPose(
    const Eigen::Vector3d & p,
    const Eigen::Quaterniond & q,
    const Eigen::Vector3d & v,
    const Eigen::Vector3d & angular_v,
    const Eigen::Vector3d & a,
    const Eigen::Vector3d & angular_a,
    const std_msgs::msg::Header & header,
    const std::array<double, 36> & cov = std::array<double, 36>{},
    const std::array<double, 36> & twist_cov = std::array<double, 36>{},
    const std::array<double, 36> & accel_cov = std::array<double, 36>{});

  DynamicPose(Pose p)
  : KinematicPose(std::move(p)) {}

  DynamicPose(KinematicPose kp)
  : KinematicPose(std::move(kp)) {}

  /**
   * Destructor.
   */
  virtual ~DynamicPose();

  /**
   * @brief Fills and returns a sensor_msgs/Imu message.
   *
   * @param msg sensor_msgs/Imu message to fill.
   */
  void to_imu(sensor_msgs::msg::Imu & msg) const;

  [[nodiscard]] inline const tf2::Vector3 & acceleration() const
  {
    return acceleration_;
  }
  [[nodiscard]] inline const tf2::Vector3 & angular_acceleration() const
  {
    return angular_acceleration_;
  }
  [[nodiscard]] inline const std::array<double, 36> & acceleration_covariance() const
  {
    return acceleration_cov_;
  }

  inline void set_acceleration(const tf2::Vector3 & a)
  {
    acceleration_ = a;
  }
  inline void set_angular_acceleration(const tf2::Vector3 & angular_a)
  {
    angular_acceleration_ = angular_a;
  }
  inline void set_acceleration_covariance(const std::array<double, 36> & a_cov)
  {
    acceleration_cov_ = a_cov;
  }

  /**
   * @brief Copy assignment operator.
   *
   * @param dp DynamicPose to copy.
   */
  DynamicPose & operator=(const DynamicPose & dp);

  /**
   * @brief Move assignment operator.
   *
   * @param dp DynamicPose to copy.
   */
  DynamicPose & operator=(DynamicPose && dp);

protected:
  /* Internal data. */
  tf2::Vector3 acceleration_ = {0.0, 0.0, 0.0}; // [m/s^2]
  tf2::Vector3 angular_acceleration_ = {0.0, 0.0, 0.0}; // [rad/s^2]
  std::array<double, 36> acceleration_cov_{};
};

} // namespace pose_kit

#endif // POSE_KIT__DYNAMIC_POSE_HPP_
