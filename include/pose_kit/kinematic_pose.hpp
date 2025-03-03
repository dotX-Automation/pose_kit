/**
 * Kinematic pose library, based on Eigen geometry types.
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

#ifndef POSE_KIT__KINEMATIC_POSE_HPP_
#define POSE_KIT__KINEMATIC_POSE_HPP_

#include "visibility_control.h"

#include <array>
#include <memory>

#include <Eigen/Geometry>

#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/utils.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_msgs/msg/header.hpp>

#include <pose_kit/pose.hpp>

namespace pose_kit
{

/**
 * Represents position, orientation, and velocity of an autonomous agent.
 */
class POSE_KIT_PUBLIC KinematicPose : public Pose
{
public:
  using SharedPtr = std::shared_ptr<KinematicPose>;
  using WeakPtr = std::weak_ptr<KinematicPose>;
  using UniquePtr = std::unique_ptr<KinematicPose>;
  using ConstSharedPtr = std::shared_ptr<const KinematicPose>;
  using ConstWeakPtr = std::weak_ptr<const KinematicPose>;

  /**
   * @brief Default constructor.
   */
  KinematicPose();

  /**
   * @brief Copy constructor.
   *
   * @param kp Pose to copy.
   */
  KinematicPose(const KinematicPose & kp);

  /**
   * @brief Constructor with initial position and linear velocity.
   *
   * @param x Initial X position [m].
   * @param y Initial Y position [m].
   * @param z Initial Z position [m].
   * @param vx Initial X linear velocity [m/s].
   * @param vy Initial Y linear velocity [m/s].
   * @param vz Initial Z linear velocity [m/s].
   * @param header ROS header.
   */
  KinematicPose(
    double x, double y, double z,
    double vx, double vy, double vz,
    const std_msgs::msg::Header & header);

  /**
   * @brief Constructor with initial attitude and angular velocity.
   *
   * @param q Initial attitude quaternion.
   * @param angular_v Initial angular velocity [rad/s].
   * @param header ROS header.
   */
  KinematicPose(
    const tf2::Quaternion & q,
    const tf2::Vector3 & angular_v,
    const std_msgs::msg::Header & header);

  /**
   * @brief Constructor with initial euler angles and angular velocity.
   *
   * @param rpy Initial euler angles [rad].
   * @param angular_v Initial angular velocity [rad/s].
   * @param header ROS header.
   */
  KinematicPose(
    const tf2::Vector3 & rpy,
    const tf2::Vector3 & angular_v,
    const std_msgs::msg::Header & header);

  /**
   * @brief Constructor with initial position, linear velocity, and heading.
   *
   * @param x Initial X position [m].
   * @param y Initial Y position [m].
   * @param z Initial Z position [m].
   * @param vx Initial X linear velocity [m/s].
   * @param vy Initial Y linear velocity [m/s].
   * @param vz Initial Z linear velocity [m/s].
   * @param heading Initial heading [rad].
   * @param header ROS header.
   * @param cov Initial covariance matrix.
   */
  KinematicPose(
    double x, double y, double z,
    double vx, double vy, double vz,
    double heading,
    const std_msgs::msg::Header & header,
    const std::array<double, 36> & cov = std::array<double, 36>{});

  /**
   * @brief Constructor with initial position, attitude, linear and angular velocity.
   *
   * @param p Initial position [m].
   * @param q Initial attitude quaternion.
   * @param vel Initial linear velocity [m/s].
   * @param angular_v Initial angular velocity [rad/s].
   * @param header ROS header.
   * @param cov Initial covariance matrix.
   * @param twist_cov Initial twist covariance matrix.
   */
  KinematicPose(
    const tf2::Vector3 & p,
    const tf2::Quaternion & q,
    const tf2::Vector3 & v,
    const tf2::Vector3 & angular_v,
    const std_msgs::msg::Header & header,
    const std::array<double, 36> & cov = std::array<double, 36>{},
    const std::array<double, 36> & twist_cov = std::array<double, 36>{});

  /**
   * @brief Constructor that builds from a PoseStamped and a TwistStamped ROS messages.
   *
   * @param pose_stamped PoseStamped ROS message.
   * @param twist_stamped TwistStamped ROS message.
   * @param header ROS header to use (to conciliate the two headers of the messages).
   */
  KinematicPose(
    const geometry_msgs::msg::PoseStamped & pose_stamped,
    const geometry_msgs::msg::TwistStamped & twist_stamped,
    const std_msgs::msg::Header & header);

  /**
   * @brief Constructor that builds from a PoseWithCovarianceStamped and a TwistWithCovarianceStamped ROS messages.
   *
   * @param pose_with_cov_stamped PoseWithCovarianceStamped ROS message.
   * @param twist_with_cov_stamped TwistWithCovarianceStamped ROS message.
   * @param header ROS header to use (to conciliate the two headers of the messages).
   */
  KinematicPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose_with_cov_stamped,
    const geometry_msgs::msg::TwistWithCovarianceStamped & twist_with_cov_stamped,
    const std_msgs::msg::Header & header);

  KinematicPose(Pose p)
  : Pose(std::move(p)) {}

  /**
   * @brief Destructor.
   */
  virtual ~KinematicPose();

  /**
   * @brief Fills a TwistStamped ROS message.
   *
   * @return TwistStamped TwistStamped ROS message.
   */
  void to_twist_stamped(geometry_msgs::msg::TwistStamped & msg) const;

  /**
   * @brief Fills a TwistWithCovarianceStamped ROS message.
   *
   * @return TwistWithCovarianceStamped TwistWithCovarianceStamped ROS message.
   */
  void to_twist_with_covariance_stamped(geometry_msgs::msg::TwistWithCovarianceStamped & msg) const;

  [[nodiscard]] inline const tf2::Vector3 & velocity() const
  {
    return velocity_;
  }
  [[nodiscard]] inline const tf2::Vector3 & angular_velocity() const
  {
    return angular_velocity_;
  }
  [[nodiscard]] inline const std::array<double, 36> & twist_covariance() const
  {
    return twist_covariance_;
  }

  inline void set_velocity(const tf2::Vector3 & v)
  {
    velocity_ = v;
  }
  inline void set_angular_velocity(const tf2::Vector3 & angular_v)
  {
    angular_velocity_ = angular_v;
  }
  inline void set_twist_covariance(const std::array<double, 36> & twist_cov)
  {
    twist_covariance_ = twist_cov;
  }

  /**
   * @brief Applies a rigid transformation to the pose.
   *
   * @param tf ROS transformation to apply.
   * @param new_frame_id New frame ID to set (optional).
   */
  void rigid_transform(
    const geometry_msgs::msg::TransformStamped & tf,
    const std::string & new_frame_id = "") override;

  /**
   * @brief Copy assignment operator.
   *
   * @param kp KinematicPose to copy.
   */
  KinematicPose & operator=(const KinematicPose & kp);

  /**
   * @brief Move assignment operator.
   *
   * @param kp KinematicPose to copy.
   */
  KinematicPose & operator=(KinematicPose && kp);

protected:
  /* Internal data. */
  tf2::Vector3 velocity_ = {0.0, 0.0, 0.0}; // [m/s]
  tf2::Vector3 angular_velocity_ = {0.0, 0.0, 0.0}; // [rad/s]
  std::array<double, 36> twist_covariance_{};
};

} // namespace pose_kit

#endif // POSE_KIT__KINEMATIC_POSE_HPP_
