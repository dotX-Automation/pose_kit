/**
 * Dynamic pose library implementation.
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

#include <pose_kit/dynamic_pose.hpp>

namespace pose_kit
{

/**
 * @brief Copy constructor.
 *
 * @param dp Pose to copy.
 */
DynamicPose::DynamicPose(const DynamicPose & dp)
: KinematicPose(dynamic_cast<const KinematicPose &>(dp))
{
  this->set_acceleration(dp.get_acceleration());
  this->set_angular_acceleration(dp.get_angular_acceleration());
}

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
DynamicPose::DynamicPose(
  double x, double y, double z,
  double vx, double vy, double vz,
  double ax, double ay, double az,
  const std_msgs::msg::Header & header)
: KinematicPose(x, y, z, vx, vy, vz, header)
{
  this->set_acceleration(Eigen::Vector3d(ax, ay, az));
  this->set_angular_acceleration(Eigen::Vector3d::Zero());
}

/**
 * @brief Constructor with initial attitude, angular velocity, and angular acceleration.
 *
 * @param q Initial attitude quaternion.
 * @param angular_vel Initial angular velocity [rad/s].
 * @param angular_accel Initial angular acceleration [rad/s^2].
 * @param header ROS header.
 */
DynamicPose::DynamicPose(
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & angular_vel,
  const Eigen::Vector3d & angular_accel,
  const std_msgs::msg::Header & header)
: KinematicPose(q, angular_vel, header)
{
  this->set_acceleration(Eigen::Vector3d::Zero());
  this->set_angular_acceleration(angular_accel);
}

/**
 * @brief Constructor with initial euler angles, angular velocity, and angular acceleration.
 *
 * @param rpy_angles Initial euler angles [rad].
 * @param angular_vel Initial angular velocity [rad/s].
 * @param angular_accel Initial angular acceleration [rad/s^2].
 * @param header ROS header.
 */
DynamicPose::DynamicPose(
  const Eigen::EulerAnglesXYZd & rpy_angles,
  const Eigen::Vector3d & angular_vel,
  const Eigen::Vector3d & angular_accel,
  const std_msgs::msg::Header & header)
: KinematicPose(rpy_angles, angular_vel, header)
{
  this->set_acceleration(Eigen::Vector3d::Zero());
  this->set_angular_acceleration(angular_accel);
}

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
DynamicPose::DynamicPose(
  double x, double y, double z,
  double vx, double vy, double vz,
  double ax, double ay, double az,
  double heading,
  const std_msgs::msg::Header & header,
  const std::array<double, 36> & cov)
: KinematicPose(x, y, z, vx, vy, vz, heading, header, cov)
{
  this->set_acceleration(Eigen::Vector3d(ax, ay, az));
  this->set_angular_acceleration(Eigen::Vector3d::Zero());
}

/**
 * @brief Constructor with initial position, attitude, linear and angular velocity and acceleration.
 *
 * @param pos Initial position [m].
 * @param q Initial attitude quaternion.
 * @param vel Initial linear velocity [m/s].
 * @param angular_vel Initial angular velocity [rad/s].
 * @param accel Initial linear acceleration [m/s^2].
 * @param angular_accel Initial angular acceleration [rad/s^2].
 * @param header ROS header.
 * @param cov Initial covariance matrix.
 * @param twist_cov Initial twist covariance matrix.
 * @param accel_cov Initial acceleration covariance matrix.
 */
DynamicPose::DynamicPose(
  const Eigen::Vector3d & pos,
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & vel,
  const Eigen::Vector3d & angular_vel,
  const Eigen::Vector3d & accel,
  const Eigen::Vector3d & angular_accel,
  const std_msgs::msg::Header & header,
  const std::array<double, 36> & cov,
  const std::array<double, 36> & twist_cov,
  const std::array<double, 36> & accel_cov)
: KinematicPose(pos, q, vel, angular_vel, header, cov, twist_cov)
{
  this->set_acceleration(accel);
  this->set_angular_acceleration(angular_accel);
  this->set_acceleration_covariance(accel_cov);
}

/**
 * @brief Copy assignment operator.
 *
 * @param dp DynamicPose to copy.
 */
DynamicPose & DynamicPose::operator=(const DynamicPose & dp)
{
  this->set_position(dp.get_position());
  this->set_attitude(dp.get_attitude());
  this->set_header(dp.get_header());
  this->set_velocity(dp.get_velocity());
  this->set_angular_velocity(dp.get_angular_velocity());
  this->set_acceleration(dp.get_acceleration());
  this->set_angular_acceleration(dp.get_angular_acceleration());
  this->set_acceleration_covariance(dp.get_acceleration_covariance());
  return *this;
}

/**
 * @brief Move assignment operator.
 *
 * @param dp DynamicPose to copy.
 */
DynamicPose & DynamicPose::operator=(DynamicPose && dp)
{
  this->set_position(dp.get_position());
  this->set_attitude(dp.get_attitude());
  this->set_header(dp.get_header());
  this->set_velocity(dp.get_velocity());
  this->set_angular_velocity(dp.get_angular_velocity());
  this->set_acceleration(dp.get_acceleration());
  this->set_angular_acceleration(dp.get_angular_acceleration());
  this->set_acceleration_covariance(dp.get_acceleration_covariance());
  return *this;
}

/**
 * @brief Fills and returns a sensor_msgs/Imu message.
 *
 * @return sensor_msgs/Imu message.
 */
sensor_msgs::msg::Imu DynamicPose::to_imu() const
{
  sensor_msgs::msg::Imu msg{};
  Eigen::Quaterniond q = this->get_attitude();
  Eigen::Vector3d angular_vel = this->get_angular_velocity();
  Eigen::Vector3d accel = this->get_acceleration();
  std::array<double, 36> pose_covariance = this->get_pose_covariance();
  std::array<double, 36> twist_covariance = this->get_twist_covariance();
  std::array<double, 36> accel_covariance = this->get_acceleration_covariance();
  msg.set__header(this->get_header());
  msg.orientation.set__w(q.w());
  msg.orientation.set__x(q.x());
  msg.orientation.set__y(q.y());
  msg.orientation.set__z(q.z());
  msg.angular_velocity.set__x(angular_vel.x());
  msg.angular_velocity.set__y(angular_vel.y());
  msg.angular_velocity.set__z(angular_vel.z());
  msg.linear_acceleration.set__x(accel.x());
  msg.linear_acceleration.set__y(accel.y());
  msg.linear_acceleration.set__z(accel.z());
  int msg_index = 0;
  for (int i = 3; i < 6; ++i) {
    for (int j = 3; j < 6; ++j) {
      msg.orientation_covariance[msg_index] = pose_covariance[i * 6 + j];
      msg.angular_velocity_covariance[msg_index] = twist_covariance[i * 6 + j];
      ++msg_index;
    }
  }
  msg_index = 0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      msg.linear_acceleration_covariance[msg_index] = accel_covariance[i * 6 + j];
      ++msg_index;
    }
  }
  return msg;
}

} // namespace pose_kit
