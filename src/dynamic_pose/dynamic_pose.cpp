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

DynamicPose::DynamicPose()
: KinematicPose()
{}

DynamicPose::~DynamicPose()
{}

DynamicPose::DynamicPose(const DynamicPose & dp)
: KinematicPose(dynamic_cast<const KinematicPose &>(dp))
{
  this->set_acceleration(dp.acceleration());
  this->set_angular_acceleration(dp.angular_acceleration());
}

DynamicPose::DynamicPose(
  double x, double y, double z,
  double vx, double vy, double vz,
  double ax, double ay, double az,
  const std_msgs::msg::Header & header)
: KinematicPose(x, y, z, vx, vy, vz, header)
{
  tf2::Vector3 a(ax, ay, az);
  tf2::Vector3 angular_a(0.0, 0.0, 0.0);
  this->set_acceleration(a);
  this->set_angular_acceleration(angular_a);
}

DynamicPose::DynamicPose(
  const tf2::Quaternion & q,
  const tf2::Vector3 & angular_v,
  const tf2::Vector3 & angular_a,
  const std_msgs::msg::Header & header)
: KinematicPose(q, angular_v, header)
{
  tf2::Vector3 a(0.0, 0.0, 0.0);
  this->set_acceleration(a);
  this->set_angular_acceleration(angular_a);
}

DynamicPose::DynamicPose(
  const tf2::Vector3 & rpy,
  const tf2::Vector3 & angular_v,
  const tf2::Vector3 & angular_a,
  const std_msgs::msg::Header & header)
: KinematicPose(rpy, angular_v, header)
{
  tf2::Vector3 a(0.0, 0.0, 0.0);
  this->set_acceleration(a);
  this->set_angular_acceleration(angular_a);
}

DynamicPose::DynamicPose(
  double x, double y, double z,
  double vx, double vy, double vz,
  double ax, double ay, double az,
  double heading,
  const std_msgs::msg::Header & header,
  const std::array<double, 36> & cov)
: KinematicPose(x, y, z, vx, vy, vz, heading, header, cov)
{
  tf2::Vector3 a(ax, ay, az);
  tf2::Vector3 angular_a(0.0, 0.0, 0.0);
  this->set_acceleration(a);
  this->set_angular_acceleration(angular_a);
}

DynamicPose::DynamicPose(
  const tf2::Vector3 & p,
  const tf2::Quaternion & q,
  const tf2::Vector3 & v,
  const tf2::Vector3 & angular_v,
  const tf2::Vector3 & a,
  const tf2::Vector3 & angular_a,
  const std_msgs::msg::Header & header,
  const std::array<double, 36> & cov,
  const std::array<double, 36> & twist_cov,
  const std::array<double, 36> & accel_cov)
: KinematicPose(p, q, v, angular_v, header, cov, twist_cov)
{
  this->set_acceleration(a);
  this->set_angular_acceleration(angular_a);
  this->set_acceleration_covariance(accel_cov);
}

DynamicPose & DynamicPose::operator=(const DynamicPose & dp)
{
  this->set_position(dp.position());
  this->set_attitude(dp.attitude());
  this->set_header(dp.header());
  this->set_velocity(dp.velocity());
  this->set_angular_velocity(dp.angular_velocity());
  this->set_acceleration(dp.acceleration());
  this->set_angular_acceleration(dp.angular_acceleration());
  this->set_acceleration_covariance(dp.acceleration_covariance());
  return *this;
}

DynamicPose & DynamicPose::operator=(DynamicPose && dp)
{
  this->set_position(dp.position());
  this->set_attitude(dp.attitude());
  this->set_header(dp.header());
  this->set_velocity(dp.velocity());
  this->set_angular_velocity(dp.angular_velocity());
  this->set_acceleration(dp.acceleration());
  this->set_angular_acceleration(dp.angular_acceleration());
  this->set_acceleration_covariance(dp.acceleration_covariance());
  return *this;
}

void DynamicPose::to_imu(sensor_msgs::msg::Imu & msg) const
{
  tf2::Quaternion q = this->attitude();
  tf2::Vector3 angular_v = this->angular_velocity();
  tf2::Vector3 a = this->acceleration();
  std::array<double, 36> pose_covariance(this->pose_covariance());
  std::array<double, 36> twist_covariance(this->twist_covariance());
  std::array<double, 36> accel_covariance(this->acceleration_covariance());
  msg.set__header(this->header());
  msg.orientation.set__x(q.x());
  msg.orientation.set__y(q.y());
  msg.orientation.set__z(q.z());
  msg.orientation.set__w(q.w());
  msg.angular_velocity.set__x(angular_v.x());
  msg.angular_velocity.set__y(angular_v.y());
  msg.angular_velocity.set__z(angular_v.z());
  msg.linear_acceleration.set__x(a.x());
  msg.linear_acceleration.set__y(a.y());
  msg.linear_acceleration.set__z(a.z());
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
}

} // namespace pose_kit
