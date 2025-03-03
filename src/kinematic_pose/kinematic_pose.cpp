/**
 * Kinematic pose library implementation.
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

#include <tf2_eigen/tf2_eigen.hpp>

#include <pose_kit/kinematic_pose.hpp>

namespace pose_kit
{

KinematicPose::KinematicPose()
: Pose()
{}

KinematicPose::~KinematicPose()
{}

KinematicPose::KinematicPose(const KinematicPose & kp)
: Pose(dynamic_cast<const Pose &>(kp))
{
  this->set_velocity(kp.velocity());
  this->set_angular_velocity(kp.angular_velocity());
  this->set_twist_covariance(kp.twist_covariance());
}

KinematicPose::KinematicPose(
  double x, double y, double z,
  double vx, double vy, double vz,
  const std_msgs::msg::Header & header)
: Pose(x, y, z, header)
{
  tf2::Vector3 p(x, y, z);
  tf2::Vector3 v(vx, vy, vz);
  this->set_position(p);
  this->set_velocity(v);
}

KinematicPose::KinematicPose(
  const tf2::Quaternion & q,
  const tf2::Vector3 & angular_v,
  const std_msgs::msg::Header & header)
: Pose(q, header)
{
  tf2::Vector3 v(0.0, 0.0, 0.0);
  this->set_velocity(v);
  this->set_angular_velocity(angular_v);
}

KinematicPose::KinematicPose(
  const tf2::Vector3 & rpy,
  const tf2::Vector3 & angular_v,
  const std_msgs::msg::Header & header)
: Pose(rpy, header)
{
  tf2::Vector3 v(0.0, 0.0, 0.0);
  this->set_velocity(v);
  this->set_angular_velocity(angular_v);
}

KinematicPose::KinematicPose(
  double x, double y, double z,
  double vx, double vy, double vz,
  double heading,
  const std_msgs::msg::Header & header,
  const std::array<double, 36> & cov)
: Pose(x, y, z, heading, header, cov)
{
  tf2::Vector3 v(vx, vy, vz);
  tf2::Vector3 angular_v(0.0, 0.0, 0.0);
  this->set_velocity(v);
  this->set_angular_velocity(angular_v);
}

KinematicPose::KinematicPose(
  const tf2::Vector3 & p,
  const tf2::Quaternion & q,
  const tf2::Vector3 & v,
  const tf2::Vector3 & angular_v,
  const std_msgs::msg::Header & header,
  const std::array<double, 36> & cov,
  const std::array<double, 36> & twist_cov)
: Pose(p, q, header, cov)
{
  this->set_velocity(v);
  this->set_angular_velocity(angular_v);
  this->set_twist_covariance(twist_cov);
}

KinematicPose::KinematicPose(
  const geometry_msgs::msg::PoseStamped & pose_stamped,
  const geometry_msgs::msg::TwistStamped & twist_stamped,
  const std_msgs::msg::Header & header)
: Pose(pose_stamped)
{
  tf2::Vector3 v(
    twist_stamped.twist.linear.x,
    twist_stamped.twist.linear.y,
    twist_stamped.twist.linear.z);
  tf2::Vector3 in;

  this->set_velocity(v);
  this->set_angular_velocity(in);
  this->set_header(header);
}

KinematicPose::KinematicPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_with_cov_stamped,
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist_with_cov_stamped,
  const std_msgs::msg::Header & header)
: Pose(pose_with_cov_stamped)
{
  tf2::Vector3 v(
    twist_with_cov_stamped.twist.twist.linear.x,
    twist_with_cov_stamped.twist.twist.linear.y,
    twist_with_cov_stamped.twist.twist.linear.z);
  tf2::Vector3 angular_v(
    twist_with_cov_stamped.twist.twist.angular.x,
    twist_with_cov_stamped.twist.twist.angular.y,
    twist_with_cov_stamped.twist.twist.angular.z);
  this->set_velocity(v);
  this->set_angular_velocity(angular_v);
  this->set_twist_covariance(twist_with_cov_stamped.twist.covariance);
  this->set_header(header);
}

KinematicPose & KinematicPose::operator=(const KinematicPose & kp)
{
  this->set_position(kp.position());
  this->set_attitude(kp.attitude());
  this->set_header(kp.header());
  this->set_velocity(kp.velocity());
  this->set_angular_velocity(kp.angular_velocity());
  this->set_twist_covariance(kp.twist_covariance());
  return *this;
}

KinematicPose & KinematicPose::operator=(KinematicPose && kp)
{
  this->set_position(kp.position());
  this->set_attitude(kp.attitude());
  this->set_header(kp.header());
  this->set_velocity(kp.velocity());
  this->set_angular_velocity(kp.angular_velocity());
  this->set_twist_covariance(kp.twist_covariance());
  return *this;
}

void KinematicPose::to_twist_stamped(geometry_msgs::msg::TwistStamped & msg) const
{
  msg.set__header(this->header());
  msg.twist.linear.set__x(this->velocity().x());
  msg.twist.linear.set__y(this->velocity().y());
  msg.twist.linear.set__z(this->velocity().z());
  msg.twist.angular.set__x(this->angular_velocity().x());
  msg.twist.angular.set__y(this->angular_velocity().y());
  msg.twist.angular.set__z(this->angular_velocity().z());
}

void KinematicPose::to_twist_with_covariance_stamped(geometry_msgs::msg::TwistWithCovarianceStamped & msg)
const
{
  msg.set__header(this->header());
  msg.twist.twist.linear.set__x(this->velocity().x());
  msg.twist.twist.linear.set__y(this->velocity().y());
  msg.twist.twist.linear.set__z(this->velocity().z());
  msg.twist.twist.angular.set__x(this->angular_velocity().x());
  msg.twist.twist.angular.set__y(this->angular_velocity().y());
  msg.twist.twist.angular.set__z(this->angular_velocity().z());
  msg.twist.set__covariance(this->twist_covariance());
}

void KinematicPose::rigid_transform(
  const geometry_msgs::msg::TransformStamped & tf,
  const std::string & new_frame_id)
{
  // Transform the pose
  Pose::rigid_transform(tf, new_frame_id);

  // Build twist
  Eigen::Vector3d eigen_v(
    this->velocity().x(),
    this->velocity().y(),
    this->velocity().z());
  Eigen::Vector3d eigen_angular_v(
    this->angular_velocity().x(),
    this->angular_velocity().y(),
    this->angular_velocity().z());
  Eigen::Matrix<double, 6, 1> twist = Eigen::Matrix<double, 6, 1>::Zero();
  twist.block<3, 1>(0, 0) = eigen_v;
  twist.block<3, 1>(3, 0) = eigen_angular_v;

  // Get isometries and tf representations and build adjoint matrix
  Eigen::Isometry3d iso_from_to = tf2::transformToEigen(tf.transform);
  Eigen::Matrix<double, 6, 6> adjoint = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Vector3d p = iso_from_to.translation();
  Eigen::Matrix<double, 3, 3> p_hat = Eigen::Matrix<double, 3, 3>::Zero();
  p_hat << 0.0, -p.z(), p.y(),
    p.z(), 0.0, -p.x(),
    -p.y(), p.x(), 0.0;
  adjoint.block<3, 3>(0, 0) = iso_from_to.rotation();
  adjoint.block<3, 3>(0, 3) = p_hat * iso_from_to.rotation();
  adjoint.block<3, 3>(3, 3) = iso_from_to.rotation();

  // Remap covariance arrays, copy the input to preserve it
  std::array<double, 36> twist_cov(this->twist_covariance());
  Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> twist_cov_in(twist_cov.data());
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> twist_cov_out(twist_covariance_.data());

  // Apply the transformation to the twist and its covariance
  // TODO Covariance rotation and selected entries need a theoretical check
  Eigen::Matrix<double, 6, 1> twist_out = adjoint * twist;
  twist_cov_out = adjoint * twist_cov_in * adjoint.inverse();

  // Write back results
  tf2::Vector3 v(
    twist_out.block<3, 1>(0, 0)(0),
    twist_out.block<3, 1>(0, 0)(1),
    twist_out.block<3, 1>(0, 0)(2));
  tf2::Vector3 angular_v(
    twist_out.block<3, 1>(3, 0)(0),
    twist_out.block<3, 1>(3, 0)(1),
    twist_out.block<3, 1>(3, 0)(2));
  this->set_velocity(v);
  this->set_angular_velocity(angular_v);
}

} // namespace pose_kit
