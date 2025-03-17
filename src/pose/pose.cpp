/**
 * Pose library implementation.
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

#include <pose_kit/pose.hpp>

namespace pose_kit
{

Pose::Pose(const Pose & p)
{
  this->set_position(p.position());
  this->set_attitude(p.attitude());
  this->set_pose_covariance(p.pose_covariance());
  this->set_header(p.header());
}

Pose::Pose(double x, double y, double z, const std_msgs::msg::Header & header)
{
  tf2::Vector3 p(x, y, z);
  tf2::Quaternion q = tf2::Quaternion::getIdentity();
  this->set_position(p);
  this->set_attitude(q);
  this->set_header(header);
}

Pose::Pose(const tf2::Quaternion & q, const std_msgs::msg::Header & header)
{
  tf2::Vector3 p(0.0, 0.0, 0.0);
  this->set_position(p);
  this->set_attitude(q);
  this->set_header(header);
}

Pose::Pose(
  const tf2::Vector3 & rpy,
  const std_msgs::msg::Header & header)
{
  tf2::Vector3 p(0.0, 0.0, 0.0);
  tf2::Quaternion q;
  q.setRPY(rpy.x(), rpy.y(), rpy.z());
  this->set_position(p);
  this->set_attitude(q);
  this->set_header(header);
}

Pose::Pose(
  double x, double y, double z, double heading,
  const std_msgs::msg::Header & header,
  const std::array<double, 36> & cov)
{
  tf2::Vector3 p(x, y, z);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, heading);
  this->set_position(p);
  this->set_attitude(q);
  this->set_pose_covariance(cov);
  this->set_header(header);
}

Pose::Pose(
  const tf2::Vector3 & p,
  const tf2::Quaternion & q,
  const std_msgs::msg::Header & header,
  const std::array<double, 36> & cov)
{
  this->set_position(p);
  this->set_attitude(q);
  this->set_pose_covariance(cov);
  this->set_header(header);
}

Pose::Pose(
  const Eigen::Vector3d & pos, const Eigen::Quaterniond & attitude,
  const std_msgs::msg::Header & header, const std::array<double, 36> & cov)
{
  tf2::Vector3 p(pos.x(), pos.y(), pos.z());
  tf2::Quaternion q(attitude.x(), attitude.y(), attitude.z(), attitude.w());
  Pose(p, q, header, cov);
}

Pose::Pose(
  const Eigen::Isometry3d & iso, const std_msgs::msg::Header & header,
  const std::array<double, 36> & cov)
{
  Eigen::Vector3d pos = iso.translation();
  Eigen::Quaterniond attitude(iso.rotation());
  Pose(pos, attitude, header, cov);
}

Pose::Pose(const geometry_msgs::msg::PoseStamped & msg)
{
  tf2::Vector3 p(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  tf2::Quaternion q(
    msg.pose.orientation.x,
    msg.pose.orientation.y,
    msg.pose.orientation.z,
    msg.pose.orientation.w);
  this->set_position(p);
  this->set_attitude(q);
  this->set_header(msg.header);
}

Pose::Pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  tf2::Vector3 p(
    msg.pose.pose.position.x,
    msg.pose.pose.position.y,
    msg.pose.pose.position.z);
  tf2::Quaternion q(
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w);
  this->set_position(p);
  this->set_attitude(q);
  this->set_pose_covariance(msg.pose.covariance);
  this->set_header(msg.header);
}

Pose::Pose(const geometry_msgs::msg::TransformStamped & msg)
{
  tf2::Vector3 p(
    msg.transform.translation.x,
    msg.transform.translation.y,
    msg.transform.translation.z);
  tf2::Quaternion q(
    msg.transform.rotation.x,
    msg.transform.rotation.y,
    msg.transform.rotation.z,
    msg.transform.rotation.w);
  this->set_position(p);
  this->set_attitude(q);
  this->set_header(msg.header);
}

Pose & Pose::operator=(const Pose & p)
{
  this->set_position(p.position());
  this->set_attitude(p.attitude());
  this->set_pose_covariance(p.pose_covariance());
  this->set_header(p.header());
  return *this;
}

Pose & Pose::operator=(Pose && p)
{
  this->set_position(p.position());
  this->set_attitude(p.attitude());
  this->set_pose_covariance(p.pose_covariance());
  this->set_header(p.header());
  return *this;
}

void Pose::to_pose_stamped(geometry_msgs::msg::PoseStamped & msg)
{
  msg.set__header(this->header());
  msg.pose.position.set__x(this->position().x());
  msg.pose.position.set__y(this->position().y());
  msg.pose.position.set__z(this->position().z());
  msg.pose.orientation.set__x(this->attitude().x());
  msg.pose.orientation.set__y(this->attitude().y());
  msg.pose.orientation.set__z(this->attitude().z());
  msg.pose.orientation.set__w(this->attitude().w());
}

void Pose::to_pose_with_covariance_stamped(geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  msg.set__header(this->header());
  msg.pose.pose.position.set__x(this->position().x());
  msg.pose.pose.position.set__y(this->position().y());
  msg.pose.pose.position.set__z(this->position().z());
  msg.pose.pose.orientation.set__x(this->attitude().x());
  msg.pose.pose.orientation.set__y(this->attitude().y());
  msg.pose.pose.orientation.set__z(this->attitude().z());
  msg.pose.pose.orientation.set__w(this->attitude().w());
  msg.pose.set__covariance(this->pose_covariance());
}

void Pose::rigid_transform(
  const geometry_msgs::msg::TransformStamped & tf,
  const std::string & new_frame_id)
{
  // Get isometries and tf representations
  Eigen::Isometry3d pose;
  this->get_isometry(pose);
  Eigen::Isometry3d iso_from_to = tf2::transformToEigen(tf.transform);
  Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
  R.block<3, 3>(0, 0) = iso_from_to.rotation();
  R.block<3, 3>(3, 3) = iso_from_to.rotation();

  // Remap covariance arrays, copy the input to preserve it
  std::array<double, 36> cov(this->pose_covariance());
  Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> cov_in(cov.data());
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> cov_out(pose_covariance_.data());

  // Apply the transformation
  // TODO Covariance rotation needs a theoretical check
  Eigen::Isometry3d new_pose = iso_from_to * pose * iso_from_to.inverse();
  cov_out = R * cov_in * R.transpose();

  // Write back the new pose
  tf2::Vector3 new_p(
    new_pose.translation().x(),
    new_pose.translation().y(),
    new_pose.translation().z());
  Eigen::Quaterniond eigen_q(new_pose.rotation());
  tf2::Quaternion new_q(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w());
  this->set_position(new_p);
  this->set_attitude(new_q);

  // Change the frame_id
  if (!new_frame_id.empty()) {
    this->set_frame_id(new_frame_id);
  }
}

} // namespace pose_kit
