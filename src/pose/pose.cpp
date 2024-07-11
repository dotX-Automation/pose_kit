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

#include <tf2_eigen/tf2_eigen.hpp>

#include <pose_kit/pose.hpp>

namespace pose_kit
{

/**
 * @brief Default constructor.
 */
Pose::Pose()
{}

/**
 * @brief Copy constructor.
 *
 * @param p Pose to copy.
 */
Pose::Pose(const Pose & p)
{
  this->set_position(p.get_position());
  this->set_attitude(p.get_attitude());
  this->set_pose_covariance(p.get_pose_covariance());
  this->set_header(p.get_header());
}

/**
 * @brief Constructor with initial position.
 *
 * @param x Initial X position [m].
 * @param y Initial Y position [m].
 * @param z Initial Z position [m].
 * @param header Pose header.
 */
Pose::Pose(double x, double y, double z, const std_msgs::msg::Header & header)
{
  this->set_position(Eigen::Vector3d(x, y, z));
  this->set_attitude(Eigen::Quaterniond::Identity());
  this->set_header(header);
}

/**
 * @brief Constructor with initial attitude.
 *
 * @param q Initial attitude quaternion.
 * @param header Pose header.
 */
Pose::Pose(const Eigen::Quaterniond & q, const std_msgs::msg::Header & header)
{
  this->set_position(Eigen::Vector3d(0.0, 0.0, 0.0));
  this->set_attitude(q);
  this->set_header(header);
}

/**
 * @brief Constructor with initial euler angles.
 *
 * @param rpy_angles Initial euler angles [rad] in [-PI +PI] (may also be an array).
 * @param header Pose header.
 */
Pose::Pose(const Eigen::EulerAnglesXYZd & rpy_angles, const std_msgs::msg::Header & header)
{
  this->set_position(Eigen::Vector3d(0.0, 0.0, 0.0));
  this->set_rpy(rpy_angles);
  this->set_header(header);
}

/**
 * @brief Constructor with initial position and heading.
 *
 * @param x Initial X position [m].
 * @param y Initial Y position [m].
 * @param z Initial Z position [m].
 * @param heading Initial heading [rad] in [-PI +PI].
 * @param header Pose header.
 * @param cov Initial pose covariance.
 */
Pose::Pose(
  double x, double y, double z, double heading,
  const std_msgs::msg::Header & header,
  const std::array<double, 36> & cov)
{
  this->set_position(Eigen::Vector3d(x, y, z));
  this->set_attitude(
    Eigen::Quaterniond(
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ())));
  this->set_pose_covariance(cov);
  this->set_header(header);
}

/**
 * @brief Constructor with initial position, attitude and euler angles.
 *
 * @param pos Initial position [m].
 * @param q Initial attitude quaternion.
 * @param header Pose header.
 * @param cov Initial pose covariance.
 */
Pose::Pose(
  const Eigen::Vector3d & pos,
  const Eigen::Quaterniond & q,
  const std_msgs::msg::Header & header,
  const std::array<double, 36> & cov)
{
  this->set_position(pos);
  this->set_attitude(q);
  this->set_pose_covariance(cov);
  this->set_header(header);
}

/**
 * @brief Constructor that builds from a PoseStamped ROS message.
 *
 * @param msg ROS message to build from.
 */
Pose::Pose(const geometry_msgs::msg::PoseStamped & msg)
{
  this->set_position(
    Eigen::Vector3d(
      msg.pose.position.x,
      msg.pose.position.y,
      msg.pose.position.z));
  this->set_attitude(
    Eigen::Quaterniond(
      msg.pose.orientation.w,
      msg.pose.orientation.x,
      msg.pose.orientation.y,
      msg.pose.orientation.z));
  this->set_header(msg.header);
}

/**
 * @brief Constructor that builds from a PoseWithCovarianceStamped ROS message.
 *
 * @param msg ROS message to build from.
 */
Pose::Pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  this->set_position(
    Eigen::Vector3d(
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z));
  this->set_attitude(
    Eigen::Quaterniond(
      msg.pose.pose.orientation.w,
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z));
  this->set_pose_covariance(msg.pose.covariance);
  this->set_header(msg.header);
}

/**
 * @brief Copy assignment operator.
 *
 * @param p Pose to copy.
 */
Pose & Pose::operator=(const Pose & p)
{
  this->set_position(p.get_position());
  this->set_attitude(p.get_attitude());
  this->set_pose_covariance(p.get_pose_covariance());
  this->set_header(p.get_header());
  return *this;
}

/**
 * @brief Move assignment operator.
 *
 * @param p Pose to move.
 */
Pose & Pose::operator=(Pose && p)
{
  this->set_position(p.get_position());
  this->set_attitude(p.get_attitude());
  this->set_rpy(p.get_rpy());
  this->set_pose_covariance(p.get_pose_covariance());
  this->set_header(p.get_header());
  return *this;
}

/**
 * @brief Default destructor.
 */
Pose::~Pose()
{}

/**
 * @brief Converts to a PoseStamped ROS message.
 *
 * @return PoseStamped ROS message.
 */
geometry_msgs::msg::PoseStamped Pose::to_pose_stamped()
{
  geometry_msgs::msg::PoseStamped msg{};
  Eigen::Vector3d position = this->get_position();
  Eigen::Quaterniond attitude = this->get_attitude();
  msg.set__header(this->get_header());
  msg.pose.position.set__x(position(0));
  msg.pose.position.set__y(position(1));
  msg.pose.position.set__z(position(2));
  msg.pose.orientation.set__w(attitude.w());
  msg.pose.orientation.set__x(attitude.x());
  msg.pose.orientation.set__y(attitude.y());
  msg.pose.orientation.set__z(attitude.z());
  return msg;
}

/**
 * @brief Converts to a PoseWithCovarianceStamped ROS message.
 *
 * @return PoseWithCovarianceStamped ROS message.
 */
geometry_msgs::msg::PoseWithCovarianceStamped Pose::to_pose_with_covariance_stamped()
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg{};
  Eigen::Vector3d position = this->get_position();
  Eigen::Quaterniond attitude = this->get_attitude();
  msg.set__header(this->get_header());
  msg.pose.pose.position.set__x(position(0));
  msg.pose.pose.position.set__y(position(1));
  msg.pose.pose.position.set__z(position(2));
  msg.pose.pose.orientation.set__w(attitude.w());
  msg.pose.pose.orientation.set__x(attitude.x());
  msg.pose.pose.orientation.set__y(attitude.y());
  msg.pose.pose.orientation.set__z(attitude.z());
  msg.pose.set__covariance(this->get_pose_covariance());
  return msg;
}

/**
 * @brief Applies a rigid transformation to the pose.
 *
 * @param tf ROS transformation to apply.
 * @param new_frame_id New frame ID to set (optional).
 */
void Pose::rigid_transform(
  const geometry_msgs::msg::TransformStamped & tf,
  const std::string & new_frame_id)
{
  // Get isometries and tf representations
  Eigen::Isometry3d pose = this->get_isometry();
  Eigen::Isometry3d iso_from_to = tf2::transformToEigen(tf.transform);
  Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
  R.block<3, 3>(0, 0) = iso_from_to.rotation();
  R.block<3, 3>(3, 3) = iso_from_to.rotation();

  // Remap covariance arrays, copy the input to preserve it
  std::array<double, 36> cov = this->get_pose_covariance();
  Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> cov_in(cov.data());
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> cov_out(pose_covariance_.data());

  // Apply the transformation
  Eigen::Isometry3d new_pose = iso_from_to * pose * iso_from_to.inverse();
  cov_out = R * cov_in * R.transpose();
  this->set_position(new_pose.translation());
  this->set_attitude(Eigen::Quaterniond(new_pose.rotation()));

  // Change the frame_id
  if (!new_frame_id.empty()) {
    this->set_frame_id(new_frame_id);
  }
}

} // namespace pose_kit
