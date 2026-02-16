/**
 * Pose library implementation.
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

#include <pose_kit/pose.hpp>

namespace pose_kit
{

// ========== Constructors and destructors ==========

// ---------- Canonical constructors ----------

Pose::Pose(
  const tf2::Vector3 & pos,
  const tf2::Quaternion & att,
  const std_msgs::msg::Header & header,
  const std::string & child_frame_id,
  const PoseCovariance & pose_cov)
: pos_(pos),
  att_(att),
  header_(header),
  child_frame_id_(child_frame_id),
  pose_cov_(pose_cov)
{
}

Pose::Pose(
  const Eigen::Vector3d & pos,
  const Eigen::Quaterniond & att,
  const std_msgs::msg::Header & header,
  const std::string & child_frame_id,
  const PoseCovariance & pose_cov)
: header_(header),
  child_frame_id_(child_frame_id),
  pose_cov_(pose_cov)
{
  set_position(pos);
  set_attitude(att);
}

Pose::Pose(
  const tf2::Vector3 & pos,
  const tf2::Vector3 & rpy,
  const std_msgs::msg::Header & header,
  const std::string & child_frame_id,
  const PoseCovariance & pose_cov)
: pos_(pos),
  header_(header),
  child_frame_id_(child_frame_id),
  pose_cov_(pose_cov)
{
  set_rpy(rpy);
}

Pose::Pose(
  const Eigen::Vector3d & pos,
  const Eigen::Vector3d & rpy,
  const std_msgs::msg::Header & header,
  const std::string & child_frame_id,
  const PoseCovariance & pose_cov)
: header_(header),
  child_frame_id_(child_frame_id),
  pose_cov_(pose_cov)
{
  set_position(pos);
  set_rpy(rpy);
}

// ---------- ROS message constructors ----------

Pose::Pose(
  const geometry_msgs::msg::PoseStamped & pose_msg,
  const std::string & child_frame_id,
  const PoseCovariance & pose_cov)
: child_frame_id_(child_frame_id),
  pose_cov_(pose_cov)
{
  set_position(
    tf2::Vector3(
      pose_msg.pose.position.x,
      pose_msg.pose.position.y,
      pose_msg.pose.position.z));
  set_attitude(
    tf2::Quaternion(
      pose_msg.pose.orientation.x,
      pose_msg.pose.orientation.y,
      pose_msg.pose.orientation.z,
      pose_msg.pose.orientation.w));
  set_header(pose_msg.header);
}

Pose::Pose(
  const geometry_msgs::msg::PoseWithCovariance & pose_msg,
  const std_msgs::msg::Header & header,
  const std::string & child_frame_id)
: header_(header),
  child_frame_id_(child_frame_id)
{
  set_position(
    tf2::Vector3(
      pose_msg.pose.position.x,
      pose_msg.pose.position.y,
      pose_msg.pose.position.z));
  set_attitude(
    tf2::Quaternion(
      pose_msg.pose.orientation.x,
      pose_msg.pose.orientation.y,
      pose_msg.pose.orientation.z,
      pose_msg.pose.orientation.w));
  set_pose_covariance(pose_msg.covariance);
}

Pose::Pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_msg,
  const std::string & child_frame_id)
: child_frame_id_(child_frame_id)
{
  set_position(
    tf2::Vector3(
      pose_msg.pose.pose.position.x,
      pose_msg.pose.pose.position.y,
      pose_msg.pose.pose.position.z));
  set_attitude(
    tf2::Quaternion(
      pose_msg.pose.pose.orientation.x,
      pose_msg.pose.pose.orientation.y,
      pose_msg.pose.pose.orientation.z,
      pose_msg.pose.pose.orientation.w));
  set_header(pose_msg.header);
  set_pose_covariance(pose_msg.pose.covariance);
}

Pose::Pose(
  const geometry_msgs::msg::TransformStamped & tf_msg,
  const PoseCovariance & pose_cov)
: pose_cov_(pose_cov)
{
  set_position(
    tf2::Vector3(
      tf_msg.transform.translation.x,
      tf_msg.transform.translation.y,
      tf_msg.transform.translation.z));
  set_attitude(
    tf2::Quaternion(
      tf_msg.transform.rotation.x,
      tf_msg.transform.rotation.y,
      tf_msg.transform.rotation.z,
      tf_msg.transform.rotation.w));
  set_header(tf_msg.header);
  set_child_frame_id(tf_msg.child_frame_id);
}

// ========== ROS message converters ==========

void Pose::to_point(geometry_msgs::msg::Point & msg) const
{
  msg.set__x(position().x());
  msg.set__y(position().y());
  msg.set__z(position().z());
}

void Pose::to_quaternion(geometry_msgs::msg::Quaternion & msg) const
{
  msg.set__x(attitude().x());
  msg.set__y(attitude().y());
  msg.set__z(attitude().z());
  msg.set__w(attitude().w());
}

void Pose::to_pose(geometry_msgs::msg::Pose & msg) const
{
  to_point(msg.position);
  to_quaternion(msg.orientation);
}

void Pose::to_pose_stamped(geometry_msgs::msg::PoseStamped & msg) const
{
  msg.set__header(header());
  to_pose(msg.pose);
}

void Pose::to_pose_with_covariance(geometry_msgs::msg::PoseWithCovariance & msg) const
{
  to_pose(msg.pose);
  msg.set__covariance(pose_covariance());
}

void Pose::to_pose_with_covariance_stamped(
  geometry_msgs::msg::PoseWithCovarianceStamped & msg) const
{
  msg.set__header(header());
  to_pose_with_covariance(msg.pose);
}

void Pose::to_transform_stamped(geometry_msgs::msg::TransformStamped & msg) const
{
  msg.set__header(header());
  msg.set__child_frame_id(child_frame_id());
  msg.transform.translation.set__x(position().x());
  msg.transform.translation.set__y(position().y());
  msg.transform.translation.set__z(position().z());
  msg.transform.rotation.set__x(attitude().x());
  msg.transform.rotation.set__y(attitude().y());
  msg.transform.rotation.set__z(attitude().z());
  msg.transform.rotation.set__w(attitude().w());
}

// ========== Main methods ==========

Pose Pose::inverse() const
{
  // Get the current isometry
  Eigen::Isometry3d T_parent_child;
  get_isometry(T_parent_child);
  // Invert the isometry
  Eigen::Isometry3d T_child_parent = T_parent_child.inverse();

  // Invert the header and child frame id
  std_msgs::msg::Header inv_header = header_;
  inv_header.set__frame_id(child_frame_id_);
  const std::string & inv_child_frame_id = header_.frame_id;

  // Invert the covariance
  const Matrix6d adj_inv = dua_math::adjoint(T_child_parent);
  PoseCovariance inv_pose_cov;
  Eigen::Map<Matrix6d> inv_cov(inv_pose_cov.data());
  Eigen::Map<const Matrix6d> curr_cov(pose_cov_.data());
  inv_cov = adj_inv * curr_cov * adj_inv.transpose();

  return Pose::from_isometry(
    T_child_parent,
    inv_header,
    inv_child_frame_id,
    inv_pose_cov);
}

void Pose::change_parent_frame(const Pose & pose)
{
  // Check frame consistency
  const std::string parent = parent_frame_id();
  const std::string & source = pose.child_frame_id();
  if (parent != source) {
    throw std::runtime_error("pose_kit: frame mismatch: got '" + source +
      "' expected '" + parent + "'");
  }

  // Get the current isometry
  Eigen::Isometry3d T_parent_child;
  get_isometry(T_parent_child);

  // Get the transform isometry
  Eigen::Isometry3d T_target_source;
  pose.get_isometry(T_target_source);

  // Update the pose: T_target_child = T_target_source * T_parent_child
  set_isometry(T_target_source * T_parent_child);

  // TODO: Update the covariance

  // Update the parent frame id
  set_parent_frame_id(pose.parent_frame_id());
}

void Pose::change_child_frame_inverse(const Pose & pose)
{
  // Check frame consistency
  const std::string child = child_frame_id();
  const std::string & source = pose.child_frame_id();
  if (child != source) {
    throw std::runtime_error("pose_kit: frame mismatch: got '" + source +
      "' expected '" + child + "'");
  }

  // Get the current isometry
  Eigen::Isometry3d T_parent_child;
  get_isometry(T_parent_child);

  // Get the transform isometry
  Eigen::Isometry3d T_target_source;
  pose.get_isometry(T_target_source);
  const Eigen::Isometry3d T_source_target = T_target_source.inverse();

  // Update the pose: T_parent_target = T_parent_child * T_source_target
  set_isometry(T_parent_child * T_source_target);

  // TODO: Update the covariance

  // Update the child frame id
  set_child_frame_id(pose.parent_frame_id());
}

void Pose::change_frames(const Pose & pose_pre, const Pose & pose_post)
{
  // Check frame consistency
  const std::string parent = parent_frame_id();
  const std::string & source_pre = pose_pre.child_frame_id();
  if (parent != source_pre) {
    throw std::runtime_error("pose_kit: pre frame mismatch: got '" + source_pre +
      "' expected '" + parent + "'");
  }

  const std::string child = child_frame_id();
  const std::string & source_post = pose_post.child_frame_id();
  if (child != source_post) {
    throw std::runtime_error("pose_kit: post frame mismatch: got '" + source_post +
      "' expected '" + child + "'");
  }

  // Get the current isometry
  Eigen::Isometry3d T_parent_child;
  get_isometry(T_parent_child);

  // Get the transform isometries
  Eigen::Isometry3d T_pre_target_source;
  pose_pre.get_isometry(T_pre_target_source);
  Eigen::Isometry3d T_post_target_source;
  pose_post.get_isometry(T_post_target_source);
  const Eigen::Isometry3d T_post_source_target = T_post_target_source.inverse();

  // Update the pose: T_target_child = T_pre_target_source * T_parent_child * T_post_source_target
  set_isometry(T_pre_target_source * T_parent_child * T_post_source_target);

  // TODO: Update the covariance

  // Update the frame ids
  set_parent_frame_id(pose_pre.parent_frame_id());
  set_child_frame_id(pose_post.parent_frame_id());
}

} // namespace pose_kit
