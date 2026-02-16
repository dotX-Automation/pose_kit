/**
 * Kinematic pose library implementation.
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

#include <pose_kit/kinematic_pose.hpp>

namespace pose_kit
{

// ========== Constructors and destructors ==========

// ---------- Canonical constructors ----------

KinematicPose::KinematicPose(
  const tf2::Vector3 & pos,
  const tf2::Quaternion & att,
  const tf2::Vector3 & lin_vel,
  const tf2::Vector3 & ang_vel,
  const std_msgs::msg::Header & header,
  const std::string & child_frame_id,
  const PoseCovariance & pose_cov,
  const TwistCovariance & twist_cov)
: Pose(pos, att, header, child_frame_id, pose_cov),
  lin_vel_(lin_vel),
  ang_vel_(ang_vel),
  twist_cov_(twist_cov)
{
}

KinematicPose::KinematicPose(
  const Eigen::Vector3d & pos,
  const Eigen::Quaterniond & att,
  const Eigen::Vector3d & lin_vel,
  const Eigen::Vector3d & ang_vel,
  const std_msgs::msg::Header & header,
  const std::string & child_frame_id,
  const PoseCovariance & pose_cov,
  const TwistCovariance & twist_cov)
: Pose(pos, att, header, child_frame_id, pose_cov),
  twist_cov_(twist_cov)
{
  set_linear_velocity(lin_vel);
  set_angular_velocity(ang_vel);
}

// ---------- ROS message constructors ----------

KinematicPose::KinematicPose(
  const geometry_msgs::msg::PoseStamped & pose_msg,
  const geometry_msgs::msg::TwistStamped & twist_msg,
  const std::string & child_frame_id,
  const PoseCovariance & pose_cov,
  const TwistCovariance & twist_cov)
: Pose(pose_msg, child_frame_id, pose_cov),
  twist_cov_(twist_cov)
{
  set_linear_velocity(
    tf2::Vector3(
      twist_msg.twist.linear.x,
      twist_msg.twist.linear.y,
      twist_msg.twist.linear.z));
  set_angular_velocity(
    tf2::Vector3(
      twist_msg.twist.angular.x,
      twist_msg.twist.angular.y,
      twist_msg.twist.angular.z));
}

KinematicPose::KinematicPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_msg,
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist_msg,
  const std::string & child_frame_id)
: Pose(pose_msg, child_frame_id),
  twist_cov_(twist_msg.twist.covariance)
{
  set_linear_velocity(
    tf2::Vector3(
      twist_msg.twist.twist.linear.x,
      twist_msg.twist.twist.linear.y,
      twist_msg.twist.twist.linear.z));
  set_angular_velocity(
    tf2::Vector3(
      twist_msg.twist.twist.angular.x,
      twist_msg.twist.twist.angular.y,
      twist_msg.twist.twist.angular.z));
}

// ---------- ROS messages ----------

void KinematicPose::to_twist(geometry_msgs::msg::Twist & msg) const
{
  msg.linear.set__x(lin_vel_.x());
  msg.linear.set__y(lin_vel_.y());
  msg.linear.set__z(lin_vel_.z());
  msg.angular.set__x(ang_vel_.x());
  msg.angular.set__y(ang_vel_.y());
  msg.angular.set__z(ang_vel_.z());
}

void KinematicPose::to_twist_stamped(geometry_msgs::msg::TwistStamped & msg) const
{
  msg.set__header(this->header());
  msg.twist.linear.set__x(lin_vel_.x());
  msg.twist.linear.set__y(lin_vel_.y());
  msg.twist.linear.set__z(lin_vel_.z());
  msg.twist.angular.set__x(ang_vel_.x());
  msg.twist.angular.set__y(ang_vel_.y());
  msg.twist.angular.set__z(ang_vel_.z());
}

void KinematicPose::to_twist_with_covariance(geometry_msgs::msg::TwistWithCovariance & msg) const
{
  msg.twist.linear.set__x(lin_vel_.x());
  msg.twist.linear.set__y(lin_vel_.y());
  msg.twist.linear.set__z(lin_vel_.z());
  msg.twist.angular.set__x(ang_vel_.x());
  msg.twist.angular.set__y(ang_vel_.y());
  msg.twist.angular.set__z(ang_vel_.z());
  msg.set__covariance(twist_cov_);
}

void KinematicPose::to_twist_with_covariance_stamped(
  geometry_msgs::msg::TwistWithCovarianceStamped & msg) const
{
  msg.set__header(this->header());
  msg.twist.twist.linear.set__x(lin_vel_.x());
  msg.twist.twist.linear.set__y(lin_vel_.y());
  msg.twist.twist.linear.set__z(lin_vel_.z());
  msg.twist.twist.angular.set__x(ang_vel_.x());
  msg.twist.twist.angular.set__y(ang_vel_.y());
  msg.twist.twist.angular.set__z(ang_vel_.z());
  msg.twist.set__covariance(twist_cov_);
}

// ---------- Main methods ----------

void KinematicPose::apply_pre_transform(const geometry_msgs::msg::TransformStamped & tf)
{
  // Update pose
  Pose::apply_pre_transform(tf);

  // Get the transform isometry
  const Eigen::Isometry3d T_target_source = tf2::transformToEigen(tf.transform);

  // Transform the twist
  Eigen::Matrix<double, 6, 1> twist;
  twist << lin_vel_.x(), lin_vel_.y(), lin_vel_.z(),
    ang_vel_.x(), ang_vel_.y(), ang_vel_.z();
  const Matrix6d adj = dua_math::adjoint(T_target_source);
  twist = adj * twist;
  set_linear_velocity(tf2::Vector3(twist(0), twist(1), twist(2)));
  set_angular_velocity(tf2::Vector3(twist(3), twist(4), twist(5)));

  // Update the covariance
  Eigen::Map<Matrix6d> twist_cov(twist_cov_.data());
  twist_cov = adj * twist_cov * adj.transpose();
}

void KinematicPose::apply_post_inverse_transform(const geometry_msgs::msg::TransformStamped & tf)
{
  // Update pose
  Pose::apply_post_inverse_transform(tf);

  // The twist and its covariance remain unchanged when changing the child frame
}

void KinematicPose::apply_transform_chain(
  const geometry_msgs::msg::TransformStamped & tf_pre,
  const geometry_msgs::msg::TransformStamped & tf_post)
{
  // Update pose
  Pose::apply_transform_chain(tf_pre, tf_post);

  // Get the pre-transform isometry
  const Eigen::Isometry3d Tpre_target_source = tf2::transformToEigen(tf_pre.transform);

  // Transform the twist (same as pre-transform only)
  Eigen::Matrix<double, 6, 1> twist;
  twist << lin_vel_.x(), lin_vel_.y(), lin_vel_.z(),
    ang_vel_.x(), ang_vel_.y(), ang_vel_.z();
  const Matrix6d adj_pre = dua_math::adjoint(Tpre_target_source);
  twist = adj_pre * twist;
  set_linear_velocity(tf2::Vector3(twist(0), twist(1), twist(2)));
  set_angular_velocity(tf2::Vector3(twist(3), twist(4), twist(5)));

  // Update the covariance
  Eigen::Map<Matrix6d> twist_cov(twist_cov_.data());
  twist_cov = adj_pre * twist_cov * adj_pre.transpose();
}

} // namespace pose_kit
