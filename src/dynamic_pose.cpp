/**
 * Dynamic pose library implementation.
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

#include <pose_kit/dynamic_pose.hpp>

namespace pose_kit
{

// ========== Constructors and destructors ==========

// ---------- Canonical constructors ----------

DynamicPose::DynamicPose(
  const tf2::Vector3 & pos,
  const tf2::Quaternion & att,
  const tf2::Vector3 & lin_vel,
  const tf2::Vector3 & ang_vel,
  const tf2::Vector3 & lin_accel,
  const tf2::Vector3 & ang_accel,
  const std_msgs::msg::Header & header,
  const std::string & child_frame_id,
  const PoseCovariance & pose_cov,
  const TwistCovariance & twist_cov,
  const AccelCovariance & accel_cov)
: KinematicPose(pos, att, lin_vel, ang_vel, header, child_frame_id, pose_cov, twist_cov),
  lin_accel_(lin_accel),
  ang_accel_(ang_accel),
  accel_cov_(accel_cov)
{
}

DynamicPose::DynamicPose(
  const Eigen::Vector3d & pos,
  const Eigen::Quaterniond & att,
  const Eigen::Vector3d & lin_vel,
  const Eigen::Vector3d & ang_vel,
  const Eigen::Vector3d & lin_accel,
  const Eigen::Vector3d & ang_accel,
  const std_msgs::msg::Header & header,
  const std::string & child_frame_id,
  const PoseCovariance & pose_cov,
  const TwistCovariance & twist_cov,
  const AccelCovariance & accel_cov)
: KinematicPose(pos, att, lin_vel, ang_vel, header, child_frame_id, pose_cov, twist_cov),
  accel_cov_(accel_cov)
{
  set_linear_acceleration(lin_accel);
  set_angular_acceleration(ang_accel);
}

// ========== Main methods ==========

void DynamicPose::change_parent_frame(const Pose & pose)
{
  // Update pose
  KinematicPose::change_parent_frame(pose);

  // TODO: Update the covariance
}

void DynamicPose::change_child_frame_inverse(const Pose & pose)
{
  // Update pose
  KinematicPose::change_child_frame_inverse(pose);

  // TODO: Update the covariance
}

void DynamicPose::change_frames(const Pose & pose_pre, const Pose & pose_post)
{
  // Update pose
  KinematicPose::change_frames(pose_pre, pose_post);

  // TODO: Update the covariance
}

} // namespace pose_kit
