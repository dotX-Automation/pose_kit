/**
 * Pose library, based on tf2 geometry types.
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

#pragma once

#include "visibility_control.h"

#include <array>
#include <memory>
#include <stdexcept>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/utils.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>

namespace pose_kit
{

/**
 * Represents position and orientation of an autonomous agent.
 */
class POSE_KIT_PUBLIC Pose
{
public:
  using SharedPtr = std::shared_ptr<Pose>;
  using WeakPtr = std::weak_ptr<Pose>;
  using UniquePtr = std::unique_ptr<Pose>;
  using ConstSharedPtr = std::shared_ptr<const Pose>;
  using ConstWeakPtr = std::weak_ptr<const Pose>;

  /**
   * @brief Default constructor.
   */
  Pose() = default;

  /**
   * @brief Copy constructor.
   *
   * @param p Pose to copy.
   */
  Pose(const Pose & p);

  /**
   * @brief Constructor with initial position.
   *
   * @param x Initial X position [m].
   * @param y Initial Y position [m].
   * @param z Initial Z position [m].
   * @param header Pose header.
   */
  Pose(
    double x, double y, double z,
    const std_msgs::msg::Header & header = std_msgs::msg::Header());

  /**
   * @brief Constructor with initial attitude quaternion.
   *
   * @param q Initial attitude quaternion.
   * @param header Pose header.
   */
  Pose(
    const tf2::Quaternion & q,
    const std_msgs::msg::Header & header = std_msgs::msg::Header());

  /**
   * @brief Constructor with initial attitude expressed as euler angles.
   * Conventions are the classic ZYX (yaw-pitch-roll).
   * Ref.: tf2/LinearMath/Matrix3x3.h
   *
   * @param rpy Initial roll, pitch, and yaw angles [rad].
   * @param header Pose header.
   */
  Pose(
    const tf2::Vector3 & rpy,
    const std_msgs::msg::Header & header = std_msgs::msg::Header());

  /**
   * @brief Constructor with initial position and heading.
   *
   * @param x Initial X position [m].
   * @param y Initial Y position [m].
   * @param z Initial Z position [m].
   * @param heading Initial heading (i.e., yaw) [rad] in [-PI +PI].
   * @param header Pose header.
   * @param cov Initial pose covariance.
   */
  Pose(
    double x, double y, double z, double heading,
    const std_msgs::msg::Header & header = std_msgs::msg::Header(),
    const std::array<double, 36> & cov = std::array<double, 36>{});

  /**
   * @brief Constructor with initial position and attitude.
   *
   * @param p Initial position [m].
   * @param q Initial attitude quaternion.
   * @param header Pose header.
   * @param cov Initial pose covariance.
   */
  Pose(
    const tf2::Vector3 & p,
    const tf2::Quaternion & q,
    const std_msgs::msg::Header & header = std_msgs::msg::Header(),
    const std::array<double, 36> & cov = std::array<double, 36>{});

  /**
   * @brief Constructor with initial position and attitude in Eigen format.
   *
   * @param pos Initial position [m].
   * @param attitude Initial attitude quaternion.
   * @param header Pose header.
   * @param cov Initial pose covariance.
   */
  Pose(
    const Eigen::Vector3d & pos,
    const Eigen::Quaterniond & attitude,
    const std_msgs::msg::Header & header = std_msgs::msg::Header(),
    const std::array<double, 36> & cov = std::array<double, 36>{});

  /**
   * @brief Constructor that builds from an Eigen isometry.
   *
   * @param iso Eigen isometry to build from.
   * @param header Pose header.
   * @param cov Initial pose covariance.
   */
  Pose(
    const Eigen::Isometry3d & iso,
    const std_msgs::msg::Header & header = std_msgs::msg::Header(),
    const std::array<double, 36> & cov = std::array<double, 36>{});

  /**
   * @brief Constructor that builds from a PoseStamped ROS message.
   *
   * @param msg ROS message to build from.
   */
  Pose(const geometry_msgs::msg::PoseStamped & msg);

  /**
   * @brief Constructor that builds from a PoseWithCovarianceStamped ROS message.
   *
   * @param msg ROS message to build from.
   */
  Pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);

  /**
   * @brief Constructor that builds from a TransformStamped ROS message.
   *
   * @param msg ROS message to build from.
   */
  Pose(const geometry_msgs::msg::TransformStamped & msg);

  /**
   * @brief Default destructor.
   */
  virtual ~Pose() = default;

  /**
   * @brief Converts to a PoseStamped ROS message.
   *
   * @param msg PoseStamped ROS message to fill.
   */
  void to_pose_stamped(geometry_msgs::msg::PoseStamped & msg) const;

  /**
   * @brief Converts to a PoseWithCovarianceStamped ROS message.
   *
   * @param msg PoseWithCovarianceStamped ROS message to fill.
   */
  void to_pose_with_covariance_stamped(geometry_msgs::msg::PoseWithCovarianceStamped & msg) const;

  /**
   * @brief Gets the position of the rigid body in Eigen format.
   *
   * @param p Eigen vector to store the position.
   */
  inline void get_position(Eigen::Vector3d & p) const
  {
    p.x() = position_.x();
    p.y() = position_.y();
    p.z() = position_.z();
  }

  /**
   * @brief Gets the attitude of the rigid body in Eigen format.
   *
   * @param q Eigen quaternion to store the attitude.
   */
  inline void get_attitude(Eigen::Quaterniond & q) const
  {
    q.w() = attitude_.w();
    q.x() = attitude_.x();
    q.y() = attitude_.y();
    q.z() = attitude_.z();
  }

  /**
   * @brief Gets the attitude of the rigid body in Euler angles.
   * Applies the same conventions as tf2::getEulerYPR.
   *
   * @param rpy Eigen vector to store the roll, pitch, and yaw angles.
   */
  inline void get_rpy(Eigen::Vector3d & rpy) const
  {
    double r, p, y;
    tf2::getEulerYPR(attitude_, y, p, r);
    rpy.x() = r;
    rpy.y() = p;
    rpy.z() = y;
  }

  /**
   * @brief Gets the heading of the rigid body.
   *
   * @return double Heading angle [rad].
   */
  inline double get_heading() const
  {
    double r, p, y;
    tf2::getEulerYPR(attitude_, y, p, r);
    return y;
  }

  /**
   * @brief Gets the isometry corresponding to the pose of the rigid __feof_unlocked_body.
   *
   * @param iso Eigen isometry to store the pose.
   */
  inline void get_isometry(Eigen::Isometry3d & iso) const
  {
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    this->get_attitude(q);
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    this->get_position(p);
    iso = Eigen::Isometry3d::Identity();
    iso.rotate(q);
    iso.pretranslate(p);
  }

  [[nodiscard]] inline const tf2::Vector3 & position() const
  {
    return position_;
  }
  [[nodiscard]] inline const tf2::Quaternion & attitude() const
  {
    return attitude_;
  }
  [[nodiscard]] inline const std::array<double, 36> & pose_covariance() const
  {
    return pose_covariance_;
  }
  [[nodiscard]] inline const std_msgs::msg::Header & header() const
  {
    return header_;
  }
  inline uint64_t get_timestamp_ns() const
  {
    return header_.stamp.sec * 1e9 + header_.stamp.nanosec;
  }
  inline uint64_t get_timestamp_us() const
  {
    return header_.stamp.sec * 1e6 + header_.stamp.nanosec / 1e3;
  }
  inline uint64_t get_timestamp_ms() const
  {
    return header_.stamp.sec * 1e3 + header_.stamp.nanosec / 1e6;
  }
  inline uint64_t get_timestamp_s() const
  {
    return header_.stamp.sec;
  }
  inline void get_frame_id(std::string & id) const
  {
    id = header_.frame_id;
  }

  inline void set_position(const tf2::Vector3 & pos)
  {
    position_ = pos;
  }
  inline void set_position(const Eigen::Vector3d & pos)
  {
    position_.setX(pos.x());
    position_.setY(pos.y());
    position_.setZ(pos.z());
  }
  inline void set_attitude(const tf2::Quaternion & q)
  {
    attitude_ = q;
  }
  inline void set_attitude(const Eigen::Quaterniond & q)
  {
    attitude_.setX(q.x());
    attitude_.setY(q.y());
    attitude_.setZ(q.z());
    attitude_.setW(q.w());
  }
  inline void set_pose_covariance(const std::array<double, 36> & cov)
  {
    pose_covariance_ = cov;
  }
  inline void set_header(const std_msgs::msg::Header & header)
  {
    header_ = header;
  }
  inline void set_timestamp(const rclcpp::Time & time_point)
  {
    header_.set__stamp(time_point);
  }
  inline void set_timestamp_ns(uint64_t timestamp_ns)
  {
    header_.stamp.set__sec(timestamp_ns / 1e9);
    header_.stamp.set__nanosec(timestamp_ns % static_cast<uint64_t>(1e9));
  }
  inline void set_timestamp_us(uint64_t timestamp_us)
  {
    header_.stamp.set__sec(timestamp_us / 1e6);
    header_.stamp.set__nanosec((timestamp_us % static_cast<uint64_t>(1e6)) * 1e3);
  }
  inline void set_timestamp_ms(uint64_t timestamp_ms)
  {
    header_.stamp.set__sec(timestamp_ms / 1e3);
    header_.stamp.set__nanosec((timestamp_ms % static_cast<uint64_t>(1e3)) * 1e6);
  }
  inline void set_timestamp_s(uint64_t timestamp_s)
  {
    header_.stamp.set__sec(timestamp_s);
    header_.stamp.set__nanosec(0);
  }
  inline void set_frame_id(const std::string & frame_id)
  {
    header_.set__frame_id(frame_id);
  }

  /**
   * @brief Applies a rigid transformation to the pose.
   *
   * @param tf ROS transformation to apply.
   * @param new_frame_id New frame ID to set (optional).
   */
  virtual void rigid_transform(
    const geometry_msgs::msg::TransformStamped & tf,
    const std::string & new_frame_id = "");

  /**
   * @brief Copy assignment operator.
   *
   * @param p Pose to copy.
   */
  Pose & operator=(const Pose & p);

  /**
   * @brief Move assignment operator.
   *
   * @param p Pose to move.
   */
  Pose & operator=(Pose && p);

protected:
  tf2::Vector3 position_ = {0.0, 0.0, 0.0}; // [m]
  tf2::Quaternion attitude_ = tf2::Quaternion::getIdentity();
  std::array<double, 36> pose_covariance_{};
  std_msgs::msg::Header header_{};
};

} // namespace pose_kit
