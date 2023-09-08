/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  Copyright (c) 2023, Metro Robots
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: David V. Lu!! */

#include <base2d_kinematics/kinematic_parameters.hpp>
#include <cmath>

namespace base2d_kinematics
{

KinematicParameters::KinematicParameters()
  : min_vel_x_(0.0), min_vel_y_(0.0), max_vel_x_(0.0), max_vel_y_(0.0), max_vel_theta_(0.0), min_speed_xy_(0.0),
    max_speed_xy_(0.0), min_speed_theta_(0.0), acc_lim_x_(0.0), acc_lim_y_(0.0), acc_lim_theta_(0.0), decel_lim_x_(0.0),
    decel_lim_y_(0.0), decel_lim_theta_(0.0), min_speed_xy_sq_(0.0), max_speed_xy_sq_(0.0)
{
}

void KinematicParameters::initialize(const base2d_kinematics_msgs::msg::Base2DKinematics& msg)
{
  min_vel_x_ = msg.min_vel_x;
  max_vel_x_ = msg.max_vel_x;
  min_vel_y_ = msg.min_vel_y;
  max_vel_y_ = msg.max_vel_y;
  max_vel_theta_ = msg.max_vel_theta;
  acc_lim_x_ = msg.acc_lim_x;
  acc_lim_y_ = msg.acc_lim_y;
  decel_lim_x_ = msg.decel_lim_x;
  decel_lim_y_ = msg.decel_lim_y;
  acc_lim_theta_ = msg.acc_lim_theta;
  decel_lim_theta_ = msg.decel_lim_theta;
  min_speed_xy_ = msg.min_speed_xy;
  max_speed_xy_ = msg.max_speed_xy;
  min_speed_theta_ = msg.min_speed_theta;

  min_speed_xy_sq_ = min_speed_xy_ * min_speed_xy_;
  max_speed_xy_sq_ = max_speed_xy_ * max_speed_xy_;
}

/**
 * @brief Helper function to set the deceleration to the negative acceleration if it was not already set.
 */
void loadAccelDecel(const rclcpp::Node::SharedPtr& node, const std::string& param_prefix, const std::string& dimension,
                    double& accel, double& decel)
{
  std::string accel_param = param_prefix + "acc_lim_" + dimension,
              decel_param = param_prefix + "decel_lim_" + dimension;
  node->get_parameter(accel_param, accel);

  if (node->has_parameter(decel_param))
  {
    node->get_parameter(decel_param, decel);
  }
  else
  {
    decel = 0.0;
  }

  if (decel == 0.0)
  {
    decel = -accel;
  }
}

void KinematicParameters::initialize(const rclcpp::Node::SharedPtr& node, const std::string& param_prefix)
{
  node_ = node;

  node->declare_parameter<double>(param_prefix + "min_vel_x", -1.0);
  node->declare_parameter<double>(param_prefix + "max_vel_x", 1.0);
  node->declare_parameter<double>(param_prefix + "min_vel_y", 0.0);
  node->declare_parameter<double>(param_prefix + "max_vel_y", 0.0);
  node->declare_parameter<double>(param_prefix + "max_vel_theta", 1.0);
  node->declare_parameter<double>(param_prefix + "acc_lim_x", 1.0);
  node->declare_parameter<double>(param_prefix + "acc_lim_y", 0.0);
  node->declare_parameter<double>(param_prefix + "decel_lim_x", 0.0);
  node->declare_parameter<double>(param_prefix + "decel_lim_y", 0.0);
  node->declare_parameter<double>(param_prefix + "acc_lim_theta", 1.0);
  node->declare_parameter<double>(param_prefix + "decel_lim_theta", 0.0);
  node->declare_parameter<double>(param_prefix + "min_speed_xy", 0.0);
  node->declare_parameter<double>(param_prefix + "max_speed_xy", 0.0);
  node->declare_parameter<double>(param_prefix + "min_speed_theta", 0.0);

  node->get_parameter(param_prefix + "min_vel_x", min_vel_x_);
  node->get_parameter(param_prefix + "max_vel_x", max_vel_x_);
  node->get_parameter(param_prefix + "min_vel_y", min_vel_y_);
  node->get_parameter(param_prefix + "max_vel_y", max_vel_y_);
  node->get_parameter(param_prefix + "max_vel_theta", max_vel_theta_);

  loadAccelDecel(node, param_prefix, "x", acc_lim_x_, decel_lim_x_);
  loadAccelDecel(node, param_prefix, "y", acc_lim_y_, decel_lim_y_);
  loadAccelDecel(node, param_prefix, "theta", acc_lim_theta_, decel_lim_theta_);

  node->get_parameter(param_prefix + "min_speed_xy", min_speed_xy_);
  node->get_parameter(param_prefix + "max_speed_xy", max_speed_xy_);
  if (max_speed_xy_ <= 0.0)
  {
    max_speed_xy_ = hypot(max_vel_x_, max_vel_y_);
  }
  node->get_parameter(param_prefix + "min_speed_theta", min_speed_theta_);

  min_speed_xy_sq_ = min_speed_xy_ * min_speed_xy_;
  max_speed_xy_sq_ = max_speed_xy_ * max_speed_xy_;
}

void KinematicParameters::startPublisher(const rclcpp::Node::SharedPtr& node)
{
  kinematics_pub_ = node->create_publisher<base2d_kinematics_msgs::msg::Base2DKinematics>(
      "base2d_kinematics",
      // Transient local is similar to latching in ROS 1.
      rclcpp::QoS(1).transient_local());
  kinematics_pub_->publish(toMsg());
}

void KinematicParameters::startSubscriber(const rclcpp::Node::SharedPtr& node)
{
  kinematics_sub_ = node->create_subscription<base2d_kinematics_msgs::msg::Base2DKinematics>(
      "base2d_kinematics", rclcpp::QoS(1).transient_local(),
      std::bind(&KinematicParameters::kinematicsCB, this, std::placeholders::_1));
}

base2d_kinematics_msgs::msg::Base2DKinematics KinematicParameters::toMsg() const
{
  base2d_kinematics_msgs::msg::Base2DKinematics msg;
  msg.min_vel_x = min_vel_x_;
  msg.max_vel_x = max_vel_x_;
  msg.min_vel_y = min_vel_y_;
  msg.max_vel_y = max_vel_y_;
  msg.max_vel_theta = max_vel_theta_;
  msg.acc_lim_x = acc_lim_x_;
  msg.acc_lim_y = acc_lim_y_;
  msg.decel_lim_x = decel_lim_x_;
  msg.decel_lim_y = decel_lim_y_;
  msg.acc_lim_theta = acc_lim_theta_;
  msg.decel_lim_theta = decel_lim_theta_;
  msg.min_speed_xy = min_speed_xy_;
  msg.max_speed_xy = max_speed_xy_;
  msg.min_speed_theta = min_speed_theta_;
  return msg;
}

double dclamp(double n, double lower, double upper)
{
  return std::max(lower, std::min(n, upper));
}

nav_2d_msgs::msg::Twist2D KinematicParameters::calculateNewVelocity(const nav_2d_msgs::msg::Twist2D& cmd_vel,
                                                                    const nav_2d_msgs::msg::Twist2D& start_vel,
                                                                    const double dt)
{
  nav_2d_msgs::msg::Twist2D new_vel;
  new_vel.x = projectVelocity(start_vel.x, acc_lim_x_, decel_lim_x_, dt, cmd_vel.x);
  new_vel.y = projectVelocity(start_vel.y, acc_lim_y_, decel_lim_y_, dt, cmd_vel.y);
  new_vel.theta = projectVelocity(start_vel.theta, acc_lim_theta_, decel_lim_theta_, dt, cmd_vel.theta);
  clamp(new_vel.x, new_vel.y, new_vel.theta);
  return new_vel;
}

void KinematicParameters::clamp(double& x, double& y, double& theta)
{
  x = dclamp(x, min_vel_x_, max_vel_x_);
  y = dclamp(y, min_vel_y_, max_vel_y_);
  theta = dclamp(theta, -max_vel_theta_, max_vel_theta_);

  double vmag_sq = x * x + y * y;
  if (max_speed_xy_ > 0.0 && vmag_sq > max_speed_xy_sq_)
  {
    double mult = max_speed_xy_ / sqrt(vmag_sq);
    x *= mult;
    y *= mult;
  }

  if (min_speed_xy_ >= 0.0 && vmag_sq < min_speed_xy_sq_)
  {
    x = 0.0;
    y = 0.0;
  }

  if (min_speed_theta_ > 0.0 && abs(theta) < min_speed_theta_)
  {
    theta = 0.0;
  }
}

geometry_msgs::msg::Pose2D KinematicParameters::calculateNewPosition(const geometry_msgs::msg::Pose2D start_pose,
                                                                     const nav_2d_msgs::msg::Twist2D& vel,
                                                                     const double dt)
{
  geometry_msgs::msg::Pose2D new_pose;
  new_pose.x = start_pose.x + (vel.x * cos(start_pose.theta) + vel.y * cos(M_PI_2 + start_pose.theta)) * dt;
  new_pose.y = start_pose.y + (vel.x * sin(start_pose.theta) + vel.y * sin(M_PI_2 + start_pose.theta)) * dt;
  new_pose.theta = start_pose.theta + vel.theta * dt;
  return new_pose;
}

bool KinematicParameters::isValidSpeed(double x, double y, double theta)
{
  double vmag_sq = x * x + y * y;
  if (max_speed_xy_ >= 0.0 && vmag_sq > max_speed_xy_sq_ + EPSILON)
    return false;
  if ((min_speed_xy_ >= 0.0 && vmag_sq + EPSILON < min_speed_xy_sq_) &&
      (min_speed_theta_ >= 0.0 && abs(theta) + EPSILON < min_speed_theta_))
    return false;
  if (vmag_sq == 0.0 && theta == 0.0)
    return false;
  return true;
}

}  // namespace base2d_kinematics
