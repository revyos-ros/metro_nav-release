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

#pragma once

#include <base2d_kinematics_msgs/msg/base2_d_kinematics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_2d_msgs/msg/twist2_d.hpp>
#include <memory>

namespace base2d_kinematics
{
const double EPSILON = 1E-5;

/**
     * @brief Given initial conditions and a time, figure out the end velocity
     *
     * @param v0 Initial velocity
     * @param accel The acceleration rate
     * @param decel The deceleration rate
     * @param dt Delta time - amount of time to project into the future
     * @param target target velocity
     * @return The velocity dt seconds after v0.
     */
inline double projectVelocity(double v0, double accel, double decel, double dt, double target)
{
  double magnitude;
  if (fabs(v0) < EPSILON)
  {
    // Starting from standstill, always accelerate
    if (target >= 0.0)
    {
      magnitude = fabs(accel);
    }
    else
    {
      magnitude = -fabs(accel);
    }
  }
  else if (v0 > 0.0)
  {
    if (v0 < target)
    {
      // Acceleration (speed magnitude increases)
      magnitude = fabs(accel);
    }
    else
    {
      // Deceleration (speed magnitude decreases)
      magnitude = -fabs(decel);
    }
  }
  else
  {
    if (v0 < target)
    {
      // Deceleration (speed magnitude decreases)
      magnitude = fabs(decel);
    }
    else
    {
      // Acceleration (speed magnitude increases)
      magnitude = -fabs(accel);
    }
  }

  double v1 = v0 + magnitude * dt;
  if (magnitude > 0.0)
  {
    return std::min(target, v1);
  }
  else
  {
    return std::max(target, v1);
  }
}

/**
 * @class KinematicParameters
 * @brief One representation of the robot's kinematics
 */
class KinematicParameters
{
public:
  KinematicParameters();

  void initialize(const base2d_kinematics_msgs::msg::Base2DKinematics& msg);
  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& param_prefix = "");

  inline double getMinX()
  {
    return min_vel_x_;
  }
  inline double getMaxX()
  {
    return max_vel_x_;
  }
  inline double getAccX()
  {
    return acc_lim_x_;
  }
  inline double getDecelX()
  {
    return decel_lim_x_;
  }

  inline double getMinY()
  {
    return min_vel_y_;
  }
  inline double getMaxY()
  {
    return max_vel_y_;
  }
  inline double getAccY()
  {
    return acc_lim_y_;
  }
  inline double getDecelY()
  {
    return decel_lim_y_;
  }

  inline double getMinSpeedXY()
  {
    return min_speed_xy_;
  }

  inline double getMinTheta()
  {
    return -max_vel_theta_;
  }
  inline double getMaxTheta()
  {
    return max_vel_theta_;
  }
  inline double getAccTheta()
  {
    return acc_lim_theta_;
  }
  inline double getDecelTheta()
  {
    return decel_lim_theta_;
  }
  inline double getMinSpeedTheta()
  {
    return min_speed_theta_;
  }

  base2d_kinematics_msgs::msg::Base2DKinematics toMsg() const;

  void startPublisher(const rclcpp::Node::SharedPtr& node);
  void startSubscriber(const rclcpp::Node::SharedPtr& node);

  void clamp(double& x, double& y, double& theta);

  /**
   * @brief Check to see whether the combined x/y/theta velocities are valid
   * @return True if the magnitude hypot(x,y) and theta are within the robot's absolute limits
   *
   * This is based on three parameters: min_speed_xy, max_speed_xy and min_speed_theta.
   * The speed is valid if
   *  1) The combined magnitude hypot(x,y) is less than max_speed_xy (or max_speed_xy is negative)
   *  AND
   *  2) min_speed_xy is negative or min_speed_theta is negative or
   *     hypot(x,y) is greater than min_speed_xy or fabs(theta) is greater than min_speed_theta.
   *
   * In English, it makes sure the diagonal motion is not too fast,
   * and that the velocity is moving in some meaningful direction.
   *
   * In Latin, quod si motus sit signum quaerit et movere ieiunium et significantissime comprehendite.
   */
  bool isValidSpeed(double x, double y, double theta);

  /**
    * @brief Calculate the velocity after a set period of time, given the desired velocity and kinematics
    *
    * @param cmd_vel Desired velocity
    * @param start_vel starting velocity
    * @param dt amount of time in seconds
    * @return new velocity after dt seconds
    */
  virtual nav_2d_msgs::msg::Twist2D calculateNewVelocity(const nav_2d_msgs::msg::Twist2D& cmd_vel,
                                                         const nav_2d_msgs::msg::Twist2D& start_vel, const double dt);

  /**
   * @brief Use the robot's kinematic model to calculate new positions for the robot
   *
   * @param start_pose Starting pose
   * @param vel Actual robot velocity (assumed to be within acceleration limits)
   * @param dt amount of time in seconds
   * @return New pose after dt seconds
   */
  virtual geometry_msgs::msg::Pose2D calculateNewPosition(const geometry_msgs::msg::Pose2D start_pose,
                                                          const nav_2d_msgs::msg::Twist2D& vel, const double dt);

  using Ptr = std::shared_ptr<KinematicParameters>;

protected:
  void kinematicsCB(const base2d_kinematics_msgs::msg::Base2DKinematics::SharedPtr msg)
  {
    initialize(*msg);
  }

  double min_vel_x_, min_vel_y_;
  double max_vel_x_, max_vel_y_, max_vel_theta_;

  double min_speed_xy_, max_speed_xy_;
  double min_speed_theta_;

  double acc_lim_x_, acc_lim_y_, acc_lim_theta_;
  double decel_lim_x_, decel_lim_y_, decel_lim_theta_;

  // Cached square values of min_speed_xy and max_speed_xy
  double min_speed_xy_sq_, max_speed_xy_sq_;

  rclcpp::Subscription<base2d_kinematics_msgs::msg::Base2DKinematics>::SharedPtr kinematics_sub_{nullptr};
  rclcpp::Publisher<base2d_kinematics_msgs::msg::Base2DKinematics>::SharedPtr kinematics_pub_{nullptr};
  rclcpp::Node::SharedPtr node_{nullptr};
};
}  // namespace base2d_kinematics
