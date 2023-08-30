/*********************************************************************
 * Software License Agreement (BSD License)
 *
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
 *   * Neither the name of Metro Robots nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: David V. Lu!! */

#include <kinematics_2d/noise_model.hpp>

namespace kinematics_2d
{
std::vector<double> loadParametersToVector(const rclcpp::Node::SharedPtr& node, const std::string& param_prefix)
{
  std::vector<double> values;

  for (const std::string& dimension : {"x", "y", "theta"})
  {
    for (const std::string& type : {"mean", "covariance"})
    {
      std::string name = param_prefix + type + "_" + dimension;
      node->declare_parameter<double>(name, 0.0);
      values.push_back(node->get_parameter(name).as_double());
    }
  }
  return values;
}

NoiseModel::NoiseModel(double mean_x, double stddev_x, double mean_y, double stddev_y, double mean_theta,
                       double stddev_theta)
  : noise_x_(mean_x, stddev_x), noise_y_(mean_y, stddev_y), noise_theta_(mean_theta, stddev_theta)
{
}

NoiseModel::NoiseModel(const std::vector<double>& six_params)
  : NoiseModel(six_params[0], six_params[1], six_params[2], six_params[3], six_params[4], six_params[5])
{
}
NoiseModel::NoiseModel(const rclcpp::Node::SharedPtr& node, const std::string& param_prefix)
  : NoiseModel(loadParametersToVector(node, param_prefix))
{
}

nav_2d_msgs::msg::Twist2D NoiseModel::applyNoise(const nav_2d_msgs::msg::Twist2D& base)
{
  static std::random_device random_device;
  static std::mt19937 random_generator(random_device());

  nav_2d_msgs::msg::Twist2D noisy = base;
  noisy.x += noise_x_(random_generator);
  noisy.y += noise_y_(random_generator);
  noisy.theta += noise_theta_(random_generator);
  return noisy;
}

std::array<double, 36> NoiseModel::getCovarianceMatrix() const
{
  std::array<double, 36> matrix;

  matrix[0] = noise_x_.stddev();
  matrix[7] = noise_y_.stddev();
  matrix[14] = 1000000000000.0;
  matrix[21] = 1000000000000.0;
  matrix[28] = 1000000000000.0;
  matrix[35] = noise_theta_.stddev();
  return matrix;
}
}  // namespace kinematics_2d
