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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <metro_benchmark_msgs/msg/compute_time.hpp>

#include <chrono>
#include <string>

namespace metro_benchmark_pub
{
typedef struct BenchmarkContextS
{
  std::string name;
  std::string id;
  std::chrono::high_resolution_clock::time_point start;

  BenchmarkContextS(const std::string& name, const std::string& id)
    : name(name), id(id), start(std::chrono::high_resolution_clock::now())
  {
  }
} BenchmarkContext;

class BenchmarkPublisher
{
public:
  BenchmarkPublisher(const rclcpp::Node::SharedPtr& node, const std::string& topic);

  void tick(const std::string& name);
  double tock(bool log = true);

protected:
  std::list<BenchmarkContext> stack_;
  std::unordered_map<std::string, int> counter_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<metro_benchmark_msgs::msg::ComputeTime>::SharedPtr pub_;
};
}  // namespace metro_benchmark_pub
