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

#include <benchmark_utils/benchmark_publisher.hpp>

namespace benchmark_utils
{
const int NANOMOD = 1000000000;

BenchmarkPublisher::BenchmarkPublisher(const rclcpp::Node::SharedPtr& node, const std::string& topic) : node_(node)
{
  if (node_)
    pub_ = node_->create_publisher<benchmark_msgs::msg::ComputeTime>(topic, 1);
}

void BenchmarkPublisher::tick(const std::string& name)
{
  int c;
  if (counter_.count(name) == 0)
  {
    c = 0;
  }
  else
  {
    c = counter_[name];
  }
  counter_[name] = c + 1;
  std::string ident = name + "_" + std::to_string(c);
  stack_.emplace_back(BenchmarkContext(name, ident));
}

double BenchmarkPublisher::tock(bool log)
{
  std::chrono::high_resolution_clock::time_point stop = std::chrono::high_resolution_clock::now();
  BenchmarkContext& cxt = stack_.back();
  std::chrono::nanoseconds duration_nano = stop - cxt.start;
  long d_nano_i = duration_nano.count();
  long s_nano_i = stop.time_since_epoch().count();

  benchmark_msgs::msg::ComputeTime msg;
  msg.header.frame_id = cxt.name;
  msg.header.stamp.sec = s_nano_i / NANOMOD;
  msg.header.stamp.nanosec = s_nano_i % NANOMOD;
  msg.duration.sec = d_nano_i / NANOMOD;
  msg.duration.nanosec = d_nano_i % NANOMOD;
  msg.id = cxt.id;

  stack_.pop_back();
  if (!stack_.empty())
  {
    msg.parent_id = stack_.back().id;
  }
  if (pub_)
    pub_->publish(msg);
  double duration_sec_f = d_nano_i / 1e9;
  if (log && node_)
  {
    RCLCPP_INFO(node_->get_logger(), "%s time: %.4f", msg.header.frame_id.c_str(), duration_sec_f);
  }

  return duration_sec_f;
}
}  // namespace benchmark_utils
