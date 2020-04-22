// -*- c++ -*-
/*
 * Copyright (C) 2020 Box Robotics, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef OUSTER_TOOLS__PERF_PERF_NODE_H_
#define OUSTER_TOOLS__PERF_PERF_NODE_H_

#include <atomic>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <ouster_tools/visibility_control.h>

namespace
{
  using PCLMsg = sensor_msgs::msg::PointCloud2;
  using PCLMsgPtr = std::shared_ptr<PCLMsg>;

  using IMGMsg = sensor_msgs::msg::Image;
  using IMGMsgPtr = std::shared_ptr<IMGMsg>;
}

namespace ouster_tools
{
  class PerfNode : public rclcpp::Node
  {
  public:
    OUSTER_TOOLS_PUBLIC
    explicit PerfNode(const rclcpp::NodeOptions& opts)
      : Node("perf_node", opts),
        log_(this->get_logger()),
        sample_count_(0)
    {
      this->declare_parameter("mode", "jitter");
      this->declare_parameter("topic", "/points");
      this->declare_parameter("out_file", "/tmp/perf_node.txt");
      this->declare_parameter("n_samples", 3000);

      this->mode_ = this->get_parameter("mode").as_string();
      this->topic_ = this->get_parameter("topic").as_string();
      this->out_file_ = this->get_parameter("out_file").as_string();
      this->n_samples_ = this->get_parameter("n_samples").as_int();

      this->msg_stamp_ = std::vector<std::uint64_t>(this->n_samples_, 0);
      this->recv_stamp_ = std::vector<std::uint64_t>(this->n_samples_, 0);

      //
      // Establishing a super-lightweight "framework" for me to test different
      // modes/metrics across different topics/data types.
      //
      if (this->mode_ == "jitter")
        {
          if (this->topic_ == "/points")
            {
              this->points_sub_ =
                this->create_subscription<PCLMsg>(
                  this->topic_,
                  rclcpp::SensorDataQoS(),
                  std::bind(&PerfNode::jitter_cb<PCLMsgPtr>, this,
                            std::placeholders::_1));
            }
          else
            {
              RCLCPP_WARN(this->log_,
                          "Unsupported 'topic': %s", this->topic_.c_str());
            }
        }
      else
        {
          RCLCPP_WARN(this->log_,
                      "Unsupported 'mode': %s", this->mode_.c_str());
        }
    }

  private:
    template<typename T>
    void jitter_cb(const T msg)
    {
      auto now = this->now().nanoseconds();
      int idx = this->sample_count_.load();
      this->recv_stamp_[idx] = now;

      rclcpp::Time msg_time =
        rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec);
      this->msg_stamp_[idx] = msg_time.nanoseconds();

      this->sample_count_++;
      RCLCPP_INFO(this->log_, "%d", this->sample_count_.load());

      if (this->sample_count_ == this->n_samples_)
        {
          RCLCPP_INFO(this->log_, "Collected %d samples", this->n_samples_);
          RCLCPP_INFO(this->log_,
                      "Writing output to: %s", this->out_file_.c_str());
          this->write_results();

          RCLCPP_WARN(this->log_, "Data collection complete, exiting.");
          rclcpp::shutdown();
        }
    }

    void write_results()
    {
      std::ofstream out;
      out.open(this->out_file_);
      out << "recv_stamp,msg_stamp" << std::endl;

      for (int i = 0; i < this->sample_count_.load(); ++i)
        {
          out << this->recv_stamp_[i] << ","
              << this->msg_stamp_[i] << std::endl;
        }

      out.close();
    }

    rclcpp::Logger log_;
    std::string mode_;
    std::string topic_;
    std::string out_file_;
    int n_samples_;
    std::atomic<int> sample_count_;

    std::vector<std::uint64_t> msg_stamp_;
    std::vector<std::uint64_t> recv_stamp_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  }; // end: class PerfNode

} // end: namespace ouster_tools

#endif // OUSTER_TOOLS__PERF_PERF_NODE_H_
