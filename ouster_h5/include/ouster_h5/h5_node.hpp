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
#ifndef OUSTER_H5__H5_NODE_H_
#define OUSTER_H5__H5_NODE_H_

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <h5_bridge.hpp>
#include <h5b_sensor_msgs.hpp>

#include <ouster_h5/visibility_control.h>

namespace
{
  using PCLMsg = sensor_msgs::msg::PointCloud2;
  using PCLMsgPtr = std::shared_ptr<const PCLMsg>;

  using IMGMsg = sensor_msgs::msg::Image;
  using IMGMsgPtr = std::shared_ptr<IMGMsg>;
}

namespace ouster_h5
{
  class H5Node : public rclcpp::Node
  {
  public:
    OUSTER_H5_PUBLIC
    explicit H5Node(const rclcpp::NodeOptions& opts)
      : Node("h5_node", opts),
        log_(this->get_logger())
    {
      this->declare_parameter("points_topic", "/points");
      this->declare_parameter("range_topic", "/range_image");
      this->declare_parameter("distance_topic", "/distance_image");
      this->declare_parameter("h5_outfile", "/tmp/h5_node.h5");
      this->declare_parameter("h5_root_group", "/ouster");

      this->points_topic_ = this->get_parameter("points_topic").as_string();
      this->range_topic_ = this->get_parameter("range_topic").as_string();
      this->distance_topic_ = this->get_parameter("distance_topic").as_string();
      this->h5_outfile_ = this->get_parameter("h5_outfile").as_string();
      this->h5_root_group_ = this->get_parameter("h5_root_group").as_string();

      this->h5_ = std::make_unique<h5_bridge::H5File>(this->h5_outfile_, "a");

      this->points_sub_ =
        this->create_subscription<PCLMsg>(
          this->points_topic_,
          rclcpp::SensorDataQoS(),
          [this](const PCLMsgPtr msg) -> void
          { this->cb(msg, this->points_topic_); });

      this->distance_sub_ =
        this->create_subscription<IMGMsg>(
          this->distance_topic_,
          rclcpp::SensorDataQoS(),
          [this](const IMGMsgPtr msg) -> void
          { this->cb(msg, this->distance_topic_); });

      // this->range_sub_ =
      //   this->create_subscription<IMGMsg>(
      //     this->range_topic_,
      //     rclcpp::SensorDataQoS(),
      //     [this](const IMGMsgPtr msg) -> void
      //     { this->cb(msg, this->range_topic_); });

    }

  private:
    template<typename T>
    void cb(const T msg, const std::string& key)
    {
      //RCLCPP_INFO(this->log_, "%s", __PRETTY_FUNCTION__);
      rclcpp::Time msg_time =
        rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec);
      std::string path =
        this->h5_root_group_ + "/" +
        std::to_string(msg_time.nanoseconds()) + key;

      {
        std::lock_guard<std::mutex> lock(this->h5_mutex_);
        h5b_sensor_msgs::write(this->h5_.get(), path, *msg);
        this->h5_->clear(path);
      }

      RCLCPP_INFO(this->log_, "Wrote: %s", path.c_str());
    }

    rclcpp::Logger log_;
    std::string points_topic_;
    std::string range_topic_;
    std::string distance_topic_;
    std::string h5_root_group_;
    std::string h5_outfile_;
    std::unique_ptr<h5_bridge::H5File> h5_;
    std::mutex h5_mutex_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr range_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr distance_sub_;

  }; // end: class H5Node

} // end: namespace ouster_h5

#endif // OUSTER_H5__H5_NODE_H_
