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
#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/string.hpp>

#include <ouster_tools/ptp.hpp>

using JSON_PUB =
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>>;
using TC_RETVAL =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * Managed node whose purpose is to publish results serialzed as JSON from a
 * set of user-defined pmc commands assuming a local ptp4l daemon.
 */
class PMCNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * Proxy to call the 2-arg version of the ctor
   */
  PMCNode(const rclcpp::NodeOptions& opts)
    : PMCNode("pmc_node", opts)
  {}

  /**
   * Declares allowable parameters and instantiates the publisher.
   */
  PMCNode(const std::string& node_name,
          const rclcpp::NodeOptions& opts)
    : rclcpp_lifecycle::LifecycleNode(node_name, opts),
    logger_(this->get_logger())
  {
    RCLCPP_INFO(this->logger_, "namespace: %s", this->get_namespace());
    RCLCPP_INFO(this->logger_, "node name: %s", this->get_name());

    this->declare_parameter("boundary_hops", 1);
    this->declare_parameter("uds_prefix", "/var/tmp/ouster_tools_pmc.sock.");
    this->declare_parameter("domain_number", 0);
    this->declare_parameter("poll_timeout", 100);
    this->declare_parameter("poll_period", 1.0f);
    this->declare_parameter<std::vector<std::string>>(
      "pmc_commands", {"GET CURRENT_DATA_SET", "GET TIME_STATUS_NP"});

    this->pub_ =
      this->create_publisher<std_msgs::msg::String>(
        "ptp", rclcpp::SystemDefaultsQoS());

    // - create the service servers

    RCLCPP_INFO(this->logger_, "node created, waiting for `configure()`...");
  }

  ~PMCNode() = default;

  /**
   * Constructs the `pmc` interface from the configured ROS params
   */
  TC_RETVAL on_configure(const rclcpp_lifecycle::State& prev_state)
  {
    RCLCPP_INFO(this->logger_, "on_configure(): %s -> %s",
                prev_state.label().c_str(),
                this->get_current_state().label().c_str());

    this->pmc_ =
      std::make_shared<ouster_tools::ptp::pmc>(
        this->get_parameter("boundary_hops").get_value<int>(),
        this->get_parameter("uds_prefix").as_string(),
        this->get_parameter("domain_number").get_value<int>(),
        this->get_parameter("poll_timeout").get_value<int>());

    RCLCPP_INFO(this->logger_, "Configuration complete.");
    return TC_RETVAL::SUCCESS;
  }

  /**
   * Activates the publisher and constructs a timer-based callback trigger to
   * poll the PTP network and publish results.
   */
  TC_RETVAL on_activate(const rclcpp_lifecycle::State& prev_state)
  {
    RCLCPP_INFO(this->logger_, "on_activate(): %s -> %s",
                prev_state.label().c_str(),
                this->get_current_state().label().c_str());

    this->pub_->on_activate();

    auto cmds = this->get_parameter("pmc_commands").as_string_array();
    auto timer_cb = [this, cmds]() -> void
      {
        if (this->pub_->get_subscription_count() <= 0)
          {
            return;
          }

        try
          {
            //RCLCPP_INFO(this->logger_, this->pmc_->poll(cmds));
            auto msg = std::make_unique<std_msgs::msg::String>();
            msg->data = this->pmc_->poll(cmds);
            this->pub_->publish(std::move(msg));
          }
        catch (const ouster_tools::ptp::error_t& ex)
          {
            RCLCPP_WARN(this->logger_, ex.what());
          }
      };
    this->timer_ =
      this->create_wall_timer(
        std::chrono::duration<long double>(
          this->get_parameter("poll_period").as_double()),
        timer_cb);

    RCLCPP_INFO(this->logger_, "Activation complete.");
    return TC_RETVAL::SUCCESS;
  }

  /**
   * Destroys the timer callback and deactivatest the publisher.
   */
  TC_RETVAL on_deactivate(const rclcpp_lifecycle::State& prev_state)
  {
    RCLCPP_INFO(this->logger_, "on_deactivate(): %s -> %s",
                prev_state.label().c_str(),
                this->get_current_state().label().c_str());

    if (this->timer_)
      {
        this->timer_.reset();
      }

    this->pub_->on_deactivate();

    RCLCPP_INFO(this->logger_, "Deactivation complete.");
    return TC_RETVAL::SUCCESS;
  }

  /**
   * Destructs the `pmc` interface.
   */
  TC_RETVAL on_cleanup(const rclcpp_lifecycle::State& prev_state)
  {
    RCLCPP_INFO(this->logger_, "on_cleanup(): %s -> %s",
                prev_state.label().c_str(),
                this->get_current_state().label().c_str());

    if (this->pmc_)
      {
        this->pmc_.reset();
      }

    RCLCPP_INFO(this->logger_, "Clean-up complete.");
    return TC_RETVAL::SUCCESS;
  }

  /**
   * NOOP for now.
   */
  TC_RETVAL on_shutdown(const rclcpp_lifecycle::State& prev_state)
  {
    RCLCPP_INFO(this->logger_, "on_shutdown(): %s -> %s",
                prev_state.label().c_str(),
                this->get_current_state().label().c_str());

    RCLCPP_INFO(this->logger_, "Shutdown complete.");
    return TC_RETVAL::SUCCESS;
  }


  /**
   * Tries to force dtors to run for all held objects.
   */
  TC_RETVAL on_error(const rclcpp_lifecycle::State& prev_state)
  {
    RCLCPP_INFO(this->logger_, "on_error(): %s -> %s",
                prev_state.label().c_str(),
                this->get_current_state().label().c_str());

    if (this->timer_)
      {
        this->timer_.reset();
      }

    if (this->pmc_)
      {
        this->pmc_.reset();
      }

    this->pub_->on_deactivate();

    RCLCPP_INFO(this->logger_, "Error processing complete.");
    return TC_RETVAL::SUCCESS;
  }

private:
  rclcpp::Logger logger_;
  JSON_PUB pub_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
  ouster_tools::ptp::pmc::SharedPtr pmc_;

}; // end: class PMCNode


int main(int argc, char ** argv)
{
  std::setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto pmc = std::make_shared<PMCNode>(options);
  exec.add_node(pmc->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
