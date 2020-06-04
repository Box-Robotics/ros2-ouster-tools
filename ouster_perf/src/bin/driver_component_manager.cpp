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
#include <functional>
#include <future>
#include <memory>
#include <string>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <ouster_tools/util.hpp>

using TC_RETVAL =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using TE_EVENT_MSG = lifecycle_msgs::msg::TransitionEvent;
using TE_SUB = rclcpp::Subscription<TE_EVENT_MSG>::SharedPtr;

using CS_MSG = lifecycle_msgs::srv::ChangeState;
using CS_CLIENT = std::shared_ptr<rclcpp::Client<CS_MSG>>;

using GS_MSG = lifecycle_msgs::srv::GetState;
using GS_CLIENT = std::shared_ptr<rclcpp::Client<GS_MSG>>;


/**
 * Lifecycle manager for the ROS2 Ouster driver when running as a
 * component. This node is also a managed node but is intended to be run out of
 * process from the driver (i.e., not in the same component container). To that
 * end, this node's FSM should be managed via launch.
 */
class DriverComponentManager : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * Proxy to call the 2-arg version of the ctor
   */
  DriverComponentManager(const rclcpp::NodeOptions& opts)
    : DriverComponentManager("driver_component_manager", opts)
  {}

  /**
   * The ctor parses the configuration file and creates a subscription to the
   * driver's `transition_event` topic.
   */
  DriverComponentManager(const std::string& node_name,
                         const rclcpp::NodeOptions& opts)
    : rclcpp_lifecycle::LifecycleNode(node_name, opts),
    logger_(this->get_logger())
  {
    RCLCPP_INFO(this->logger_, "%s/%s",
                this->get_namespace(), this->get_name());

    this->declare_parameter("driver_node", "/ouster_driver");
    this->declare_parameter("timeout_millis", 1000);

    this->driver_node_ = this->get_parameter("driver_node").as_string();
    this->timeout_millis_ = this->get_parameter("timeout_millis").as_int();

    this->sub_ =
      this->create_subscription<TE_EVENT_MSG>(
        this->driver_node_ + "/transition_event",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&DriverComponentManager::te_callback, this,
                  std::placeholders::_1));

    this->client_get_state_ =
      this->create_client<GS_MSG>(this->driver_node_ + "/get_state");
    this->client_change_state_ =
      this->create_client<CS_MSG>(this->driver_node_ + "/change_state");

    RCLCPP_INFO(this->logger_, "node created, waiting for `configure()`...");
  }

  ~DriverComponentManager() = default;

  /**
   * Queues a service call to get the current state of the driver.
   */
  TC_RETVAL on_configure(const rclcpp_lifecycle::State& prev_state)
  {
    RCLCPP_INFO(this->logger_, "on_configure(): %s -> %s",
                prev_state.label().c_str(),
                this->get_current_state().label().c_str());

    auto timeout = std::chrono::milliseconds(this->timeout_millis_);

    //
    // make sure both `get_state` and `change_state` are available for us to
    // call on the driver.
    //

    if (! this->client_get_state_->wait_for_service(timeout))
      {
        RCLCPP_WARN(this->logger_,
                    "Service not available: %s",
                    this->client_get_state_->get_service_name());
        return TC_RETVAL::FAILURE;
      }

    if (! this->client_change_state_->wait_for_service(timeout))
      {
        RCLCPP_WARN(this->logger_,
                    "Service not available: %s",
                    this->client_change_state_->get_service_name());
        return TC_RETVAL::FAILURE;
      }

    // we need to exit this callback before checking if the future is ready
    // (which is why we split this across `on_configure` and `on_activate`). If
    // we waited on the future here, we would have recursive spinning which the
    // ROS2 middleware executors do not support. By leaving the callback, we
    // allow the future to `spin` and we pick up its result in `on_activate`.
    auto req = std::make_shared<GS_MSG::Request>();
    this->gs_future_result_ = this->client_get_state_->async_send_request(req);

    if (! this->gs_future_result_.valid())
      {
        RCLCPP_WARN(this->logger_,
                    "Failed to queue service call: %s",
                    this->client_get_state_->get_service_name());

        return TC_RETVAL::FAILURE;
      }

    return TC_RETVAL::SUCCESS;
  }

  /**
   * Our goal here is to emit an event to the driver that will cause it to
   * activate. To that end, we handle three cases that we consider SUCCESS:
   *
   * 1. Driver is `unconfigured`, we emit `configure`
   * 2. Driver is `inactive`, we emit `activate`
   * 3. Driver is `active`, we do nothing
   *
   * If the driver is not in one of these states, we error out.
   *
   * NOTE: If a condition is met (see above) where we emit a transition event,
   * we simply queue the event via a future. Assuming the driver receives that
   * event, our `transition_event` subscriber, will be notified and drives the
   * rest of the transitions we would like to run the driver's FSM
   * through. This callback simply "knocks down the first domino" in the
   * cascade we wish to see happen.
   */
  TC_RETVAL on_activate(const rclcpp_lifecycle::State& prev_state)
  {
    RCLCPP_INFO(this->logger_, "on_activate(): %s -> %s",
                prev_state.label().c_str(),
                this->get_current_state().label().c_str());

    auto timeout = std::chrono::milliseconds(this->timeout_millis_);
    auto future_status =
      ouster_tools::wait_for_result(this->gs_future_result_, timeout);

    if (future_status != std::future_status::ready)
      {
        RCLCPP_WARN(this->logger_,
                    "Timeout calling: %s",
                    this->client_get_state_->get_service_name());
        return TC_RETVAL::FAILURE;
      }

    auto state = this->gs_future_result_.get()->current_state;
    RCLCPP_INFO(this->logger_, "Driver state: %d: %s",
                static_cast<int>(state.id), state.label.c_str());

    auto req = std::make_shared<CS_MSG::Request>();

    switch (state.id)
      {
      case state.PRIMARY_STATE_UNCONFIGURED:
        RCLCPP_INFO(this->logger_, "Emitting configure event...");
        req->transition.id = req->transition.TRANSITION_CONFIGURE;
        this->cs_future_result_ =
          this->client_change_state_->async_send_request(req);
        break;

      case state.PRIMARY_STATE_INACTIVE:
        RCLCPP_INFO(this->logger_, "Emitting activate event...");
        req->transition.id = req->transition.TRANSITION_ACTIVATE;
        this->cs_future_result_ =
          this->client_change_state_->async_send_request(req);
        break;

      case state.PRIMARY_STATE_ACTIVE:
        RCLCPP_INFO(this->logger_, "Driver already active, nothing to do.");
        break;

      default:
        RCLCPP_WARN(this->logger_, "Cannot handle: %d: %s",
                    static_cast<int>(state.id), state.label.c_str());
        return TC_RETVAL::FAILURE;
      }

    return TC_RETVAL::SUCCESS;
  }

  /**
   * Currently, the launch file that is intended to bring this node up
   * will simply try to reactivate us if we are asked to deactivate ... it is a
   * "high availability" approach. To that end, we do nothing here.
   */
  TC_RETVAL on_deactivate(const rclcpp_lifecycle::State& prev_state)
  {
    RCLCPP_INFO(this->logger_, "on_deactivate(): %s -> %s",
                prev_state.label().c_str(),
                this->get_current_state().label().c_str());

    return TC_RETVAL::SUCCESS;
  }

  /**
   * See notes above for `on_deactivate`.
   */
  TC_RETVAL on_cleanup(const rclcpp_lifecycle::State& prev_state)
  {
    RCLCPP_INFO(this->logger_, "on_cleanup(): %s -> %s",
                prev_state.label().c_str(),
                this->get_current_state().label().c_str());

    return TC_RETVAL::SUCCESS;
  }

  /**
   * Attempts to also shutdown the driver.
   */
  TC_RETVAL on_shutdown(const rclcpp_lifecycle::State& prev_state)
  {
    RCLCPP_INFO(this->logger_, "on_shutdown(): %s -> %s",
                prev_state.label().c_str(),
                this->get_current_state().label().c_str());

    auto timeout = std::chrono::milliseconds(this->timeout_millis_);
    auto req = std::make_shared<CS_MSG::Request>();
    req->transition.id = req->transition.TRANSITION_ACTIVE_SHUTDOWN;
    this->cs_future_result_ =
      this->client_change_state_->async_send_request(req);

    // We log here b/c technically, the call is made after we exit due to how
    // the executors spinning works.
    RCLCPP_WARN(this->logger_, "Making open-loop call to driver to shutdown!");

    return TC_RETVAL::SUCCESS;
  }

  /**
   * XXX: Need to get some miles on this to see what type of error recovery I
   * need to support.
   */
  TC_RETVAL on_error(const rclcpp_lifecycle::State& prev_state)
  {
    RCLCPP_INFO(this->logger_, "on_error(): %s -> %s",
                prev_state.label().c_str(),
                this->get_current_state().label().c_str());

    return TC_RETVAL::SUCCESS;
  }

private:
  void te_callback(const std::shared_ptr<TE_EVENT_MSG> msg)
  {
    RCLCPP_INFO(this->logger_, "EVT: %s -> %s",
                msg->start_state.label.c_str(),
                msg->goal_state.label.c_str());

    auto req = std::make_shared<CS_MSG::Request>();

    rclcpp_lifecycle::State this_state = this->get_current_state();
    if (this_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        //
        // We are active, so we want the driver to move to active as well
        //
        if (((msg->start_state.id ==
              lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING) &&
             (msg->goal_state.id ==
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)) ||
            ((msg->start_state.id ==
              lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING) &&
             (msg->goal_state.id ==
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)))
          {
            RCLCPP_INFO(this->logger_, "Emitting activate event...");
            req->transition.id = req->transition.TRANSITION_ACTIVATE;
            this->cs_future_result_ =
              this->client_change_state_->async_send_request(req);
          }
      }
  }

  rclcpp::Logger logger_;
  std::string driver_node_;
  int timeout_millis_;
  TE_SUB sub_;
  CS_CLIENT client_change_state_;
  std::shared_future<CS_MSG::Response::SharedPtr> cs_future_result_;
  GS_CLIENT client_get_state_;
  std::shared_future<GS_MSG::Response::SharedPtr> gs_future_result_;

}; // end: class DriverComponentManager


int main(int argc, char ** argv)
{
  std::setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto mgr = std::make_shared<DriverComponentManager>(options);
  exec.add_node(mgr->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
