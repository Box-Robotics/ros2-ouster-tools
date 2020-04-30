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
#ifndef OUSTER_TOOLS__UTIL_WAIT_FOR_RESULT_H_
#define OUSTER_TOOLS__UTIL_WAIT_FOR_RESULT_H_

#include <chrono>
#include <future>
#include <rclcpp/rclcpp.hpp>
#include <ouster_tools/visibility_control.h>

namespace ouster_tools
{
  /**
   * Waits for a future to be ready with a maximum timeout duration.
   *
   * This function provides a blocking (for no longer than the specified
   * `timeout` duration) busy wait loop for a future to complete. It takes the
   * entire duration and chunks it into 100 ms wait periods so that it can
   * return early should the ROS middleware request a shutdown.
   *
   * @param[in] future The `future` to wait on
   * @param[in] timeout The max `duration` to wait for completion.
   *
   * @return The `future_status` of the `future`.
   */
  template<typename FutureT, typename TimeoutT>
  OUSTER_TOOLS_PUBLIC std::future_status
  wait_for_result(FutureT& future, TimeoutT timeout)
  {
    auto end = std::chrono::steady_clock::now() + timeout;
    std::chrono::milliseconds wait_period(100);
    std::future_status status = std::future_status::timeout;

    do
      {
        auto now = std::chrono::steady_clock::now();
        auto time_left = end - now;
        if (time_left <= std::chrono::seconds(0))
          {
            break;
          }

        status =
          future.wait_for((time_left < wait_period) ? time_left : wait_period);

      } while (rclcpp::ok() && (status != std::future_status::ready));

    return status;
  }

} // end: namespace ouster_tools

#endif // OUSTER_TOOLS__UTIL_WAIT_FOR_RESULT_H_
