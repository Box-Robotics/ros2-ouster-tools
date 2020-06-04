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
#include <sstream>
#include <string>
#include <vector>
#include <ouster_ptp/pmc.hpp>
#include <pmc_impl.hpp>

ouster_ptp::pmc::pmc(int boundary_hops,
                     const std::string& uds_prefix,
                     int domain_number,
                     int poll_timeout)
  : pImpl(new ouster_ptp::pmc::Impl(
            boundary_hops, uds_prefix, domain_number, poll_timeout))
{ }

ouster_ptp::pmc::~pmc() = default;

std::string ouster_ptp::pmc::poll(const std::string& cmd)
{
  return this->pImpl->do_command(cmd);
}

std::string
ouster_ptp::pmc::poll(const std::vector<std::string>& cmd_vec)
{
  std::stringstream json_str;
  json_str << "[";

  int last = cmd_vec.size();
  int i = 0;
  for (const auto& cmd : cmd_vec)
    {
      json_str << "{\"" << cmd << "\":" << this->poll(cmd) << "}";
      if (i < (last - 1))
        {
          json_str << ",";
        }
      i++;
    }

  json_str << "]";
  return json_str.str();
}
