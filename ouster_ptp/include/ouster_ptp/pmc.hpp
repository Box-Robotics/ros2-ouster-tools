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
#ifndef OUSTER_PTP__PMC_H_
#define OUSTER_PTP__PMC_H_

#include <memory>
#include <string>
#include <vector>

#include <ouster_ptp/visibility_control.h>

namespace ouster_ptp
{
  /**
   * Wrapper over the Linux PTP management client
   *
   * This class provides a simple interface to access the functionality of
   * `pmc` the PTP management client. Its functionality is limited to only
   * those features that we are currently using in this toolbox. As we need
   * more features, this interface will expose them. Said another way, this is
   * not a comprehensive C++ interface to PTP or PMC but serves a purpose for
   * time synchronizing a host machine and Ouster LiDARs.
   */
  class OUSTER_PTP_PUBLIC pmc
  {
  public:
    using Ptr = std::unique_ptr<pmc>;
    using SharedPtr = std::shared_ptr<pmc>;

    /**
     * Constructs the PTP Management Client (PMC)
     *
     * Our current implementation assumes a Unix Domain Socket (UDS) is used to
     * receive PTP management messages from localhost.
     *
     * @param[in] boundary_hops Number of boundary hops
     * @param[in] uds_prefix The prefix (pid will get appended) of the UDS for
     *                       IPC with linuxptp
     * @param[in] domain_number PTP domain number
     * @param[in] poll_timeout Timeout in millis to wait for server response
     */
    explicit pmc(int boundary_hops,
                 const std::string& uds_prefix = "/var/tmp/ou_pmc.sock.",
                 int domain_number = 0,
                 int poll_timeout = 100);

    /**
     * RAII deallocs
     */
    ~pmc();

    /**
     * Runs a PMC command and returns its results serialized to JSON
     *
     * @param[in] cmd The command string
     * @return A JSON serialization of the command return
     */
    std::string poll(const std::string& cmd);

    /**
     * Runs a batch of PMC commands and returns the result as JSON
     *
     * @param[in] cmd_vec A vector of command strings
     * @return A JSON serialization of the command return
     */
    std::string poll(const std::vector<std::string>& cmd_vec);

    // disable copy/move
    pmc(pmc&&) = delete;
    pmc& operator=(pmc&&) = delete;
    pmc(const pmc&) = delete;
    pmc& operator=(const pmc&) = delete;

  private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

  }; // end: class pmc

} // end: namespace ouster_ptp

#endif // OUSTER_PTP__PMC_H_
