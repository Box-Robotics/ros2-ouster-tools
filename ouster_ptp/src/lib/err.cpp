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

#include <ouster_ptp/err.hpp>
#include <cstring>

const int ouster_ptp::OK = 0;
const int ouster_ptp::LINUXPTP_ERR = -900000;
const int ouster_ptp::LINUXPTP_CTOR_ERR = -900001;
const int ouster_ptp::LINUXPTP_CONFIG_ERR = -900002;
const int ouster_ptp::LINUXPTP_POLL_ERR = -900003;
const int ouster_ptp::LINUXPTP_CMDEXE_ERR = -900004;

const char *ouster_ptp::strerror(int errnum)
{
  switch (errnum)
    {
    case ouster_ptp::OK:
      return "OK";
    case ouster_ptp::LINUXPTP_ERR:
      return "linuxptp: Generic error";
    case ouster_ptp::LINUXPTP_CTOR_ERR:
      return "linuxptp: Construction or allocation error";
    case ouster_ptp::LINUXPTP_CONFIG_ERR:
      return "linuxptp: Configuration error";
    case ouster_ptp::LINUXPTP_POLL_ERR:
      return "linuxptp: poll failed";
    case ouster_ptp::LINUXPTP_CMDEXE_ERR:
      return "linuxptp: Failed in pmc command execution";
    default:
      return ::strerror(errnum);
    }
}

ouster_ptp::error_t::error_t(int errnum)
  : std::exception(), errnum_(errnum) { }

int ouster_ptp::error_t::code() const noexcept
{
  return this->errnum_;
}

const char *ouster_ptp::error_t::what() const noexcept
{
  return ouster_ptp::strerror(this->code());
}
