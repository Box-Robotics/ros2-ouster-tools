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
#ifndef OUSTER_PTP__ERR_H_
#define OUSTER_PTP__ERR_H_

#include <exception>
#include <ouster_ptp/visibility_control.h>

namespace ouster_ptp
{
  extern OUSTER_PTP_PUBLIC const int OK;
  extern OUSTER_PTP_PUBLIC const int LINUXPTP_ERR;
  extern OUSTER_PTP_PUBLIC const int LINUXPTP_CTOR_ERR;
  extern OUSTER_PTP_PUBLIC const int LINUXPTP_CONFIG_ERR;
  extern OUSTER_PTP_PUBLIC const int LINUXPTP_POLL_ERR;
  extern OUSTER_PTP_PUBLIC const int LINUXPTP_CMDEXE_ERR;

  /**
   * Human-readable stringification of an error code
   *
   * @param[in] errnum The error code to convert to a string
   * @return A stringified version of the error code
   */
  const char *strerror(int errnum);

  /**
   * Exception wrapper for error codes
   */
  class OUSTER_PTP_PUBLIC error_t : public std::exception
  {
  public:
    /**
     * Constructs and error_t (exception) from an error code.
     */
    error_t(int errnum);

    /**
     * Retrieves the exception message
     */
    virtual const char *what() const noexcept;

    /**
     * Accessor to the internally wrapped error code
     */
    int code() const noexcept;

  private:
    int errnum_;

  }; // end: class error_t

} // end: namespace ouster_ptp

#endif // OUSTER_PTP__ERR_H_
