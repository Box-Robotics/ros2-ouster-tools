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
#ifndef OUSTER_TOOLS__PTP_PMC_IMPL_H_
#define OUSTER_TOOLS__PTP_PMC_IMPL_H_

#include <errno.h>
#include <poll.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>

#include <ouster_tools/ptp/err.hpp>

extern "C"
{
#include <linuxptp/fsm.h>
#include <linuxptp/pmc_common.h>
#include <linuxptp/print.h>
}

#define P41 ((double)(1ULL << 41))

using namespace std::chrono_literals;

//-----------------------------
// Impl interface
//-----------------------------

namespace ouster_tools::ptp
{
  class pmc::Impl
  {
  public:
    Impl(int boundary_hops,
         const std::string& uds_prefix,
         int domain_number,
         int poll_timeout);
    ~Impl();

    std::string do_command(const std::string& cmd);
    std::string to_json(struct ptp_message *msg);

  private:
    static char* text2str(struct PTPText *text);
    static char* bin2str(Octet *data, int len);

    struct ::config *cfg_;
    struct ::pmc *pmc_;
    int poll_tmo_;

  }; // end: class pmc::Impl

} // end: namespace ouster_tools::ptp

//-----------------------------
// Impl implementation
//-----------------------------

//
// As of now, we assume a local ptp4l daemon running and so we a priori connect
// via UDS. If we want to pull this out in a more generalized PTP library and
// not a utility library for working with Ouster LiDARs, this should be
// generalized.
//
ouster_tools::ptp::pmc::Impl::Impl(
  int boundary_hops, const std::string& uds_prefix, int domain_number,
  int poll_timeout)
  : poll_tmo_(poll_timeout)
{

  print_set_progname("ouster_tools_pmc");
  print_set_syslog(1);
  print_set_verbose(1);

  this->cfg_ = config_create();
  if (! this->cfg_)
    {
      throw ouster_tools::ptp::error_t(ouster_tools::ptp::LINUXPTP_CTOR_ERR);
    }

  if (config_set_int(this->cfg_, "network_transport", TRANS_UDS))
    {
      throw ouster_tools::ptp::error_t(ouster_tools::ptp::LINUXPTP_CONFIG_ERR);
    }

  if (config_set_int(this->cfg_, "domainNumber", domain_number))
    {
      throw ouster_tools::ptp::error_t(ouster_tools::ptp::LINUXPTP_CONFIG_ERR);
    }

  std::string iface_name(uds_prefix);
  std::stringstream iface_stream;
  iface_stream << iface_name << static_cast<int>(getpid());

  this->pmc_ =
    pmc_create(this->cfg_,
               TRANS_UDS,
               iface_stream.str().c_str(),
               boundary_hops,
               domain_number,
               0,  // transport_specific
               0); // zero_datalen

  if (! this->pmc_)
    {
      throw ouster_tools::ptp::error_t(ouster_tools::ptp::LINUXPTP_CTOR_ERR);
    }

}

ouster_tools::ptp::pmc::Impl::~Impl()
{
  pmc_destroy(this->pmc_);
  config_destroy(this->cfg_);
}

std::string
ouster_tools::ptp::pmc::Impl::do_command(const std::string& cmd)
{
  std::stringstream json_str;
  // stamping here b/c it makes the code cleaner. this will present a stamp
  // that is slightly before the command was actually sent, but, will be a very
  // close approximation.
  auto ts = std::chrono::system_clock::now() + 0ns;
  json_str << "{\"send_stamp_ns\":\""
           << ts.time_since_epoch().count()
           << "\",\"ptp_msgs\":[";

  struct ptp_message *msg;
  struct pollfd pollfd[1];
  pollfd[0].fd = pmc_get_transport_fd(this->pmc_);

  bool do_cmd = true;
  bool is_first = true;

  while (true)
    {
      pollfd[0].events = POLLIN | POLLPRI;

      if (do_cmd)
        {
          pollfd[0].events |= POLLOUT;
        }

      int cnt = ::poll(pollfd, 1, this->poll_tmo_);

      if (cnt < 0)
        {
          if (EINTR == errno)
            {
              continue;
            }
          else
            {
              throw ouster_tools::ptp::error_t(
                      ouster_tools::ptp::LINUXPTP_POLL_ERR);
            }
        }
      else if (! cnt)
        {
          // poll timeout (nothing left to read)
          break;
        }

      if (pollfd[0].revents & POLLOUT)
        {
          if (pmc_do_command(this->pmc_, const_cast<char*>(cmd.c_str())))
            {
              throw ouster_tools::ptp::error_t(
                      ouster_tools::ptp::LINUXPTP_CMDEXE_ERR);
            }

          // flag that we no longer need to write our command
          do_cmd = false;
        }

      if (pollfd[0].revents & (POLLIN | POLLPRI))
        {
          msg = pmc_recv(this->pmc_);
          if (msg)
            {
              if (is_first)
                {
                  is_first = false;
                }
              else
                {
                  json_str << ",";
                }

              auto tp = std::chrono::system_clock::now() + 0ns;
              json_str << "{\"recv_stamp_ns\":\""
                       << tp.time_since_epoch().count()
                       << "\",";
              json_str << this->to_json(msg);
              msg_put(msg); // decrement msg reference count

              json_str << "}";
            }
        }
    }

  msg_cleanup();
  json_str << "]}";
  return json_str.str();
}

char*
ouster_tools::ptp::pmc::Impl::text2str(struct PTPText *text)
{
  static struct static_ptp_text s;
  s.max_symbols = -1;
  static_ptp_text_copy(&s, text);
  return (char*)(s.text);
}

char*
ouster_tools::ptp::pmc::Impl::bin2str(Octet *data, int len)
{
  static char buf[BIN_BUF_SIZE];
  return bin2str_impl(data, len, buf, sizeof(buf));
}

std::string
ouster_tools::ptp::pmc::Impl::to_json(struct ptp_message *msg)
{
  std::stringstream json_str;
  //json_str << "{";

  int action;
  struct TLV *tlv;
  struct management_tlv *mgt;
  //struct management_tlv_datum *mtd;
  struct defaultDS *dds;
  struct currentDS *cds;
  struct parentDS *pds;
  struct timePropertiesDS *tp;
  struct time_status_np *tsn;
  //struct grandmaster_settings_np *gsn;
  struct mgmt_clock_description *cd;
  struct tlv_extra *extra;
  struct portDS *p;
  //struct port_ds_np *pnp;
  //struct port_properties_np *ppn;
  //struct port_stats_np *pcp;

  if (msg_type(msg) != MANAGEMENT)
    {
      return std::string("{}");
    }
  action = management_action(msg);
  if (action < GET || action > ACKNOWLEDGE)
    {
      return std::string("{}");
    }

  json_str << "\"ptp_header\":{"
           << "\"sourcePortIdentity\":\""
           << pid2str(&msg->header.sourcePortIdentity) << "\","
           << "\"sequenceId\":\""
           << msg->header.sequenceId << "\","
           << "\"action\":\""
           << pmc_action_string(action) << "\""
           << "}";

  if (msg_tlv_count(msg) != 1)
    {
      goto out;
    }

  extra = TAILQ_FIRST(&msg->tlv_list);
  tlv = (struct TLV *) msg->management.suffix;
  if (tlv->type != TLV_MANAGEMENT)
    {
      goto out;
    }

  mgt = (struct management_tlv *) msg->management.suffix;
  if (mgt->length == 2 && mgt->id != TLV_NULL_MANAGEMENT)
    {
      goto out;
    }

  switch (mgt->id)
    {
    case TLV_CLOCK_DESCRIPTION:
      cd = &extra->cd;
      json_str << ",\"clock_description\":{"
               << "\"clockType\": \""
               << std::hex << std::showbase << align16(cd->clockType) << "\","
               << "\"physicalLayerProtocol\":\""
               << text2str(cd->physicalLayerProtocol) << "\","
               << "\"physicalAddress\":\""
               << bin2str(cd->physicalAddress->address,
                          align16(&cd->physicalAddress->length)) << "\","
               << "\"protocolAddress\":\""
               << std::dec << std::noshowbase
               << align16(&cd->protocolAddress->networkProtocol)
               << " " << portaddr2str(cd->protocolAddress) << "\","
               << "\"manufacturerId\":\""
               << bin2str(cd->manufacturerIdentity, OUI_LEN) << "\","
               << "\"productDescription\":\""
               << text2str(cd->productDescription) << "\","
               << "\"revisionData\":\""
               << text2str(cd->revisionData) << "\","
               << "\"userDescription\":\""
               << text2str(cd->userDescription) << "\","
               << "\"profileId\":\""
               << bin2str(cd->profileIdentity, PROFILE_ID_LEN) << "\""
               << "}";
      break;

    case TLV_USER_DESCRIPTION:
      json_str << ",\"user_description\":{"
               << "\"userDescription\":\""
               << text2str(extra->cd.userDescription) << "\""
               << "}";
      break;

    case TLV_DEFAULT_DATA_SET:
      dds = (struct defaultDS *) mgt->data;
      json_str << ",\"default_data_set\":{"
               << "\"twoStepFlag\":\""
               << (dds->flags & DDS_TWO_STEP_FLAG ? 1 : 0) << "\","
               << "\"slaveOnly\":\""
               << (dds->flags & DDS_SLAVE_ONLY ? 1 : 0) << "\","
               << "\"numberPorts\":\""  << dds->numberPorts << "\","
               << "\"priority1\":\""
               << static_cast<int>(dds->priority1) << "\","
               << "\"clockClass\":\""
               << static_cast<int>(dds->clockQuality.clockClass) << "\","
               << std::hex << std::showbase
               << "\"clockAccuracy\":\""
               << static_cast<int>(dds->clockQuality.clockAccuracy) << "\","
               << "\"offsetScaledLogVariance\":\""
               << dds->clockQuality.offsetScaledLogVariance << "\","
               << std::dec << std::noshowbase
               << "\"priority2\":\""
               << static_cast<int>(dds->priority2) << "\","
               << "\"clockIdentity\":\""
               << cid2str(&dds->clockIdentity) << "\","
               << "\"domainNumber\":\""
               << static_cast<int>(dds->domainNumber) << "\""
               << "}";
      break;

    case TLV_CURRENT_DATA_SET:
      cds = (struct currentDS *) mgt->data;
      json_str << ",\"current_data_set\":{"
               << "\"stepsRemoved\":\""
               << cds->stepsRemoved << "\","
               << "\"offsetFromMaster\":\""
               << std::fixed << std::setprecision(1)
               << cds->offsetFromMaster / 65536.0f << "\","
               << "\"meanPathDelay\":\""
               << std::fixed << std::setprecision(1)
               << cds->meanPathDelay / 65536.0f << "\""
               << "}";
      break;

    case TLV_PARENT_DATA_SET:
      pds = (struct parentDS *) mgt->data;
      json_str << ",\"parent_data_set\":{"
               << "\"parentPortIdentity\":\""
               << pid2str(&pds->parentPortIdentity) << "\","
               << "\"parentStats\":\""
               << static_cast<int>(pds->parentStats) << "\","
               << std::hex << std::showbase
               << "\"observedParentOffsetScaledLogVariance\":\""
               << pds->observedParentOffsetScaledLogVariance << "\","
               << "\"observedParentClockPhaseChangeRate\":\""
               << pds->observedParentClockPhaseChangeRate << "\","
               << std::dec << std::noshowbase
               << "\"grandmasterPriority1\":\""
               << static_cast<int>(pds->grandmasterPriority1) << "\","
               << "\"gm.ClockClass\":\""
               << static_cast<int>(pds->grandmasterClockQuality.clockClass)
               << "\","
               << std::hex << std::showbase
               << "\"gm.ClockAccuracy\":\""
               << static_cast<int>(pds->grandmasterClockQuality.clockAccuracy)
               << "\","
               << "\"gm.OffsetScaledLogVariance\":\""
               << pds->grandmasterClockQuality.offsetScaledLogVariance << "\","
               << std::dec << std::noshowbase
               << "\"grandmasterPriority2\":\""
               << static_cast<int>(pds->grandmasterPriority2) << "\","
               << "\"grandmasterIdentity\":\""
               << cid2str(&pds->grandmasterIdentity) << "\""
               << "}";
      break;

    case TLV_TIME_PROPERTIES_DATA_SET:
      tp = (struct timePropertiesDS *) mgt->data;
      json_str << ",\"time_properties_data_set\":{"
               << "\"currentUtcOffset\":\""
               << static_cast<int>(tp->currentUtcOffset) << "\","
               << "\"leap61\":\"" << (tp->flags & LEAP_61 ? 1 : 0) << "\","
               << "\"leap59\":\"" << (tp->flags & LEAP_59 ? 1 : 0) << "\","
               << "\"currentUtcOffsetValid\":\""
               << (tp->flags & UTC_OFF_VALID ? 1 : 0) << "\","
               << "\"ptpTimescale\":\""
               << (tp->flags & PTP_TIMESCALE ? 1 : 0) << "\","
               << "\"timeTraceable\":\""
               << (tp->flags & TIME_TRACEABLE ? 1 : 0) << "\","
               << "\"frequencyTraceable\":\""
               << (tp->flags & FREQ_TRACEABLE ? 1 : 0) << "\","
               << "\"timeSource\":\""
               << std::hex << std::showbase
               << static_cast<int>(tp->timeSource) << "\""
               << "}";
      break;

    case TLV_PRIORITY1:
      break;

    case TLV_PRIORITY2:
      break;

    case TLV_DOMAIN:
      break;

    case TLV_SLAVE_ONLY:
      break;

    case TLV_CLOCK_ACCURACY:
      break;

    case TLV_TRACEABILITY_PROPERTIES:
      break;

    case TLV_TIMESCALE_PROPERTIES:
      break;

    case TLV_TIME_STATUS_NP:
      tsn = (struct time_status_np *) mgt->data;
      json_str << ",\"time_status_np\":{"
               << "\"master_offset\":\""
               << static_cast<std::int64_t>(tsn->master_offset) << "\","
               << "\"ingress_time\":\""
               << static_cast<std::int64_t>(tsn->ingress_time) << "\","
               << "\"cumulativeScaledRateOffset\":\""
               << std::showpos << std::fixed << std::setprecision(9)
               << (tsn->cumulativeScaledRateOffset + 0.0) / P41 << "\","
               << std::noshowpos
               << "\"scaledLastGmPhaseChange\":\""
               << tsn->scaledLastGmPhaseChange << "\","
               << "\"gmTimeBaseIndicator\":\""
               << static_cast<int>(tsn->gmTimeBaseIndicator) << "\","
               << "\"lastGmPhaseChange\":\"0x"
               << std::noshowbase << std::hex
               << std::setfill('0') << std::setw(4)
               << static_cast<int>(tsn->lastGmPhaseChange.nanoseconds_msb)
               << "'" << std::setw(16)
               << static_cast<std::int64_t>(
                    tsn->lastGmPhaseChange.nanoseconds_lsb) << "."
               << std::setw(4)
               << static_cast<int>(
                    tsn->lastGmPhaseChange.fractional_nanoseconds)
               << "\","
               << "\"gmPresent\":\"" << (tsn->gmPresent ? "true" : "false")
               << "\","
               << "\"gmIdentity\":\"" << cid2str(&tsn->gmIdentity) << "\""
               << "}";
      break;

    case TLV_GRANDMASTER_SETTINGS_NP:
      break;

    case TLV_PORT_DATA_SET:
      p = (struct portDS *) mgt->data;
      if (p->portState > PS_SLAVE)
        {
          p->portState = 0;
        }
      json_str << ",\"port_data_set\":{"
               << "\"portIdentity\":\""
               << pid2str(&p->portIdentity) << "\","
               << "\"portState\":\""
               << ps_str[p->portState] << "\","
               << "\"logMinDelayReqInterval\":\""
               << static_cast<int>(p->logMinDelayReqInterval) << "\","
               << "\"peerMeanPathDelay\":\""
               << static_cast<std::int64_t>(p->peerMeanPathDelay >> 16) << "\","
               << "\"logAnnounceInterval\":\""
               << static_cast<int>(p->logAnnounceInterval) << "\","
               << "\"announceReceiptTimeout\":\""
               << static_cast<int>(p->announceReceiptTimeout) << "\","
               << "\"logSyncInterval\":\""
               << static_cast<int>(p->logSyncInterval) << "\","
               << "\"delayMechanism\":\""
               << static_cast<int>(p->delayMechanism) << "\","
               << "\"logMinPdelayReqInterval\":\""
               << static_cast<int>(p->logMinPdelayReqInterval) << "\","
               << "\"versionNumber\":\""
               << static_cast<int>(p->versionNumber) << "\""
               << "}";
      break;

    case TLV_PORT_DATA_SET_NP:
      break;

    case TLV_PORT_PROPERTIES_NP:
      break;

    case TLV_PORT_STATS_NP:
      break;

    case TLV_LOG_ANNOUNCE_INTERVAL:
      break;

    case TLV_ANNOUNCE_RECEIPT_TIMEOUT:
      break;

    case TLV_LOG_SYNC_INTERVAL:
      break;

    case TLV_VERSION_NUMBER:
      break;

    case TLV_DELAY_MECHANISM:
      break;

    case TLV_LOG_MIN_PDELAY_REQ_INTERVAL:
      break;
    }

out:
  //json_str << "}";
  return json_str.str();
}

#endif // OUSTER_TOOLS__PTP_PMC_IMPL_H_
