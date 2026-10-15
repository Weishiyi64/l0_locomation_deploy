#pragma once

#include <linux/can/error.h>

#include <cerrno>
#include <cstring>
#include <sstream>
#include <string>

#include "socketcan_wrapper/socketcan_manager.h"  // defines can::Frame

namespace socketcan_wrapper {

inline std::string parseCanRxErrorFrame(const can::Frame& msg) {
  std::ostringstream oss;
  uint32_t error_mask = msg.id;

  bool has_error = false;

  // Parse error types from ID field (error mask)
  if (error_mask & CAN_ERR_TX_TIMEOUT)
  {
    oss << "TX_TIMEOUT; ";
    has_error = true;
  }

  if (error_mask & CAN_ERR_LOSTARB)
  {
    oss << "LOST_ARBITRATION";
    if (msg.data[0] != CAN_ERR_LOSTARB_UNSPEC)
    {
      oss << "(bit " << (int)msg.data[0] << ")";
    }
    has_error = true;
  }

  if (error_mask & CAN_ERR_CRTL)
  {
    oss << "CONTROLLER_ERROR(";
    uint8_t ctrl_status = msg.data[1];
    bool first = true;
    if (ctrl_status & CAN_ERR_CRTL_RX_OVERFLOW)
    {
      oss << (first ? "" : "|") << "RX_OVERFLOW";
      first = false;
    }
    if (ctrl_status & CAN_ERR_CRTL_TX_OVERFLOW)
    {
      oss << (first ? "" : "|") << "TX_OVERFLOW";
      first = false;
    }
    if (ctrl_status & CAN_ERR_CRTL_RX_WARNING)
    {
      oss << (first ? "" : "|") << "RX_WARNING";
      first = false;
    }
    if (ctrl_status & CAN_ERR_CRTL_TX_WARNING)
    {
      oss << (first ? "" : "|") << "TX_WARNING";
      first = false;
    }
    if (ctrl_status & CAN_ERR_CRTL_RX_PASSIVE)
    {
      oss << (first ? "" : "|") << "RX_PASSIVE";
      first = false;
    }
    if (ctrl_status & CAN_ERR_CRTL_TX_PASSIVE)
    {
      oss << (first ? "" : "|") << "TX_PASSIVE";
      first = false;
    }
    if (ctrl_status & CAN_ERR_CRTL_ACTIVE)
    {
      oss << (first ? "" : "|") << "ACTIVE";
      first = false;
    }
    if (first)
    {
      oss << "UNSPEC";
    }
    oss << "); ";
    has_error = true;
  }

  if (error_mask & CAN_ERR_PROT)
  {
    oss << "PROTOCOL_VIOLATION(";
    uint8_t prot_type = msg.data[2];
    uint8_t prot_loc = msg.data[3];
    bool first = true;
    if (prot_type & CAN_ERR_PROT_BIT)
    {
      oss << (first ? "" : "|") << "BIT_ERROR";
      first = false;
    }
    if (prot_type & CAN_ERR_PROT_FORM)
    {
      oss << (first ? "" : "|") << "FORMAT_ERROR";
      first = false;
    }
    if (prot_type & CAN_ERR_PROT_STUFF)
    {
      oss << (first ? "" : "|") << "STUFFING_ERROR";
      first = false;
    }
    if (prot_type & CAN_ERR_PROT_BIT0)
    {
      oss << (first ? "" : "|") << "CANNOT_SEND_DOMINANT";
      first = false;
    }
    if (prot_type & CAN_ERR_PROT_BIT1)
    {
      oss << (first ? "" : "|") << "CANNOT_SEND_RECESSIVE";
      first = false;
    }
    if (prot_type & CAN_ERR_PROT_OVERLOAD)
    {
      oss << (first ? "" : "|") << "BUS_OVERLOAD";
      first = false;
    }
    if (prot_type & CAN_ERR_PROT_ACTIVE)
    {
      oss << (first ? "" : "|") << "ACTIVE_ERROR";
      first = false;
    }
    if (prot_type & CAN_ERR_PROT_TX)
    {
      oss << (first ? "" : "|") << "TX_ERROR";
      first = false;
    }
    if (first)
    {
      oss << "UNSPEC";
    }
    oss << ", loc=0x" << std::hex << (int)prot_loc << std::dec << "); ";
    has_error = true;
  }

  if (error_mask & CAN_ERR_TRX)
  {
    oss << "TRANSCEIVER_ERROR(";
    uint8_t trx_status = msg.data[4];
    if (trx_status == CAN_ERR_TRX_UNSPEC)
    {
      oss << "UNSPEC";
    }
    else
    {
      oss << "0x" << std::hex << (int)trx_status << std::dec;
    }
    oss << "); ";
    has_error = true;
  }

  if (error_mask & CAN_ERR_ACK)
  {
    oss << "NO_ACK; ";
    has_error = true;
  }

  if (error_mask & CAN_ERR_BUSOFF)
  {
    oss << "BUS_OFF; ";
    has_error = true;
  }

  if (error_mask & CAN_ERR_BUSERROR)
  {
    oss << "BUS_ERROR; ";
    has_error = true;
  }

  if (error_mask & CAN_ERR_RESTARTED)
  {
    oss << "CONTROLLER_RESTARTED; ";
    has_error = true;
  }

  // Add error counters
  uint8_t tx_error_count = msg.data[6];
  uint8_t rx_error_count = msg.data[7];
  oss << "ErrorCounters: TX=" << (int)tx_error_count << " RX=" << (int)rx_error_count;

  if (!has_error)
  {
    oss << "UNKNOWN_ERROR";
  }

  return oss.str();
}

inline std::string parseCanTxError(int err) {
  std::ostringstream oss;

  if (err == 0)
  {
    oss << "errno=0 (Unknown error)";
  }
  else if (err == ENOBUFS)
  {  // 105: No buffer space available
    oss << "errno=" << err << " (ENOBUFS - Buffer full)";
  }
  else if (err == ENETDOWN)
  {  // 100: Network is down
    oss << "errno=" << err << " (ENETDOWN - Interface is down)";
  }
  else if (err == ENODEV)
  {  // 19: No such device
    oss << "errno=" << err << " (ENODEV - Device disconnected)";
  }
  else if (err == EAGAIN || err == EWOULDBLOCK)
  {  // 11: Resource temporarily unavailable
    oss << "errno=" << err << " (EAGAIN - Temporarily unavailable)";
  }
  else
  {
    oss << "errno=" << err << " (" << strerror(err) << ")";
  }

  return oss.str();
}

}  // namespace socketcan_wrapper
