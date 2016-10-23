// Copyright (c) 2012, XMOS Ltd., All rights reserved
// This software is freely distributable under a derivative of the
// University of Illinois/NCSA Open Source License posted in
// LICENSE.txt and at <http://github.xcore.com/>

#include "otp_board_info.h"
#include <xs1.h>
#include <xclib.h>

/// Size of the OTP in words.
#define OTP_SIZE 0x800

/// OTP control signals.
enum {
  OTP_CTRL_READ = 1 << 0,
  OTP_CTRL_STATUS = 1 << 5,
  OTP_CTRL_RESET_M = 1 << 13
};

typedef struct board_info_header_t {
  unsigned address;
  unsigned bitmap;
} board_info_header_t;

/// Read a word from the specified address in the OTP.
static unsigned otp_read_word(otp_ports_t &ports, unsigned address)
{
  unsigned value;
  ports.addr <: address;

  // If the application booted from OTP the bootloader may have left
  // differential mode enabled. Reset the mode registers to default settings.
  // There is no need to do this on every read - we could do it just once at the
  // start. However it is good for code size to do this at the same time as the
  // read from OTP since the ports will be in registers for the OTP read.
  ports.ctrl <: OTP_CTRL_RESET_M;
  ports.ctrl <: 0;

  // Start the read command.
  ports.ctrl <: OTP_CTRL_READ;
  // Wait for status to go high. Use peek otherwise the value of the control
  // signals we are driving will become undefined.
  do {
    value = peek(ports.ctrl);
  } while ((value & OTP_CTRL_STATUS) == 0);
  ports.ctrl <: 0;
  // Grab the data.
  ports.data :> value;

  return value;
}

/// Search the end of the OTP for a valid board info header.
static int otp_board_info_get_header(otp_ports_t &ports,
                                     board_info_header_t &info)
{
  int address = OTP_SIZE - 1;
  do {
    unsigned bitmap = otp_read_word(ports, address);
    unsigned length;
    // Stop if bitmap has not been written.
    if (bitmap >> 31)
      return 0;
    // If bitmap is valid we are done.
    if (bitmap >> 30) {
      info.address = address;
      info.bitmap = bitmap;
      return 1;
    }
    // Otherwise skip this bitmap and continue searching.
    length = (bitmap >> 25) & 0x1f;
    if (length == 0) {
      // Bailout on invalid length to avoid infinite loop.
      return 0;
    }
    address -= length;
  } while (address >= 0);
  // Got to the start of the OTP without finding a header.
  return 0;
}

static unsigned otp_board_info_get_num_macs(const board_info_header_t &info)
{
  return (info.bitmap >> 22) & 0x7;
}

int otp_board_info_get_mac(otp_ports_t &ports, unsigned i, char mac[6])
{
  unsigned address;
  unsigned macaddr[2];
  board_info_header_t info;
  if (!otp_board_info_get_header(ports, info))
    return 0;
  if (i >= otp_board_info_get_num_macs(info))
    return 0;
  address = info.address - (2 + 2 * i);
  macaddr[0] = byterev(otp_read_word(ports, address + 1));
  macaddr[1] = byterev(otp_read_word(ports, address));
  // Assumes little endian byte order.
  for (unsigned i = 0; i < 6; i++) {
    mac[i] = (macaddr, char[])[i + 2];
  }
  return 1;
}

static int otp_board_info_has_serial(const board_info_header_t &info)
{
  return (info.bitmap >> 21) & 1;
}

int
otp_board_info_get_serial(otp_ports_t &ports, unsigned &value)
{
  unsigned address;
  board_info_header_t info;
  if (!otp_board_info_get_header(ports, info))
    return 0;
  if (!otp_board_info_has_serial(info))
    return 0;
  address = info.address - (otp_board_info_get_num_macs(info) * 2 + 1);
  value = otp_read_word(ports, address);
  return 1;
}
