// Copyright (c) 2012, XMOS Ltd., All rights reserved
// This software is freely distributable under a derivative of the
// University of Illinois/NCSA Open Source License posted in
// LICENSE.txt and at <http://github.xcore.com/>

/**
 * Functions for reading board information (serial number, MAC address).
 * from the OTP memory of an XCore. This information can be written to the
 * device using XBURN.
 */

#include <xccompat.h>

#ifndef _otp_board_info_h_
#define _otp_board_info_h_

/**
 * otp_ports_t structure - contains ports used to access the OTP memory.
 */
typedef struct otp_ports_t {
  port data;
#ifdef __XC__
  out port addr;
  out port ctrl;
#else
  port addr;
  port ctrl;
#endif
} otp_ports_t;

/**
 * Standard initializer for an otp_ports_t structure. Use as follows:
 * on stdcore[0]: otp_ports_t otp_ports = OTP_PORTS_INITIALIZER;
 */
#define OTP_PORTS_INITIALIZER \
{ \
  XS1_PORT_32B, \
  XS1_PORT_16C, \
  XS1_PORT_16D \
}

/**
 * Read a MAC address from the board information written at the end of the OTP
 * memory.
 * \param ports Ports used to access the OTP memory.
 * \param index Index of the MAC address to retrieve.
 * \param mac Array to write the MAC address to.
 * \return Returns 1 on success, 0 on failure.
 */
int otp_board_info_get_mac(REFERENCE_PARAM(otp_ports_t, ports), unsigned index,
                           char mac[6]);

/**
 * Read a serial number from the board information written at the end of the OTP
 * memory.
 * \param ports Ports used to access the OTP memory.
 * \param value Variable to store the serial number to.
 * \return Returns 1 on success, 0 on failure.
 */
int otp_board_info_get_serial(REFERENCE_PARAM(otp_ports_t, ports),
                              REFERENCE_PARAM(unsigned, value));

#endif /* _otp_board_info_h_ */
