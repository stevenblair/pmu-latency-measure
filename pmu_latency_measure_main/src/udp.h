///**
// * IEC 61850 Ethernet to UDP tunnel
// *
// * Copyright (c) 2012 Steven Blair
// *
// * This program is free software; you can redistribute it and/or
// * modify it under the terms of the GNU General Public License
// * as published by the Free Software Foundation; either version 2
// * of the License, or (at your option) any later version.
//
// * This program is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// * GNU General Public License for more details.
//
// * You should have received a copy of the GNU General Public License
// * along with this program; if not, write to the Free Software
// * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
// */
//
////#pragma once
//
//#ifndef __UDP_H__
//#define __UDP_H__
//
//
//
////extern "C" {
//
////#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
////extern "C" {
////#endif
//
//#include <inttypes.h>
//#include <stdlib.h>
//#include <string.h>
////#include "rtwtypes.h"
////#include "TcEthernetSampleInterfaces.h"
//
//#define LITTLE_ENDIAN		1
//
//#define UDP_HEADER_BYTES    8
//#define IP_HEADER_BYTES     20
//#define ETH_HEADER_BYTES    14
//
//#define UDP_DEFAULT_PORT    4713
//
//#define IP_DEFAULT_TTL      100
//#define IP_VERSION          0x4
//#define IP_IHL              0x5
//#define IP_PROTOCOL_UDP     0x11
//
//extern uint8_t IP_SOURCE[4];
//extern uint8_t IP_DEST[4];
//
//extern uint8_t ETH_SOURCE[6];
//extern uint8_t ETH_DEST[6];
//
//typedef struct eth_header {
//    uint8_t dest[6];
//    uint8_t source[6];
//    uint16_t ethertype;
//} ETH;
//
//typedef struct ip_header {
//    uint16_t info;
//    uint16_t length;
//    uint16_t id;
//    uint16_t flags_frag;
//    uint8_t ttl;
//    uint8_t protocol;
//    uint16_t checksum;
//    uint32_t source;
//    uint32_t dest;
//    ETH eth;
//} IP;
//
//typedef struct udp_header {
//    uint16_t source;
//    uint16_t dest;
//    uint16_t length;
//    uint16_t checksum;
//    IP ip;
//} UDP;
//
//
//// a simple memcpy implementation, that reverses endian-ness
//void reversememcpy(unsigned char *dst, const unsigned char *src, unsigned int len);
//
//// copies bytes to network format (big-endian)
//void netmemcpy(unsigned char *dst, const unsigned char *src, unsigned int len);
//
//int encodeETH(unsigned char* data, ETH* eth, const char *payload, int payload_length);
//
//uint16_t getIPChecksum(IP *ip);
//
//int encodeIP(unsigned char* data, IP* ip, const char *payload, int payload_length);
//
//int encode_UDP(unsigned char* data, UDP* udp, const char *payload, int payload_length);
//
//void init_existing_UDP(UDP *udp, uint8_t *ip, uint8_t *mac);
//
//extern void set_UDP_dest(UDP *udp, uint8_t *ip, uint8_t *mac, uint16_t remote_port);
//
////};
//
//#endif
//
////#endif
