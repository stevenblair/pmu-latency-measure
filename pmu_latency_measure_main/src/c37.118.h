////#pragma once
//
//#ifndef C37118_H_
//#define C37118_H_
//
////#include <iostream>
////#include <list>
////#include <string>
////#include <unistd.h>
//#include "udp.h"
////#include "interface.h"
////#include "PMU_M_classInterfaces.h"
//
//#define MIN(a,b) ((a) < (b) ? (a) : (b))
//
//enum Message_Type { C37_118_Data, C37_118_CFG_1, C37_118_CFG_2, C37_118_CFG_3, C37_118_DATA_TRANSMISSION };
//
//enum Command_Type { C37_118_DATA_OFF = 1, C37_118_DATA_ON = 2 };
//
//extern uint8_t buf_payload[512];
//extern UDP udp;
//
//extern uint16_t ComputeCRC(unsigned char *buf, unsigned char len);
//
//extern uint16_t write_data_transmission_frame(unsigned char *buf, uint32_t SOC_recv, uint16_t transmission_state);
//
//extern uint16_t write_ethernet_frame_into_buf(unsigned char *buf, uint32_t SOC_recv, uint16_t transmission_state);
//
//
////extern "C" {
////};
//
//#endif
