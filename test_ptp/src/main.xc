#include <xs1.h>
#include <xclib.h>
#include <print.h>
#include <platform.h>
#include <stdlib.h>
#include "otp_board_info.h"
#include "ethernet.h"
#include "ethernet_board_support.h"
#include <xscope.h>
//#include "gptp.h"
#include "ethernet_conf.h"
#include "mac_custom_filter.h"
#include "debug_print.h"

//void xscope_user_init(void) {
//
//#if 0
//  xscope_register(3, XSCOPE_CONTINUOUS, "local_egress_ts", XSCOPE_UINT, "Value",
//    XSCOPE_CONTINUOUS, "received_sync_ts", XSCOPE_INT, "Value",
//    XSCOPE_CONTINUOUS, "residence", XSCOPE_INT, "Value");
///*
//  xscope_register(4, XSCOPE_CONTINUOUS, "rdptr", XSCOPE_UINT, "Value",
//    XSCOPE_CONTINUOUS, "wrptr", XSCOPE_UINT, "Value",
//    XSCOPE_CONTINUOUS, "hdr", XSCOPE_UINT, "Value",
//    XSCOPE_CONTINUOUS, "hdr->next", XSCOPE_INT, "Value");
//*/
///*
//  xscope_register(2, XSCOPE_CONTINUOUS, "commit", XSCOPE_UINT, "Value",
//    XSCOPE_CONTINUOUS, "buf", XSCOPE_INT, "Value");
//*/
//    // XSCOPE_CONTINUOUS, "buf", XSCOPE_INT, "Value");
//    // XSCOPE_CONTINUOUS, "fwdbuf", XSCOPE_INT, "Value");
//#else
//  xscope_register(0);
//#endif
//  xscope_config_io(XSCOPE_IO_BASIC);
//}

on ETHERNET_DEFAULT_TILE: otp_ports_t otp_ports = OTP_PORTS_INITIALIZER;

smi_interface_t smi1 = ETHERNET_DEFAULT_SMI_INIT;

// Circle slot
mii_interface_t mii1 = ETHERNET_DEFAULT_MII_INIT;

// Square slot
on tile[1]: mii_interface_t mii2 = {
  XS1_CLKBLK_3,
  XS1_CLKBLK_4,
  XS1_PORT_1B,
  XS1_PORT_4D,
  XS1_PORT_4A,
  XS1_PORT_1C,
  XS1_PORT_1G,
  XS1_PORT_1F,
  XS1_PORT_4B
};

// PTP sync port
//on stdcore[0]: port ptp_sync_port = XS1_PORT_1C;//XS1_PORT_4A;




#define NORMAL_PORT     0   // rx on this port is tx'd normally
#define DELAYED_PORT    1   // rx on this port is tx'd with delay


#define PTP_PERIODIC_TIME (50000)




int in_byte_counter = 0;
int out_byte_counter = 0;
//#define MAX_DELAY_MESG_LENGTH   (1000 + (8*8))        // TODO define max packet size
#define MAX_DELAY_MESG_LENGTH   1024        // TODO define max packet size
#define MAX_BUF_LENGTH  5

typedef struct delay_buf {
    unsigned int buf[MAX_DELAY_MESG_LENGTH / 4];
//    unsigned char buf[MAX_DELAY_MESG_LENGTH];
    unsigned len;
    unsigned rx_ts;
    unsigned src_port;
    unsigned to_send;
} delay_buf_t;

delay_buf_t delay_buffer[MAX_BUF_LENGTH];

unsigned int next_free_buf = 0;
unsigned int start_replay = 0;
//unsigned int next_emptiable_buf = -1;


#pragma select handler
void delay_recv_and_process_packet(chanend c_rx, chanend c_tx) {
//    unsigned ts;
//    unsigned src_port;
//    unsigned len;
    //  unsigned int buf[MAX_DELAY_MESG_LENGTH / 4];

//    if (next_free_buf == next_emptiable_buf) {
//        debug_printf("buffer overflow\n");
//    }
//    else {
      safe_mac_rx_timed(c_rx,
    //          (buf, unsigned char[]),
                      (delay_buffer[next_free_buf].buf, unsigned char[]),
                      delay_buffer[next_free_buf].len,
                      delay_buffer[next_free_buf].rx_ts,
                      delay_buffer[next_free_buf].src_port,
                       MAX_DELAY_MESG_LENGTH);

      delay_buffer[next_free_buf].to_send = 1;

//      debug_printf("next_free_buf %d\n", next_free_buf);
      in_byte_counter += delay_buffer[next_free_buf].len;
//      xscope_int(IN_BYTE_COUNTER, in_byte_counter);
//    }

//  debug_printf("RX %d bytes, port %d; total %d bytes\n", delay_buffer[next_free_buf].len, delay_buffer[next_free_buf].src_port, in_byte_counter);


      next_free_buf++;
      if (next_free_buf >= MAX_BUF_LENGTH) {
          next_free_buf = 0;
          start_replay = 1;
      }


//  // TODO do something with buf contents
//  if (src_port == NORMAL_PORT) {
//      unsigned int sentTime;
//      mac_tx_timed(c_tx, buf, len, sentTime, DELAYED_PORT);    // TODO check ptp_tx_timed() implementation
//
//      // TODO do something with sentTime and ts
////      debug_printf("  TX %d bytes, port %d, t %d us\n", len, DELAYED_PORT, (sentTime - ts) / 10);
////      debug_printf("arrived on port %d, sent on port %d, TX %d bytes, t %d us\n", src_port, DELAYED_PORT, len, (sentTime - ts) / 10);
//
//      int duration_us = (sentTime - ts) / 10;
//      if (duration_us > 500) {
//          debug_printf("arrived on port %d, sent on port %d, TX %d bytes, t %d us\n", src_port, DELAYED_PORT, len, duration_us);
//      }
//  }
//  else if (src_port == DELAYED_PORT) {
//      unsigned int sentTime;
//      mac_tx_timed(c_tx, buf, len, sentTime, NORMAL_PORT);    // TODO check ptp_tx_timed() implementation
//
//      // TODO do something with sentTime and ts
////      debug_printf("  TX %d bytes, port %d, t %d us\n", len, NORMAL_PORT, (sentTime - ts) / 10);
////      debug_printf("arrived on port %d, sent on port %d, TX %d bytes, t %d us\n", src_port, NORMAL_PORT, len, (sentTime - ts) / 10);
//
//      int duration_us = (sentTime - ts) / 10;
//      if (duration_us > 500) {
//          debug_printf("arrived on port %d, sent on port %d, TX %d bytes, t %d us\n", src_port, NORMAL_PORT, len, duration_us);
//      }
//  }


  if (start_replay == 1) {
          unsigned int buf_to_forward = next_free_buf;//(next_free_buf + (MAX_BUF_LENGTH - 2)) % MAX_BUF_LENGTH;
      //    debug_printf("next_free_buf %d, buf_to_forward %d\n", next_free_buf, buf_to_forward);

          if (delay_buffer[buf_to_forward].src_port == DELAYED_PORT) {
              if (delay_buffer[buf_to_forward].to_send == 1) {
                  unsigned int sentTime;
                  mac_tx_timed(c_tx, delay_buffer[buf_to_forward].buf, delay_buffer[buf_to_forward].len, sentTime, NORMAL_PORT);    // TODO check ptp_tx_timed() implementation
                  delay_buffer[buf_to_forward].to_send = 0;

                  // TODO do something with sentTime and ts
                  //      debug_printf("  TX %d bytes, port %d, t %d us\n", len, NORMAL_PORT, (sentTime - ts) / 10);
                  //      debug_printf("arrived on port %d, sent on port %d, TX %d bytes, t %d us\n", src_port, NORMAL_PORT, len, (sentTime - ts) / 10);

                  out_byte_counter += delay_buffer[buf_to_forward].len;
      //            xscope_int(OUT_BYTE_COUNTER, out_byte_counter);


                  int duration_us = (sentTime - delay_buffer[buf_to_forward].rx_ts) / 100;
                  xscope_int(FRAME_DELAY, duration_us);
          //            if (duration_us > 500) {
      //                debug_printf("arrived on port %d, sent on port %d, TX %d bytes, t %d us\n", delay_buffer[buf_to_forward].src_port, NORMAL_PORT, delay_buffer[buf_to_forward].len, duration_us);
          //            }
              }
              else {
                  debug_printf("found buffer already sent\n");
              }
          }
          else {
              debug_printf("unexpected source port %d\n", delay_buffer[buf_to_forward].src_port);
          }
      }



//  ptp_recv(c_tx, (buf, unsigned char[]), ts, src_port, len);
}


void periodic(chanend c_tx) {
    if (start_replay == 1) {
        unsigned int buf_to_forward = next_free_buf;//(next_free_buf + (MAX_BUF_LENGTH - 2)) % MAX_BUF_LENGTH;
    //    debug_printf("next_free_buf %d, buf_to_forward %d\n", next_free_buf, buf_to_forward);

        if (delay_buffer[buf_to_forward].src_port == DELAYED_PORT) {
            if (delay_buffer[buf_to_forward].to_send == 1) {
                unsigned int sentTime;
                mac_tx_timed(c_tx, delay_buffer[buf_to_forward].buf, delay_buffer[buf_to_forward].len, sentTime, NORMAL_PORT);    // TODO check ptp_tx_timed() implementation
                delay_buffer[buf_to_forward].to_send = 0;

                // TODO do something with sentTime and ts
                //      debug_printf("  TX %d bytes, port %d, t %d us\n", len, NORMAL_PORT, (sentTime - ts) / 10);
                //      debug_printf("arrived on port %d, sent on port %d, TX %d bytes, t %d us\n", src_port, NORMAL_PORT, len, (sentTime - ts) / 10);

                out_byte_counter += delay_buffer[buf_to_forward].len;
    //            xscope_int(OUT_BYTE_COUNTER, out_byte_counter);


                int duration_us = (sentTime - delay_buffer[buf_to_forward].rx_ts) / 100;
                xscope_int(FRAME_DELAY, duration_us);
        //            if (duration_us > 500) {
    //                debug_printf("arrived on port %d, sent on port %d, TX %d bytes, t %d us\n", delay_buffer[buf_to_forward].src_port, NORMAL_PORT, delay_buffer[buf_to_forward].len, duration_us);
        //            }
            }
        }
        else {
            debug_printf("unexpected source port %d\n", delay_buffer[buf_to_forward].src_port);
        }
    }
}



void delay_server(chanend c_rx, chanend c_tx) {
  timer ptp_timer;
  int ptp_timeout;

//  mac_set_custom_filter(c_rx, MAC_FILTER_PTP);
  mac_set_custom_filter(c_rx, 0xFFFFFFFF);
  ptp_timer :> ptp_timeout;

  while (1) {
    [[ordered]]
    select {
//        case ptp_timer when timerafter(ptp_timeout) :> void:
//            periodic(c_tx);
//    //            ptp_periodic(c_tx, ptp_timeout);
//            ptp_timeout += PTP_PERIODIC_TIME;
//            break;
        case delay_recv_and_process_packet(c_rx, c_tx):
            break;
      }
  }
}


int main()
{
  chan c_mac_rx[1], c_mac_tx[1];
//  chan c_ptp[1];

  par
  {
    on ETHERNET_DEFAULT_TILE:
    {
      char mac_address[6];
      otp_board_info_get_mac(otp_ports, 0, mac_address);
      smi_init(smi1);
      eth_phy_config(1, smi1);
      ethernet_server_full_two_port(mii1,
                                    mii2,
                                    smi1,
                                    null,
                                    mac_address,
                                    c_mac_rx, 1,
                                    c_mac_tx, 1);
    }


    on stdcore[0]: delay_server(c_mac_rx[0], c_mac_tx[0]);





    // enable both these tasks for a PTP server:
//    on stdcore[0]: ptp_server(c_mac_rx[0],
//                              c_mac_tx[0],
//                              c_ptp,
//                              1,
//                              PTP_GRANDMASTER_CAPABLE);
//    on stdcore[0]: ptp_output_test_clock(c_ptp[0], ptp_sync_port, 100000000);
  }

  return 0;
}

