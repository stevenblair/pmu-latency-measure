#include <platform.h>
#include <print.h>
#include <xccompat.h>
#include <string.h>
#include <xscope.h>
#include "debug_print.h"
//#include "media_fifo.h"
#include "ethernet_board_support.h"
//#include "simple_demo_controller.h"
//#include "avb_ethernet.h"
#include "xccompat.h"
#include "otp_board_info.h"
#include "ethernet.h"
#include "mii.h"


on tile[0]: out port p_led = XS1_PORT_4A;

on tile[0]: otp_ports_t otp_ports0 = OTP_PORTS_INITIALIZER;
on ETHERNET_DEFAULT_TILE: otp_ports_t otp_ports1 = OTP_PORTS_INITIALIZER;

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

//on tile[0]: out port p_leds = XS1_PORT_4F;
//
//#if AVB_DEMO_ENABLE_LISTENER
//on tile[0]: out buffered port:32 p_aud_dout[AVB_DEMO_NUM_CHANNELS/2] = PORT_SDATA_OUT;
//#else
//  #define p_aud_dout null
//#endif
//
//#if AVB_DEMO_ENABLE_TALKER
//on tile[0]: in buffered port:32 p_aud_din[AVB_DEMO_NUM_CHANNELS/2] = PORT_SDATA_IN;
//#else
//  #define p_aud_din null
//#endif
//
//#if AVB_XA_SK_AUDIO_PLL_SLICE
//on tile[0]: out port p_audio_shared = PORT_AUDIO_SHARED;
//#endif
//
//#if AVB_DEMO_ENABLE_LISTENER
//media_output_fifo_data_t ofifo_data[AVB_NUM_MEDIA_OUTPUTS];
//media_output_fifo_t ofifos[AVB_NUM_MEDIA_OUTPUTS];
//#else
//  #define ofifos null
//#endif
//
//#if AVB_DEMO_ENABLE_TALKER
//media_input_fifo_data_t ififo_data[AVB_NUM_MEDIA_INPUTS];
//media_input_fifo_t ififos[AVB_NUM_MEDIA_INPUTS];
//#else
//  #define ififos null
//#endif


enum mac_rx_chans {
  MAC_RX_TO_MEDIA_CLOCK = 0,
#if AVB_DEMO_ENABLE_LISTENER
  MAC_RX_TO_LISTENER,
#endif
  MAC_RX_TO_SRP,
  MAC_RX_TO_1722_1,
  NUM_MAC_RX_CHANS
};

enum mac_tx_chans {
  MAC_TX_TO_MEDIA_CLOCK = 0,
#if AVB_DEMO_ENABLE_TALKER
  MAC_TX_TO_TALKER,
#endif
  MAC_TX_TO_SRP,
  MAC_TX_TO_1722_1,
  MAC_TX_TO_AVB_MANAGER,
  NUM_MAC_TX_CHANS
};


void LED_task() {
    timer t;
    unsigned int time;
    int LED_state = 0;

    for (int i = 0; i < 100; i++) {
        select {
            case t when timerafter(time) :> void:
                time += XS1_TIMER_MHZ * 1000 * 1000;
                if (LED_state == 1) {
                    LED_state = 0;
                    p_led <: 0x0E;
                }
                else {
                    LED_state = 1;
                    p_led <: 0x0F;
                }
                break;
        }
    }
}


int main(void) {
  // Ethernet channels
  chan c_mac_tx[NUM_MAC_TX_CHANS];
  chan c_mac_rx[NUM_MAC_RX_CHANS];

  par {
    on tile[0]: LED_task();
    on ETHERNET_DEFAULT_TILE: {
      char mac_address[6];
      otp_board_info_get_mac(otp_ports1, 0, mac_address);
      smi_init(smi1);
      eth_phy_config(1, smi1);
      ethernet_server_full_two_port(mii1,
                                    mii2,
                                    smi1,
                                    null,
                                    mac_address,
                                    c_mac_rx, NUM_MAC_RX_CHANS,
                                    c_mac_tx, NUM_MAC_TX_CHANS);
    }
  }

  return 0;
}
