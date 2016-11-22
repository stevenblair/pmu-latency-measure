#ifndef __ethernet_conf_h__
#define __ethernet_conf_h__

/******** ETHERNET MAC CONFIGURATION PARAMETERS *************************************************/
#define ETHERNET_DEFAULT_IMPLEMENTATION full

#define MAX_ETHERNET_PACKET_SIZE (1518)

#define NUM_MII_RX_BUF 6
#define NUM_MII_TX_BUF 3

#define ETHERNET_RX_HP_QUEUE 1
#define ETHERNET_TX_HP_QUEUE 1

#define MAX_ETHERNET_CLIENTS   4

#define ETHERNET_MAX_TX_HP_PACKET_SIZE (300)
#define ETHERNET_MAX_TX_LP_PACKET_SIZE (1518)

#define MII_RX_BUFSIZE_HIGH_PRIORITY (1100 + (3*(ETHERNET_MAX_TX_HP_PACKET_SIZE)))
#define MII_RX_BUFSIZE_LOW_PRIORITY (1518*3)

#define MII_TX_BUFSIZE_HIGH_PRIORITY (1100 + (3*(ETHERNET_MAX_TX_HP_PACKET_SIZE)))
#define MII_TX_BUFSIZE_LOW_PRIORITY (2000)

#define NUM_ETHERNET_PORTS 2
#define NUM_ETHERNET_MASTER_PORTS 2

#define ETHERNET_RX_ENABLE_TIMER_OFFSET_REQ 1

#define DISABLE_ETHERNET_PORT_FORWARDING 1

#endif
