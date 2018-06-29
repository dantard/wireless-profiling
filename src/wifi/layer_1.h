/*                Copyright (C) 2000-2017, Danilo Tardioli               *
 *           Centro Universitario de la Defensa Zaragoza, SPAIN          *
 *
 *  Contact Addresses: Danilo Tardioli                   dantard@unizar.es
 *
 *  This is free software; you can  redistribute it and/or  modify it
 *  under the terms of the GNU General Public License  as published by the
 *  Free Software Foundation;  either  version 2, or (at  your option) any
 *  later version.
 *
 *  This software is distributed  in the  hope  that  it will be   useful,
 *  but WITHOUT  ANY  WARRANTY;   without  even the implied   warranty  of
 *  MERCHANTABILITY  or  FITNESS FOR A  PARTICULAR PURPOSE.    See the GNU
 *  General Public License for more details.
 *
 *  You should have received  a  copy of  the  GNU General Public  License
 *  distributed with RT-WMP;  see file COPYING.   If not,  write to the
 *  Free Software  Foundation,  59 Temple Place  -  Suite 330,  Boston, MA
 *  02111-1307, USA.
 *
 *  As a  special exception, if you  link this  unit  with other  files to
 *  produce an   executable,   this unit  does  not  by  itself cause  the
 *  resulting executable to be covered by the  GNU General Public License.
 *  This exception does  not however invalidate  any other reasons why the
 *  executable file might be covered by the GNU Public License.
 *
 *----------------------------------------------------------------------*/

#ifndef L1_INTERFACE_H_
#define L1_INTERFACE_H_

#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <netinet/ether.h>
#include <unistd.h>
#include <sys/types.h>

struct rx_info_t {
    int error, retries;
    unsigned short proto;
    float rate;
    unsigned short channel, ieee80211_duration, ieee80211_seq, snap_type, udp_port;
    signed char rssi;
    unsigned char antenna, is_ip, is_udp, ieee80211_to_ds, ieee80211_is_retry, ieee80211_is_beacon, ieee80211_type;
    unsigned long long mac_time;
    char * data;
    int len;
    char addr1[13];
    char addr2[13];
    char addr3[13];
};

#define  IEEE80211_TYPE_DATA     0x0008
#define u8 unsigned char
#define u16 unsigned short

struct ieee80211_frame
{
   u16 fc;                 /**< 802.11 Frame Control field */
   u16 duration;           /**< Microseconds to reserve link */
   u8 addr1[ETH_ALEN];     /**< Address 1 (immediate receiver) */
   u8 addr2[ETH_ALEN];     /**< Address 2 (immediate sender) */
   u8 addr3[ETH_ALEN];     /**< Address 3 (often "forward to") */
   u16 seq;                /**< 802.11 Sequence Control field */
   u8 data[0];             /**< Beginning of frame data */
} __attribute__((packed));

struct ieee80211_llc_snap_header
{
       /* LLC part: */
       u8 dsap;                /**< Destination SAP ID */
       u8 ssap;                /**< Source SAP ID */
       u8 ctrl;                /**< Control information */

       /* SNAP part: */
       u8 oui[3];              /**< Organization code, usually 0 */
       u16 ethertype;          /**< Ethernet Type field */
} __attribute__((packed));

int raw_init(char * ifName, int protocol);
int raw_receive(int sockfd, char * buf, int BUF_SIZ);
int raw_receive_timed(int sockfd, char * buf, int BUF_SIZ, int delay);
int raw_send(int sockfd, char * buf, int BUF_SIZ, char bcast_mac[]);
rx_info_t raw_parse_radiotap(char * buf, int len);

#endif /* L1_INTERFACE_H_ */
