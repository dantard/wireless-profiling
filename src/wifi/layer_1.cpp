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


#include "layer_1.h"
extern "C"{
#include "radiotap_iter.h"
}
#include "radiotap.h"

/* variables */
static struct sockaddr_ll destination;
static unsigned char bcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static unsigned char source[6];
static char txb[2345];
static struct ethhdr * eh;

int raw_init(char * ifName, int protocol){

    int sockfd = -1, sockopt;
    struct ifreq ifopts, if_ip;
    eh = (struct ethhdr *) txb;
    eh->h_proto = protocol;

    memset(&if_ip, 0, sizeof(struct ifreq));

    /* Open PF_PACKET socket, listening for EtherType ETHER_TYPE */

    //if ((sockfd = socket(PF_PACKET, SOCK_RAW, htons(protocol))) == -1) {
    if ((sockfd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_ALL))) == -1) {
        perror("listener: socket");
        return -1;
    }

    strncpy(ifopts.ifr_name, ifName, IFNAMSIZ-1);
    ioctl(sockfd, SIOCGIFFLAGS, &ifopts);
    ifopts.ifr_flags |= IFF_PROMISC;
    ioctl(sockfd, SIOCSIFFLAGS, &ifopts);

    /* Allow the socket to be reused - incase connection is closed prematurely */
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &sockopt, sizeof sockopt) == -1) {
        perror("setsockopt");
        close(sockfd);
        return -1;
    }


    /* Get interface index in ifopts.ifr_ifindex */
    if (ioctl(sockfd, SIOCGIFINDEX, &ifopts) == -1) {
        perror("SIOCGIFINDEX");
        return -1;
    }

    int ifindex = ifopts.ifr_ifindex;

    /* Bind to device */
    //    if (setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, ifName, IFNAMSIZ-1) == -1)	{
    //        perror("SO_BINDTODEVICE");
    //        close(sockfd);
    //        return -1;
    //    }

    /* Bind socket to interface to receive frame ONLY from that interface */
    struct sockaddr_ll sa;
    sa.sll_family = AF_PACKET;
    sa.sll_ifindex = ifindex;
    sa.sll_protocol = htons(ETH_P_ALL);
    if ((bind(sockfd, (struct sockaddr *) &sa, sizeof(sa))) == -1) {
        perror("SO_BINDTODEVICE: ");
        return -1;
    }

    /* Get iface HW address */
    if (ioctl(sockfd, SIOCGIFHWADDR, &ifopts) < 0){
        perror("SIOCGIFHWADDR: ");
        return -1;
    }

    /* Prepare source address */
    for (int i = 0; i< 6; i++){
        source[i] = ifopts.ifr_hwaddr.sa_data[i];
    }
    memcpy((void *) eh->h_source, (void*) source, ETH_ALEN);

    /* Prepare destination */
    destination.sll_family = PF_PACKET;
    destination.sll_ifindex = ifindex;
    destination.sll_halen = ETH_ALEN;
    destination.sll_protocol = htons(protocol);

    return sockfd;
}

int raw_receive(int sockfd, char * buf, int BUF_SIZ){
    return recvfrom(sockfd, buf, BUF_SIZ, 0, NULL, NULL);
}
int raw_receive_timed(int sockfd, char * buf, int BUF_SIZ, int timeout){
    int r;
    static struct timeval tv;
    if (timeout > 0) {
        fd_set fd_rx;
        tv.tv_sec = 0;
        tv.tv_usec = 1000 * timeout; /* timeout in ms and not us */
        FD_ZERO(&fd_rx);
        FD_SET(sockfd, &fd_rx);
        r = select(FD_SETSIZE, &fd_rx, NULL, NULL, &tv);
    }else{
        r = 1;
    }

    if (r > 0){
        return raw_receive(sockfd,buf, BUF_SIZ);
    }else{
        return -1;
    }
}


int raw_send(int sockfd, char * buf, int BUF_SIZ, char mac[]){
    memcpy((void *) eh->h_dest, (void*) mac, ETH_ALEN);
    memcpy(txb + ETH_HLEN, buf, BUF_SIZ);
    destination.sll_pkttype = PACKET_BROADCAST;
    return sendto(sockfd, txb, BUF_SIZ + ETH_HLEN, 0, (struct sockaddr*) &destination, sizeof(destination));
}

rx_info_t raw_parse_radiotap2(char * buf, int len){

    rx_info_t rxinfo;
    memset(&rxinfo, 0, sizeof(rx_info_t));

    int offset = buf[0x02];

    if (offset  == 0x24 && len > 0x48 + 4){
        rxinfo.proto = htons(*((unsigned short *) &buf[0x5a]));
        rxinfo.rate = float(buf[0x19])/2.0;
        rxinfo.channel = *((unsigned short *) &buf[0x1a]);
        rxinfo.rssi = buf[0x22];
        rxinfo.antenna = buf[0x23];
        rxinfo.mac_time = *((unsigned long long int *) &buf[0x10]);
        rxinfo.error = 0;
        rxinfo.data = &buf[0x44];
        rxinfo.len = len - 0x44 - 4;
    }else if(offset == 0x12 && len > 0x4e){
        rxinfo.proto = htons(*((unsigned short *) &buf[0x48]));
        rxinfo.rate = float(buf[0x09])/2.0;
        rxinfo.channel = *((unsigned short *) &buf[0x0a]);
        rxinfo.rssi = buf[0x0e];
        rxinfo.antenna = buf[0x0f];
        rxinfo.mac_time = 0;
        rxinfo.error = 0;
        rxinfo.data = &buf[0x4e];
        rxinfo.len = len - 0x4e;
    }else if(offset == 0x15 && len > 0x4e){
        rxinfo.proto = htons(*((unsigned short *) &buf[0x4d]));
        rxinfo.rate = float(buf[0x09])/2.0;
        rxinfo.channel = *((unsigned short *) &buf[0x0a]);
        rxinfo.rssi = buf[0x0e];
        rxinfo.antenna = buf[0x0f];
        rxinfo.mac_time = 0;
        rxinfo.error = 0;
        rxinfo.data = &buf[0x53];
        rxinfo.len = len - 0x53;
    }else{
        rxinfo.error = 1;
    }
    return rxinfo;
}

rx_info_t raw_parse_radiotap(char * buff, int len){

    rx_info_t rxinfo;
    memset(&rxinfo, 0, sizeof(rx_info_t));

    struct ieee80211_radiotap_header * radiotap_hdr = (struct ieee80211_radiotap_header *) buff;
    struct ieee80211_radiotap_iterator iterator;
    int ret = ieee80211_radiotap_iterator_init(&iterator, radiotap_hdr, radiotap_hdr->it_len, 0);

    while (!ret) {

        ret = ieee80211_radiotap_iterator_next(&iterator);

        if (ret){
            continue;
        }

        /* see if this argument is something we can use */

        switch (iterator.this_arg_index) {
        /*
         * You must take care when dereferencing iterator.this_arg
         * for multibyte types... the pointer is not aligned.  Use
         * get_unaligned((type *)iterator.this_arg) to dereference
         * iterator.this_arg for type "type" safely on all arches.
         */
        case IEEE80211_RADIOTAP_RATE:
            rxinfo.rate = ((double)  (*iterator.this_arg) * 0.5);
            break;
        case IEEE80211_RADIOTAP_DBM_ANTSIGNAL:
            rxinfo.rssi = (*iterator.this_arg);
            break;
        case IEEE80211_RADIOTAP_ANTENNA:
            rxinfo.antenna = (*iterator.this_arg);
            break;
        case IEEE80211_RADIOTAP_DBM_TX_POWER:
            break;
        case IEEE80211_RADIOTAP_DATA_RETRIES:
            rxinfo.retries = (*iterator.this_arg);
            break;
        case IEEE80211_RADIOTAP_DB_ANTNOISE:
            //noise_dbm = *iterator.this_arg;
            break;
        case  IEEE80211_RADIOTAP_CHANNEL:
            rxinfo.channel = (* (unsigned short*) iterator.this_arg);
            break;
        default:
            break;
        }
    } /* while more rt headers */

    if (radiotap_hdr->it_len >= 0x24){
        rxinfo.mac_time = *((unsigned long long int *) &buff[0x10]);
    }

    ieee80211_frame * frame_80211 = (ieee80211_frame*) &buff[radiotap_hdr->it_len];

    rxinfo.ieee80211_duration = frame_80211->duration;
    rxinfo.ieee80211_seq = frame_80211->seq;
    rxinfo.len = len - radiotap_hdr->it_len - sizeof(ieee80211_frame);

    sprintf(rxinfo.addr1,"%02x%02x%02x%02x%02x%02x", frame_80211->addr1[0],frame_80211->addr1[1],frame_80211->addr1[2],frame_80211->addr1[3],frame_80211->addr1[4],frame_80211->addr1[5]);
    sprintf(rxinfo.addr2,"%02x%02x%02x%02x%02x%02x", frame_80211->addr2[0],frame_80211->addr2[1],frame_80211->addr2[2],frame_80211->addr2[3],frame_80211->addr2[4],frame_80211->addr2[5]);
    sprintf(rxinfo.addr3,"%02x%02x%02x%02x%02x%02x", frame_80211->addr3[0],frame_80211->addr3[1],frame_80211->addr3[2],frame_80211->addr3[3],frame_80211->addr3[4],frame_80211->addr3[5]);


    int qos_padding;
    unsigned char ieee80211_type = ((unsigned char)buff[radiotap_hdr->it_len]);
    rxinfo.ieee80211_type = ieee80211_type;
    /* is a DATA frame */
    if (ieee80211_type == 0x08){
        qos_padding = 0;
    }
    /* is a QOS frame */
    else if (ieee80211_type == 0x88){
        qos_padding = 2;
    }
    /* is a Beacon frame */
    else if (ieee80211_type == 0x80){
        rxinfo.data = (char*) (frame_80211->data);
        rxinfo.ieee80211_is_beacon = 1;
        return rxinfo;
    }else{
        rxinfo.error = 1;
        return rxinfo;
    }

    unsigned char flags = buff[radiotap_hdr->it_len + 1];
    rxinfo.ieee80211_to_ds = (flags & 0x01) > 0;
    rxinfo.ieee80211_is_retry = (flags & 0x08) > 0;

    ieee80211_llc_snap_header * snap = (ieee80211_llc_snap_header*) (frame_80211->data + qos_padding);
    rxinfo.data = (char*) (frame_80211->data + sizeof(ieee80211_llc_snap_header) + qos_padding);
    rxinfo.len = rxinfo.len - sizeof(ieee80211_llc_snap_header) - qos_padding;

    rxinfo.snap_type = ntohs(snap->ethertype);
    rxinfo.proto = snap->ethertype;

    if (rxinfo.snap_type == 0x0800){

        rxinfo.is_ip = 1;
        iphdr * ip = (iphdr*) (frame_80211->data + sizeof(ieee80211_llc_snap_header) + qos_padding);
        rxinfo.data = (char*) (frame_80211->data + sizeof(ieee80211_llc_snap_header) + sizeof(iphdr) + qos_padding);
        rxinfo.len = rxinfo.len - sizeof(iphdr);

        if (ip->protocol == 0x11){
            rxinfo.is_udp = 1;
            udphdr * udp = (udphdr *) (frame_80211->data + sizeof(ieee80211_llc_snap_header) + sizeof(iphdr) + qos_padding);
            rxinfo.udp_port = ntohs(udp->dest);
            rxinfo.data = (char*) (frame_80211->data + sizeof(ieee80211_llc_snap_header) + sizeof(iphdr) + sizeof(udphdr) + qos_padding);
            rxinfo.len = rxinfo.len - sizeof(udphdr);
        }
    }
    rxinfo.len = rxinfo.len;// - 4;
    rxinfo.error = 0;
    return rxinfo;
}
