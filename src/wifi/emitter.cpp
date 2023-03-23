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

#include <stdio.h>
#include "bridge.h"
#include <unistd.h>
#include <string.h>
#include "../Argon.h"
#include <arpa/inet.h>
#include "layer_1.h"
#include "common.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <chrono>
#include <thread>

double getTime(){

    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts); // Works on Linux
    return (double)ts.tv_sec + 1e-9*(double)ts.tv_nsec;

}

int64_t get_time_us(){
    const auto p1 = std::chrono::system_clock::now();
    int64_t begin = std::chrono::duration_cast<std::chrono::microseconds>(p1.time_since_epoch()).count();
    return begin;
}

int64_t start = 0;
int prev = 0, num = 0, global_prev = 0, pps = 0, first_num = -1;
ros::Publisher pub, pub2;

void th_pps(){
    while (ros::ok()) {
        auto now = get_time_us();
        if (now - start > 1000000) {
            pps = int(1000000 * int64_t(num - global_prev) / (now - start));
            //fprintf(stderr, "%d %ld %d\n", num - global_prev, now - start, pps);
            //pps = num - global_prev;
            start = now;
            global_prev = num;

            std_msgs::Int32 i32;
            i32.data = pps;
            pub.publish(i32);
        }
        usleep(10);
    }
}



int main(int argc, char *argv[]){
    char iface[32], buf[2500], dest_mac[6];

    Argon argo;
    std::string ssid, interface, dest_ip, my_ip, dest_mac_str;
    bool reconfigure, killing, wait_join, udp_on_ap, my_ip_set, shutup, use_ros;
    int freq, rate, period, count, size, eid, txpow, sock, protocol = 0x6868, udp;
    struct sockaddr_in serveraddr;

    argo.addString('i',"interface", interface, "wlan0", "Interface (default 'wlan0')");
    argo.addInt('f',"freq", freq, 2437, "Set frequency");
    argo.addInt('p',"period", period, 100, "Set period in ms");
    argo.addInt('x',"tx-power", txpow, -1, "Set transmission power");
    argo.addInt('c',"count", count, -1, "Number of frames (default -1, no limit)");
    argo.addInt('R',"rate", rate, -1, "Set channel rate");
    argo.addInt('e',"emitter-id", eid, 0, "Set emitter id");
    argo.addInt('s',"size", size, 512, "Set message size");
    argo.addString('S',"ssid", ssid, "wifi", "Set network SSID");
    argo.addSwitch('k', "kill", killing, "kill problematic apps");
    argo.addSwitch('w', "wait", wait_join, "Wait joning the network before continuing");
    argo.addToggle('y', "dont-configure-card", reconfigure, true, "Avoid configuring the interface");
    argo.addInt('u', "udp", udp, -1, "Use UDP with the specified port instead of RAW");
    argo.addInt('r', "raw", protocol, 0x6868, "Use RAW with the specified protocol number");
    argo.addString('M',"set-my-ip", my_ip, "192.168.1.1", "Use UDP and specify own address", & my_ip_set);
    argo.addString('A',"dest-ip", dest_ip, "192.168.1.1", "Use UDP and send to the specified address");
    argo.addSwitch('U', "udp-on-ap", udp_on_ap, "Shortcut for -u 34567 -y (for using UDP with Access Point)");
    argo.addString('a', "dest-mac", dest_mac_str, "FF:FF:FF:FF:FF:FF","Specify destination MAC (default broadcast)");
    argo.addSwitch('Q', "quiet", shutup, "Do not print anything");
    argo.addToggle('W', "use-ros", use_ros, false,"Publish data on ROS");


    argo.addIncompatibility('U',"fSkwyru");
    argo.addDependency('A',"uU", Argon::LEFT);
    argo.addDependency('i',"xRM", Argon::LEFT);

    argo.process(argc, argv);


    if (use_ros){
        ros::init(argc, argv, "emitter");
        ros::NodeHandle n;
        pub = n.advertise<std_msgs::Int32>("wifi/emitter/pps/", 1);
        pub2 = n.advertise<std_msgs::Int32>("wifi/emitter/num/", 1);
        auto th = new std::thread(th_pps);
    }


    locate_commands();
    sprintf(iface, "%s",interface.c_str());

    if (udp_on_ap){
        udp = 34567;
        reconfigure = false;
    }

    if (udp > 0){
        memset(&serveraddr, 0, sizeof(serveraddr));
        serveraddr.sin_family = PF_INET;
        serveraddr.sin_port = htons(udp);
        serveraddr.sin_addr.s_addr = inet_addr(dest_ip.c_str());
    }

    if (udp <= 0 || reconfigure || txpow >=0 || rate > 0){
        if (!check_if_exists(iface)){
            fprintf(stderr,"*** ERROR: No interface %s\n", iface);
            failure(1);
        }
    }

    if (killing){
        kill();
    }

    if (reconfigure){
        fprintf(stderr,"Configuring interface %s\n", iface);
        if (set_iface_state(iface, 0)){
            fprintf(stderr,"*** ERROR: unable to set down inteface %s\n", iface);
            failure(3);
        }
        if (set_iface_mode(iface, 1)){
            fprintf(stderr,"*** ERROR: unable to set ad-hoc mode on %s\n", iface);
            failure(4);
        }

        if (set_iface_state(iface, 1)){
            fprintf(stderr,"*** ERROR: unable to set up inteface %s\n", iface);
            failure(5);
        }

        iface_leave_ibss(iface);

        if (set_iface_param(iface, (char*) ssid.c_str(), freq, rate > 0 ? rate : 6)){
            fprintf(stderr,"*** ERROR: unable to set parameters for interface %s\n", iface);
            failure(6);
        }
    }

    if (txpow >= 0 && set_iface_power(iface, txpow)){
        fprintf(stderr,"*** ERROR: unable to set tx power for interface %s\n", iface);
        failure(7);
    }

    if (rate > 0 && set_iface_rate(iface, rate)){
        fprintf(stderr,"*** ERROR: unable to set rate for interface %s\n", iface);
        failure(17);
    }

    if (argo.provided('M') && set_iface_address(iface, (char *) my_ip.c_str())){
        fprintf(stderr,"*** ERROR: unable to set IP for interface %s\n", iface);
        failure(17);
    }

    if (wait_join){
        int result = wait_joining(iface, (char *) ssid.c_str(), freq);
        if (result){
            fprintf(stderr,"*** ERROR: unable to join network\n");
            failure(8);
        }
    }

    if (udp > 0){
        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        int val = 1;
        setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &val, sizeof(val));
    }else{
        int byte[6];
        if( 6 == sscanf( dest_mac_str.c_str(), "%x:%x:%x:%x:%x:%x",
                         &byte[0], &byte[1], &byte[2],
                         &byte[3], &byte[4], &byte[5] ) ){
            for (int i = 0; i<6 ; i++){
                dest_mac[i] = char(byte[i]);
            }
        }else{
            fprintf(stderr,"*** ERROR: incorrect mac address\n");
            failure(15);
        }
        sock = raw_init(iface, protocol);
    }

    if (sock < 0 ){
        fprintf(stderr,"*** ERROR: unable to create socket\n");
        failure(10);
    }

    int64_t start = 0;
    int idx = 0, err = 0;
    auto * data = (data_t * ) buf;
    data->sender = char(eid);

    char sys_class_filename[256];
    sprintf(sys_class_filename, "/sys/class/net/%s/statistics/tx_packets", interface.c_str());

    while (count == -1 || idx < count) {
        if (use_ros and not ros::ok()) {
            break;
        }

        data->serial = idx;
        int len;

        if (udp > 0) {
            len = sendto(sock, buf, size, 0, (struct sockaddr *) &serveraddr, sizeof(serveraddr));
        } else {
            len = raw_send(sock, buf, size, dest_mac);
        }
        err += len < 0 ? 1 : 0;

        FILE *fptr = fopen(sys_class_filename, "r");
        fscanf(fptr, "%d", &num);
        fclose(fptr);

        if (not shutup) {
            if (num != prev) {
                fprintf(stderr, "Enqueued: %d packets, %d sent per second (%d/%d)  \r", idx, pps, num, prev);
                prev = num;
            }
        }
        if (first_num == -1) {
            global_prev = num;
            first_num = num;
        }
        if (use_ros) {


            std_msgs::Int32 i32;
            i32.data = num - first_num;
            pub2.publish(i32);
        }

        if (period > 0) {
            usleep(period * 1000 + 1);
        }
        idx++;
    }
    fprintf(stderr,"\n");

    close(sock);
}
