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
#include <ros/ros.h>
#include <wireless_profiling/Frame.h>
#include "common.h"
#include <pcap.h>
#include <boost/circular_buffer.hpp>
#include <signal.h>
#include <chrono>

bool signaled = false, use_ros, quiet = false;

double getTime(bool use_ros){
    if (use_ros){
        return ros::Time::now().toSec();
    }else{
        timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts); // Works on Linux
        return (double)ts.tv_sec + 1e-9*(double)ts.tv_nsec;
    }
}

#define DEBUG(c) //c

void time(int idx){
    static auto prev = std::chrono::high_resolution_clock::now();

    if (idx == 0){
        prev = std::chrono::high_resolution_clock::now();
    }
    auto now = std::chrono::high_resolution_clock::now();
    DEBUG(std::cerr << "Time ("<< idx <<"): " << std::chrono::duration_cast<std::chrono::nanoseconds>(now-prev).count()/1000 << std::endl;);
    prev = now;
}


void my_handler (int param){
    static double last = 0;
    fprintf(stderr,"\n");
    if (getTime(false) - last < 1.0 || quiet){

        if (use_ros){
            ros::shutdown();
            fprintf(stderr,"\n");
        }else{
            fprintf(stderr,"\n");
            exit(0);
        }
    }
    last = getTime(false);
    signaled = true;
}

int main(int argc, char *argv[]){
    char iface[32], monit[32];
    char buffer[2500];
    char * buf;
    pcap_t * pcap;

    Argon a;
    int  freq, rate = 1, count, udp, protocol, sock, idx = 0;
    std::string ssid, interface, file, deleting;
    bool use_mon, reconfigure, is_deleting, hr, killing,
            wait_join, only_ds_to_sta, has_data, address, beacon, use_file, print, one_topic_per_sender;
    ros::Publisher pub;

    a.addSwitch('A', "address", address, "Show 802.11 addresses");
    a.addSwitch('b', "beacon", beacon, "Show also beacons");
    a.addInt('c',"count", count, -1, "Number of frames (default -1, no limit)");
    a.addString('D', "delete", deleting, "wlan0","Delete specified monitor interface", &is_deleting);
    a.addInt('f',"freq", freq, 2437, "Set frequency");
    a.addString('F',"pcap-file", file, "", "Use .pcap file for input", &use_file);
    a.addSwitch('H', "human-readable", hr, "Human readable format");
    a.addString('i',"interface", interface, "wlan0", "Interface (default 'wlan0')");
    a.addSwitch('k', "kill", killing, "kill problematic apps");
    a.addToggle('m', "dont-use-mon", use_mon, true, "Use monitor interface");
    a.addSwitch('o', "only-ds-to-sta", only_ds_to_sta, "Show only frame from DS to STA");
    a.addSwitch('p', "print", print, "Print on screen");
    a.addSwitch('q', "quiet", quiet, "Don't print anything");
    a.addInt('r', "raw", protocol, 0x6868, "Show frames with this protocol");
    a.addToggle('R', "dont-use-ros", use_ros, true,"Avoid the need for roscore");
    a.addString('s',"ssid", ssid, "wifi", "Set network SSID");
    a.addInt('u', "udp", udp, -1 ,"Show frames with this UDP port");
    a.addSwitch('w', "wait", wait_join, "Wait joning the network before continuing");
    a.addToggle('x', "dont-print-data", has_data, true, "Don't print sender data");
    a.addToggle('y', "dont-reconfigure", reconfigure, true,"Avoid reconfiguring the interface");
    a.addSwitch('Y', "one-topic-per-sender", one_topic_per_sender, "Publish one topic for each sender");


    a.addIncompatibility('F',"iskwmyD");
    a.addIncompatibility('p','H');
    a.addAlone('D');

    a.process(argc, argv);

    signal (SIGINT, my_handler);

    if (use_ros){
        ros::init(argc, argv, "wifi_" + interface, ros::init_options::NoSigintHandler);
        ros::NodeHandle n;
        a.processROS();
        pub = n.advertise<wireless_profiling::Frame> ("out/wifi/", 100);
    }

    locate_commands();

    if (is_deleting){
        fprintf(stderr,"Deleting interface %s...", deleting.c_str());
        int error =  delete_interface((char *) deleting.c_str());
        if (error){
            fprintf(stderr,"failed!\n");
        }else{
            fprintf(stderr,"Done.\n");
        }
        exit(error);
    }

    if (use_file){

        char errbuff[PCAP_ERRBUF_SIZE];
        pcap = pcap_open_offline(file.c_str(), errbuff);
        if (pcap == 0){
            failure(0);
        }
    }else{

        sprintf(iface, "%s", interface.c_str());
        sprintf(monit, "%smon", interface.substr(interface.length()-5).c_str());

        if (!check_if_exists(iface)){
            fprintf(stderr,"*** ERROR: No interface %s\n", iface);
            failure(1);
        }

        if (killing){
            kill();
        }

        if (use_mon){
            if (check_if_exists(monit)){
                fprintf(stderr,"Interface %s already exists\n", monit);
            }else{
                fprintf(stderr,"Creating monitor %s for interface %s\n", monit, iface);
                if (create_monitor(iface, monit)!=0){
                    fprintf(stderr,"*** ERROR: Unable to create monitor %s\n", monit);
                    failure(2);
                }
            }
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

            if (set_iface_param(iface, (char*) ssid.c_str(), freq, rate)){
                fprintf(stderr,"*** ERROR: unable to set parameters for interface %s\n", iface);
                failure(6);
            }
        }

        if (wait_join){
            int result = wait_joining(iface, (char *) ssid.c_str(), freq);
            if (result){
                fprintf(stderr,"*** ERROR: unable to join network\n");
                failure(7);
            }
        }

        if (use_mon){
            sock = raw_init(monit, protocol);
        }else{
            sock = raw_init(iface, protocol);
        }

        if (sock < 0 ){
            return 1;
        }
        buf = &buffer[0];
    }

    std::map<int, ros::Publisher> pubs;
    std::vector<int> lost(10,0);
    std::vector<int> total(10,0);
    std::vector<int> last_serial(10,-1);

    boost::circular_buffer<double> cb(20);
    boost::circular_buffer<double> rssi(20);

    while (count == -1 || idx < count){

        int len;
        time (0);

        if (use_file){
            struct pcap_pkthdr *header;
            if (pcap_next_ex(pcap, &header, (const u_char**) &buf) < 0){
                break;
            }
            len = header->len;
        }else{
            len = raw_receive_timed(sock, buf, 2500, 0);
        }
        time (1);
        if (use_ros && !ros::ok()){
            break;
        }

        if (len < 0){
            continue;
        }

        rx_info_t rxinfo;
        data_t * data;

        if (use_mon){
time (2);
            rxinfo = raw_parse_radiotap(buf, len);
time (3);
            if (rxinfo.error != 0){
                continue;
            }

            bool publish = (udp < 0 && protocol < 0 && !beacon);
            publish |= rxinfo.is_udp && rxinfo.udp_port == udp;
            publish |= rxinfo.proto == protocol;
            publish |= beacon && rxinfo.ieee80211_is_beacon;

            if (!publish){
                continue;
            }

            data = (data_t *) rxinfo.data;
        }else{
            data = (data_t *) &buf[14];
            short proto = *(short*)&buf[12];
            if (proto != 0x6868){
                continue;
            }
        }
        if (use_ros){
            time (4);
            wireless_profiling::Frame p;
            p.header.stamp = ros::Time::now();
            p.header.seq = idx;
            p.header.frame_id = "wifi";
            if (use_mon){
                p.antenna = rxinfo.antenna;
                p.channel = rxinfo.channel;
                p.len = rxinfo.len;
                p.mac_time = rxinfo.mac_time;
                p.proto = rxinfo.proto;
                p.rate = rxinfo.rate;
                p.rssi = rxinfo.rssi;
                p.addr1 = rxinfo.addr1;
                p.addr2 = rxinfo.addr2;
                p.addr3 = rxinfo.addr3;
                //REVIEW: memcpy(&p.data[0],rxinfo.data, rxinfo.len);
                p.ieee80211_duration = rxinfo.ieee80211_duration;
                p.ieee80211_is_beacon = rxinfo.ieee80211_is_beacon;
                p.ieee80211_seq = rxinfo.ieee80211_seq;
                p.ieee80211_is_retry = rxinfo.ieee80211_is_retry;
                p.ieee80211_to_ds = rxinfo.ieee80211_to_ds;
                p.is_ip =rxinfo.is_ip;
                p.is_udp =rxinfo.is_udp;
                p.retries = rxinfo.retries;
            }
            if (has_data){
                p.sender = data->sender;
                p.serial = data->serial;
            }

            if (last_serial[p.sender]==-1){
                last_serial[p.sender] = p.serial - 1;
            }

            if (total[p.sender] %1000 == 0){
                total[p.sender] = 0;
                lost[p.sender] = 0;
            }

            total[p.sender] += 1;
            lost[p.sender] = lost[p.sender] + (p.serial - last_serial[p.sender] - 1);

            p.mac_time = 100-100*lost[p.sender]/total[p.sender];


            last_serial[p.sender] = p.serial;


            if (one_topic_per_sender){
                if (pubs.find(p.sender) == pubs.end()){
                    ros::NodeHandle n;
                    pubs[p.sender] = n.advertise<wireless_profiling::Frame> ("out/wifi/" + std::to_string(p.sender), 100);
                }
                pubs[p.sender].publish(p);
            }
            time (5);

            pub.publish(p);
            time (6);


        }

        if (hr){
            fprintf(stderr,"Time: %17f Count: %5d type:%02x tods:%1d retries:%1d size:%4d rate:%5.1f freq:%4d rssi:%3d antenna:%1d time:%8lld",
                    getTime(use_ros),idx, rxinfo.ieee80211_type, rxinfo.ieee80211_to_ds, rxinfo.ieee80211_is_retry, rxinfo.len,
                    rxinfo.rate, rxinfo.channel, rxinfo.rssi, rxinfo.antenna, rxinfo.mac_time);
            if (has_data){
                fprintf(stderr," sender: %3d serial: %8d", data->sender, data->serial);
            }
            if (address){
                fprintf(stderr," %s %s %s",rxinfo.addr1,rxinfo.addr2,rxinfo.addr3);
            }
            fprintf(stderr,"\n");
        }else if (print){
            fprintf(stderr,"%17f %5d %02d %1d %1d %4d %5.1f %4d %3d %1d %8lld",
                    getTime(use_ros), idx, rxinfo.ieee80211_type, rxinfo.ieee80211_to_ds, rxinfo.ieee80211_is_retry, rxinfo.len,
                    rxinfo.rate, rxinfo.channel, rxinfo.rssi, rxinfo.antenna, rxinfo.mac_time);
            if (has_data){
                fprintf(stderr," %3d %8d", data->sender, data->serial);
            }
            if (address){
                fprintf(stderr," %s %s %s",rxinfo.addr1,rxinfo.addr2,rxinfo.addr3);
            }
            fprintf(stderr,"\n");
        }else if (!quiet){
            static int rx = 0, missed = 0, last = data->serial - 1, rpt = 0;
            static double last_time = getTime(use_ros);

            if (signaled || data->serial < last){
                last = data->serial - 1;
                rx = 0;
                missed = 0;
                signaled = false;
            }

            double now = getTime(use_ros);
            cb.push_back(now-last_time);
            double mean = 0;
            for (int i = 0; i<cb.size(); i++){
                mean+=cb.at(i);
            }

            rssi.push_back(rxinfo.rssi);
            double mrssi = 0;
            for (int i = 0; i<rssi.size(); i++){
                mrssi+=rssi.at(i);
            }

            if (data->serial == last){
                rpt ++;
            }else{
                missed += (data->serial - last - 1);
            }
            time (6);


                fprintf(stderr,"RSSI:%5.1f | RX:%8d | SER: %8d | Lost:%5d | PDR:%5.1f | RTY:%5d | T:%6.4fms     \r", mrssi/double(rssi.size()), rx++, data->serial,
                    missed, 100.0 - 100.0*double(missed)/double(rx + missed), rpt, mean/double(cb.size()) );


            time (7);

            last = data->serial;
            last_time = now;
        }

        idx++;
    }


    close(sock);
}
