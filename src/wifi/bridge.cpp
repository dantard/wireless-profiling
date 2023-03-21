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
#include <stdarg.h>
//#include "iw-4.0/iw.h"
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <netpacket/packet.h>
#include <net/ethernet.h>     /* the L2 protocols */
#include <asm/types.h>
#include <time.h>
#include <netinet/if_ether.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#define DBG(c) c

char iw[256];
char ifconfig[256];
const char * mute = "2>/dev/null 1>/dev/null";
char cmd[256];

int locate_commands(void) {
    if(system("ifconfig 2>/dev/null 1>/dev/null") == 0) {
        sprintf(ifconfig, "sudo ifconfig");
    } else if(system("/sbin/ifconfig 2>/dev/null 1>/dev/null") == 0) {
        sprintf(ifconfig, "sudo /sbin/ifconfig");
    } else {
        fprintf(stderr, "Unable to locate 'ifconfig' command");
        return -1;
    }

    if(system("iw 2>/dev/null 1>/dev/null") == 0) {
        sprintf(iw, "sudo iw");
    } else if(system("/sbin/iw 2>/dev/null 1>/dev/null") == 0) {
        sprintf(iw, "sudo /sbin/iw");
    } else {
        fprintf(stderr, "Unable to locate 'iw' command");
        return -2;
    }

    return 0;
}


int set_iface_state(char * interface, int status) {
    sprintf(cmd, "%s %s %s %s", ifconfig, interface, status ? "up" : "down", mute);
    DBG(fprintf(stderr, "%s\n", cmd););
    return system(cmd);
}
int iface_leave_ibss(char * interface) {
    sprintf(cmd, "%s dev %s ibss leave %s", iw, interface, mute);
    DBG(fprintf(stderr, "%s\n", cmd););
    return system(cmd);
}
int set_iface_mode(char * interface, int status) {
    sprintf(cmd, "%s dev %s set type %s %s", iw, interface, status ? "ibss" : "managed", mute);
    DBG(fprintf(stderr, "%s\n", cmd););
    return system(cmd);
}
int set_iface_address(char * interface, char * address) {
    sprintf(cmd, "%s %s %s up %s", ifconfig, interface, address, mute);
    DBG(fprintf(stderr, "%s\n", cmd););
    return system(cmd);
}
int set_iface_leave(char * interface) {
    sprintf(cmd, "%s dev %s ibss leave %s", iw, interface, mute);
    DBG(fprintf(stderr, "%s\n", cmd););
    return system(cmd);
}
int set_iface_param(char * interface, char * essid, int freq, int rate) {
    sprintf(cmd, "%s dev %s ibss join %s %d NOHT fixed-freq mcast-rate %d %s", iw, interface, essid, freq, rate, mute);
    DBG(fprintf(stderr, "%s\n", cmd););
    return system(cmd);
}
int set_iface_power(char * interface, int power_dbm) {
    sprintf(cmd, "%s dev %s set txpower fixed %d %s", iw, interface, power_dbm*100, mute);
    DBG(fprintf(stderr, "%s\n", cmd););
    return system(cmd);
}

int set_iface_rate(char * interface, int rate) {
    if (rate > 0){
        sprintf(cmd, "%s dev %s set bitrates legacy-2.4 %d %s", iw, interface, rate, mute);
    }else{
        sprintf(cmd, "%s dev %s set bitrates %s", iw, interface, mute);
    }
    DBG(fprintf(stderr, "%s\n", cmd););
    return system(cmd);
}

int delete_interface(char * interface) {
    sprintf(cmd, "%s dev %s del %s", iw, interface, mute);
    DBG(fprintf(stderr, "%s\n", cmd););
    return system(cmd);
}

int check_if_exists(char * interface) {
    sprintf(cmd, "%s %s %s", ifconfig, interface, mute);
    return system(cmd) == 0;
}

int system2(const char * command, char * output, int out_size){
    FILE *fp;
    fp = popen(command, "r");
    if (fp == NULL) {
        fprintf(stderr, "error");

        return -10000;
    }

    /* Read the output a line at a time - output it. */
    while (fgets(output, out_size-1, fp) != NULL);

    /* close */
    pclose(fp);

    return 0;
}

int create_monitor(char * interface, char *name) {
    locate_commands();
    delete_interface(name);
    sprintf(cmd, "%s dev %s interface add %s type monitor %s", iw, interface, name, mute);

    if(system(cmd) != 0) {
        return -1;
    }

    int res = set_iface_state(name, 1);

    if(res) {
        return -2;
    }

    return 0;
}

int wait_joining(char * interface, char * name, int freq) {
    sprintf(cmd, "%s dev %s link", iw, interface);
    fprintf(stderr, "Joining network...");
    FILE *fp;
    char result[1024], data[256], cnt = 0, fail = 0;

    while(1) {
        fp = popen(cmd, "r");
        char * len = fgets(result, 1024, fp);

        if(len != 0) {
            if(strstr(result, "Joined")) {
                fgets(result, 1024, fp);

                if(strstr(result, name) == 0) {
                    fail += 1;
                }

                fgets(result, 1024, fp);
                sprintf(data, "%d", freq);

                if(strstr(result, data) == 0) {
                    fail += 2;
                }
                break;
            }
        }

        fprintf(stderr, ".");
        pclose(fp);
        sleep(1);

        if(cnt ++ > 30) {
            return -10;
        }
    }

    if(fail == 0) {
        fprintf(stderr, "done.\n");
    } else if(fail == 1) {
        fprintf(stderr, "joined, but essid is wrong.\n");
    } else if(fail == 2) {
        fprintf(stderr, "joined, but frequency is wrong.\n");
    } else if(fail == 3) {
        fprintf(stderr, "joined, but both frequency and essid are wrong.\n");
    }

    return 0;
}


int set_network(char * interface, char * name, int freq, int rate) {
    int res = locate_commands();

    if(res) {
        return -1;
    }

    res = set_iface_state(interface, 0);

    if(res) {
        return -2;
    }

    res = set_iface_mode(interface, 0);

    if(res) {
        return -3;
    }

    res = set_iface_mode(interface, 1);

    if(res) {
        return -4;
    }

    res = set_iface_state(interface, 1);

    if(res) {
        return -5;
    }

    res = set_iface_param(interface, name, freq, rate);

    if(res) {
        return -7;
    }

    return 0;
}

