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

void failure(int i){
    exit(i);
}

void kill(){
    char result[256];
    int has_nm, has_supplicant, has_avahi;
    system2("ps ax | grep NetworkManager | grep -v grep | wc -l", result, 256);
    has_nm = atoi(result) > 0 ? 1 : 0;

    if (has_nm){
        fprintf(stderr,"Stopping Network Manager...");
        int res = system("service network-manager stop 2>/dev/null 1>/dev/null");
        sleep(1);
        if (res){
            fprintf(stderr,"failure.\n");
        }else{
            fprintf(stderr,"done.\n");
        }

    }

    system2("ps ax | grep wpa_supplicant | grep -v grep | wc -l", result, 256);
    has_supplicant = atoi(result) > 0 ? 1 : 0;

    if (has_supplicant){
        fprintf(stderr,"Killing wpa_supplicant(s)...");
        int res = system("killall -9 wpa_supplicant 2>/dev/null 1>/dev/null");
        sleep(1);
        if (res){
            fprintf(stderr,"failure.\n");
        }else{
            fprintf(stderr,"done.\n");
        }

    }

    system2("ps ax | grep avahi-daemon   | grep -v grep | wc -l", result, 256);
    has_avahi = atoi(result) > 0 ? 1 : 0;

    if (has_avahi){
        fprintf(stderr,"Stopping avahi-daemon(s)...");
        int res = system("service avahi-daemon stop 2>/dev/null 1>/dev/null");
        sleep(1);
        if (res){
            fprintf(stderr,"failure.\n");
        }else{
            fprintf(stderr,"done.\n");
        }

    }
}



