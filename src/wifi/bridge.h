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


#ifndef BRIDGE_H
#define BRIDGE_H

//int iw(int num_param, ...);
int set_network(char * interface, char * name, int freq, int rate);
//int delete_interface(char * interface);
//int create_monitor(const char * interface, char *name);
//int set_ip(char *iface_name, const char *ip_addr);
//int set_iface_down(char * interface);
int locate_commands(void);
int set_iface_state(char * interface, int status);
int set_iface_mode(char * interface, int status);
int set_iface_address(char * interface, char * address);
int set_iface_param(char * interface, char * essid, int freq, int rate);
int set_iface_power(char * interface, int power_dbm);
int set_iface_rate(char * interface, int rate);
int delete_interface(char * interface);
int create_monitor(char * interface, char *name);
int wait_joining(char * interface, char * name, int freq);
int system2(const char * command, char * output, int out_size);
int check_if_exists(char * interface);
int iface_leave_ibss(char * interface);

#endif
