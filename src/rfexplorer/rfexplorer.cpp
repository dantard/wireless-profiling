/*                Copyright (C) 2000-2017, Danilo Tardioli               *
 *           Centro Universitario de la Defensa Zaragoza, SPAIN          *
 *
 *  Contact Addresses: Danilo Tardioli                   dantard@unizar.es
 *
 *  Original code by Carlos Rizzo carloserizzo@gmail.com
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

#include "ros/ros.h"
#include "wireless_profiling/RFExplorer.h"
#include "std_msgs/String.h"

#include "../Argon.h"

#include <pthread.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/time.h>
#include <cstdio>
#include <signal.h>

#include <iostream>
#include <boost/circular_buffer.hpp>


bool print, extended, write_to_disk, use_ros, running = true, verbose = true;
int cb_size, column;
int fpSerial;
ros::Publisher pub_data;
std::string filename;
int lower_freq_MHz, upper_freq_MHz;

int serialInit(char * port, int baud) {
    int BAUD = 0;
    int fd = -1;
    struct termios newtio;

    //Open the serial port as a file descriptor for low level configuration
    fd = open(port, O_RDWR | O_NOCTTY );
    if (fd < 0) {
        ROS_ERROR("serialInit: Could not open serial device %s",port);
        return -1;
    }

    // set up new settings
    memset(&newtio, 0,sizeof(newtio));
    newtio.c_cflag =  CS8 | CLOCAL ;// | CREAD;  //no parity, 1 stop bit
    newtio.c_iflag = IGNCR;                  //ignore CR, other options off
    newtio.c_iflag |= IGNBRK;                //ignore break condition
    newtio.c_oflag = 0;                      //all options off
    newtio.c_lflag = ICANON;                 //process input as lines

    //newtio.c_cc[VMIN]  = 0;
    //newtio.c_cc[VTIME] = 100;
    // activate new settings
    tcflush(fd, TCIFLUSH);

    //Look up appropriate baud rate constant
    switch (baud) {
    case 38400:
    default:
        BAUD = B38400;
        break;
    case 19200:
        BAUD  = B19200;
        break;
    case 9600:
        BAUD  = B9600;
        break;
    case 4800:
        BAUD  = B4800;
        break;
    case 2400:
        BAUD  = B2400;
        break;
    case 1800:
        BAUD  = B1800;
    case 115200:
        BAUD  = B115200;
        break;
    case 500000:
        BAUD  = B500000;
        break;
    case 1200:
        BAUD  = B1200;
        break;
    }

    if (cfsetispeed(&newtio, BAUD) < 0 || cfsetospeed(&newtio, BAUD) < 0) {
        ROS_ERROR("serialInit: Failed to set serial baud rate: %d", baud);
        close(fd);
        return -1;
    }

    tcsetattr(fd, TCSANOW, &newtio);
    tcflush(fd, TCIOFLUSH);

    return fd;
}


//Process ROS command message, send to uController
void ucCommandCallback(const std_msgs::String::ConstPtr& msg){
    //fprintf(fpSerial, "%s", msg->data.c_str()); //appends newline
}

double median(boost::circular_buffer<double> scores){
    double median;
    size_t size = scores.size();

    sort(scores.begin(), scores.end());

    if (size  % 2 == 0) {
        median = (scores[size / 2 - 1] + scores[size / 2]) / 2;
    }
    else {
        median = scores[size / 2];
    }
    return median;
}

double mean(boost::circular_buffer<double> scores){
    double mean = 0;

    for(int i = 0 ; i < scores.size(); i++){
        mean=mean+scores[i];
    }
    mean=mean/scores.size();

    return mean;
}


void sfwrite(FILE * f, wireless_profiling::RFExplorer & frame, double ts, bool extended){
    fprintf(f,"%8.5f %5d %7.2f %3d %3d %7.2f %7.2f %6.4f %5.1f %7.2f %3d %7.2f %7.2f %7.2f",
            ts,
            frame.header.seq,
            frame.rssi,
            frame.column,
            frame.buffer_size,
            frame.mean,
            frame.median,
            frame.dt,
            frame.hz,
            frame.peak_value,
            frame.peak_column,
            frame.lower_freq_mhz,
            frame.upper_freq_mhz,
            frame.peak_frequency);
    if (extended){
        for (int i = 0; i< frame.data.size(); i++){
            fprintf(f," %7.2f", frame.data.at(i));
        }
    }
    fprintf(f,"\n");
    fflush(f);
}

double getTime(){
    if (use_ros){
        return ros::Time::now().toSec();
    }else{
        timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts); // Works on Linux
        return (double)ts.tv_sec + 1e-9*(double)ts.tv_nsec;
    }
}

void *rcvThread(void * ) {

    int seq = 0;
    FILE * filep;
    double prev = getTime();
    unsigned char ucResponse[512];

    boost::circular_buffer<double> cb(cb_size);

    if (write_to_disk){
        char stime[256];
        time_t now = time(NULL);

        strftime(stime, sizeof(stime), "%Y%m%d-%H%M%S", localtime(&now));
        filename += "-" + std::string(stime) +".rfex";

        filep = fopen(filename.c_str(),"w+");
        if (filep == 0){
            ROS_ERROR("Unable to open file %s for writing", filename.c_str());
            if (use_ros){
                ros::shutdown();
            }
            return NULL;
        }
    }

    while (running) {

        double max_dbm = -1e6;
        int max_col = -1, idx;

        idx = read(fpSerial,ucResponse, 512);

        if (idx == 0 || ucResponse[0] != '$'){
            continue;
        }

        int bgn, end;
        if (ucResponse[2] == 'p'){
            /* RF Explorer analyzer */
            bgn = 3; end = 115;
        }else{
            /* WIFI analyzer */
            bgn = 2; end = 15;
        }

        wireless_profiling::RFExplorer frame;

        double dbm, now = getTime();
        for (int i = bgn; i < end; i++){

            dbm = - double(ucResponse[i])/2.0;
            frame.data.push_back(dbm);

            if(dbm >= max_dbm){
                max_dbm = dbm;
                max_col = i - bgn;
            }
        }

        if (column < 0){
            cb.push_back(max_dbm);
            frame.column = max_col;
            frame.rssi = max_dbm;
        }else{
            column = column >= frame.data.size() ? frame.data.size() - 1: column;
            cb.push_back(frame.data[column]);
            frame.column = column;
            frame.rssi = frame.data[column];
        }

        frame.mean = mean(cb);
        frame.median = median(cb);
        frame.peak_value = max_dbm;
        frame.peak_column = max_col;
        frame.header.seq = seq++;
        frame.dt = now - prev;
        frame.buffer_size = cb_size;
        frame.hz = 1.0/frame.dt;
        frame.lower_freq_mhz = lower_freq_MHz;
        frame.upper_freq_mhz = upper_freq_MHz;
        frame.peak_frequency = lower_freq_MHz + double(frame.peak_column) * (upper_freq_MHz - lower_freq_MHz) / double(end-bgn);

        if (print){
            sfwrite(stdout,frame,now,extended);
        }

        if (write_to_disk){
            sfwrite(filep, frame,now, extended);
        }

        if (use_ros) {
            frame.header.stamp = ros::Time::now();
            pub_data.publish(frame);
        }

        if (verbose && !print){
            static int serial = 0;
            serial ++;
            serial % 4 == 3 && fprintf(stderr,"Streaming...\r");
            serial % 4 == 2 && fprintf(stderr,"Streaming.. \r");
            serial % 4 == 1 && fprintf(stderr,"Streaming.  \r");
            serial % 4 == 0 && fprintf(stderr,"Streaming   \r");
        }

        prev = now;
    }

    if (write_to_disk){
        fclose(filep);
    }

    if (use_ros){
        ros::shutdown();
    }

    pthread_exit(NULL);
    return NULL;
}

int parseRange(int &low, int &high, int &amp_bottom, int &amp_top){
    char cmd1[1000];
    memset(cmd1,0,sizeof(cmd1));

    while (running && memcmp(cmd1,"#C2-F:",6)){
        int len = read(fpSerial,cmd1,1000);
        cmd1[len] = 0;
    }

    int step, nsteps, a;
    low = high = amp_top = amp_bottom = 0;
    sscanf((const char*)&cmd1[6],"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",&low, &step, &amp_top, &amp_bottom, &nsteps,&a,&a,&a,&a,&a,&a,&a,&a);
    high = low + int(float(step*nsteps)/1000.0);
    high += high % 1000 ? 1 : 0;
    return 1;
}

int setRange(int low, int high, int amp_bottom, int amp_top){
    char cmd1[256];

    sprintf(cmd1,"#%cC2-F,%07d,%07d,%04d,%04d", 32, low, high, amp_top, amp_bottom);

    for (int i = 0; i < strlen(cmd1); i++){
        write(fpSerial, &cmd1[i], 1);
        usleep(25000);
    }
}

int requestInfo(){
    char cmd1[256];

    sprintf(cmd1,"#%cC0", 4);

    for (int i = 0; i < strlen(cmd1); i++){
        write(fpSerial, &cmd1[i], 1);
        usleep(25000);
    }
}

int configure(char * port, bool reconfigure, int baud){
    unsigned char cmd1[] = {0x23, 0x04, 0x63, 0x38};

    int fp = -1;

    if (reconfigure){
        verbose && fprintf(stderr, "Configuring RF Explorer...");
        fp = serialInit(port, baud);
        if (fp == -1){
            verbose && fprintf(stderr, "failure!\n");
            return -1;
        }
        for (int i = 0; i < sizeof(cmd1); i++){
            write(fp, &cmd1[i], 1);
            verbose && fprintf(stderr, ".");
            usleep(100000);
        }
        close(fp);
        usleep(100000);
        verbose && fprintf(stderr, "done.\n");

    }

    fp = serialInit(port, 115200);

    return fp;
}

void  my_handler(int s){
    fprintf(stderr,"Exiting...\n");
    running = false;
}

int main(int argc, char **argv) {

    std::string port;
    int baud, atop, abottom;
    bool reconfigure_port, is_wifi, is_freq_set;

    Argon a;
    a.addString('p',"port",port,"/dev/ttyUSB0", "RFExplorer's USB port");
    a.addString('w',"write",filename,"data.txt", "Write data to file", & write_to_disk);
    a.addToggle('x',"dont-reconfigure-port",reconfigure_port,true, "Don't reconfigure port");
    a.addToggle('R',"dont-use-ros",use_ros,true, "Don't use ROS");
    a.addSwitch('s',"print", print, "Print data to screen");
    a.addSwitch('e',"extended", extended, "Print extended data");
    a.addInt('b', "buf-size",cb_size,10,"Circular buffer size");
    a.addInt('c', "column",column,-1,"Column to be analysed (default: -1, use peak column)");
    a.addInt('B', "baud",baud,2400,"Default baud rate at turning on the device");
    a.addInt('l', "lower-freq-MHz", lower_freq_MHz, 0, "Lower frequency for RFExplorer", &is_freq_set);
    a.addInt('u', "upper-freq-MHz", upper_freq_MHz, 1000, "Upper frequency for RFExplorer");
    a.addSwitch('W', "wifi", is_wifi, "Set lower and upper wifi frequencies");
    a.addInt('U', "atop", atop, -40, "Lower frequency for RFExplorer", &is_freq_set);
    a.addInt('L', "abottom", abottom, -120, "Upper frequency for RFExplorer");

    a.addDependency('L','l');
    a.addDependency('l','u', Argon::BOTH);
    a.addDependency('L','U', Argon::BOTH);
    a.addIncompatibility('W',"lu");

    a.process(argc, argv);

    signal (SIGINT, my_handler);

    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != NULL){
        verbose = strstr(cwd,".ros") == NULL;
    }

    if (use_ros){
        ros::init(argc, argv, "rfexplorer", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
        ros::NodeHandle hn("~");

        bool ros_has_lf =  hn.getParam("lower_freq_MHz",lower_freq_MHz);
        bool ros_has_uf =  hn.getParam("upper_freq_MHz",upper_freq_MHz);
        hn.getParam("buf_size", cb_size);
        hn.getParam("column", column);
        hn.getParam("baud", baud);
        hn.getParam("port", port);
        hn.getParam("atop", atop);
        hn.getParam("abottom", abottom);
        if (ros_has_lf != ros_has_uf){
            ROS_ERROR("Both range frequencies must be set");
            ros::shutdown();
            exit(0);
        }else{
            is_freq_set |= ros_has_uf;
        }
    }

    fpSerial = configure( (char*) port.c_str(), reconfigure_port, baud);

    if (fpSerial == -1){
        ROS_ERROR("Unable to configure device on port %s", port.c_str());
        return -1;
    }

    if (is_wifi) {
        lower_freq_MHz = 2412;
        upper_freq_MHz = 2477;
    }else{
        int low, high, alow, ahigh;
        if (is_freq_set){

            if (lower_freq_MHz > upper_freq_MHz){
                fprintf(stderr,"*** ERROR: Upper frequency must be higher than lower frequency, exiting.\n");
                return -2;
            }

            verbose && fprintf(stderr,"Setting and checking frequency range...");
            setRange(lower_freq_MHz*1000, upper_freq_MHz*1000, abottom, atop);
            parseRange(low, high, alow, ahigh);
            if (low!=lower_freq_MHz*1000 || high!=upper_freq_MHz*1000 || ahigh!=atop || alow != abottom){                
                fprintf(stderr,"\n*** ERROR: Unable to set requested frequencies or amp, exiting.\n");
                return -3;
            }
            verbose && fprintf(stderr,"done.\n");
        }else{
            verbose && fprintf(stderr,"Getting frequency range...");
            requestInfo();
            parseRange(low, high, atop, abottom);
            lower_freq_MHz = low / 1000;
            upper_freq_MHz = high / 1000;
            verbose && fprintf(stderr,"done.\n");
        }
    }

    if (use_ros){
        ros::init(argc, argv, "rfexplorer");
        ros::NodeHandle nh;

        //ros::Subscriber sub_command = nh.subscribe("rfexplorer/in/command", 1, ucCommandCallback);
        pub_data = nh.advertise<wireless_profiling::RFExplorer>("rfexplorer/out/data", 1);

        //Create receive thread
        pthread_t rcvThrID;
        int err = pthread_create(&rcvThrID, NULL, rcvThread, NULL);
        if (err != 0) {
            ROS_ERROR("*** ERROR: Unable to create receive thread, exiting.");
            return -4;
        }
        ros::spin();
        pthread_join(rcvThrID, NULL);
        close(fpSerial);
    }else{
        rcvThread(NULL);
        close(fpSerial);
    }
    return 0;
}
