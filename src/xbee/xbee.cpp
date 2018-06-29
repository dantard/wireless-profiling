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

#include "ros/ros.h"
#include "wifi/XBee.h"
#include "std_msgs/String.h"

#include "../Argon.h"
#include <time.h>
#include <pthread.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <signal.h>
#include <arpa/inet.h>
#include <iostream>
#include <boost/circular_buffer.hpp>


struct XBeeFrameRX{
    unsigned char start;
    unsigned char msb, lsb, id;
    unsigned short address;
    unsigned char rssi;
    unsigned char options;
    unsigned char data[0];
}__attribute__((packed));

struct XBeeFrameTX{
    unsigned char start;
    unsigned char msb, lsb, frame_type, frame_id;
    unsigned short address;
    unsigned char options;
    unsigned char data[0];
}__attribute__((packed));

struct XBeeFrameCMD{
    unsigned char start;
    unsigned char msb, lsb, frame_type, frame_id;
    unsigned char command[2];
    unsigned char data[0];
}__attribute__((packed));


struct data_t{
    unsigned char sender;
    int serial;
    int cs;
    unsigned char txpower;
    char data[0];
}__attribute__((packed));


bool print, write_to_disk, use_ros, print_sending, running = true, verbose = true;
int cb_size, delay, count, emitter, size, dest_address, power, fpSerial;
ros::Publisher pub_data;
std::string filename;
unsigned short rxaddress;


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
    newtio.c_cflag =  CS8 | CLOCAL | CREAD;  //no parity, 1 stop bit
    //newtio.c_iflag = IGNCR;                  //ignore CR, other options off
    //newtio.c_iflag |= IGNBRK;                //ignore break condition
    newtio.c_oflag = 0;                      //all options off
    //newtio.c_lflag = ICANON;                 //process input as lines
    newtio.c_cc[VMIN]  = 0;
    newtio.c_cc[VTIME] = 10;

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
       // ROS_ERROR("serialInit: Failed to set serial baud rate: %d", baud);
        close(fd);
        return -1;
    }

    tcsetattr(fd, TCSANOW, &newtio);
    tcflush(fd, TCIOFLUSH);

    return fd;
}


//Process ROS command message, send to uController

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

double getTime(){
    if (use_ros){
        return ros::Time::now().toSec();
    }else{
        timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts); // Works on Linux
        return (double)ts.tv_sec + 1e-9*(double)ts.tv_nsec;
    }
}

void sfwrite(FILE * f, wifi::XBee & frame){
    fprintf(f,"%8.5f %5d %3d %7.2f %3d %3d %7.2f %7.2f %6.4f %5.1f  %5d %4d %4d %2d",
            getTime(),
            frame.header.seq,
            frame.sender,
            frame.rssi,
            frame.size,
            frame.buffer_size,
            frame.mean,
            frame.median,
            frame.dt,
            frame.hz,
            frame.serial,
            frame.rxaddress,
            frame.txaddress,
            frame.txpower);
    fprintf(f,"\n");
    fflush(f);
}




unsigned char checksum(unsigned char * frame){
    int length = frame[2] + 3;
    unsigned char cs = 0xFF;
    for (int i  = 3; i < length; i++){
        cs = cs - frame[i];
    }
    return cs;
}


void *rcvThread(void * ) {

    int seq = 0;
    FILE * filep;
    unsigned char buff[2500];
    double prev = getTime();
    boost::circular_buffer<double> cb(cb_size);

    if (write_to_disk){
        char stime[256];
        time_t now = time(NULL);

        strftime(stime, sizeof(stime), "%Y%m%d-%H%M%S", localtime(&now));
        filename += "-" + std::string(stime) +".xbee";

        filep = fopen(filename.c_str(),"w");
        if (filep == 0){
            ROS_ERROR("Unable to open file %s for writing", filename.c_str());
            ros::shutdown();
            return NULL;
        }else{
            verbose && fprintf(stderr,"Logging in %s\n", filename.c_str());
        }
    }

    XBeeFrameRX * xframe = (XBeeFrameRX *) buff;

    while (running && (use_ros || ros::ok())) {

        int idx;

        do {
            idx = read(fpSerial, buff, sizeof(char));
        } while (running && (idx == 0 || xframe->start != 0x7e));

        while (running && idx < 4){
            idx += read(fpSerial, buff + idx, sizeof(char));
        }

        while (running && idx < xframe->lsb + 4){
            idx += read(fpSerial, buff + idx, sizeof(char));
        }

        if (!running) break;

        if (print_sending){
            fprintf(stderr, "Sent bytes: ");
            for (int i = 0; i < xframe->lsb + 4; i++){
                fprintf(stderr, "%2x ",buff[i]);
            }
            fprintf(stderr, "\n");
        }

        double now = getTime();

        cb.push_back(- xframe->rssi);

        wifi::XBee frame;

        frame.rssi = - xframe->rssi;
        frame.mean = mean(cb);
        frame.median = median(cb);
        frame.header.seq = seq++;
        frame.dt = now - prev;
        frame.buffer_size = cb_size;
        frame.hz = 1.0/frame.dt;
        frame.size = idx;
        frame.txaddress = htons(xframe->address);
        frame.rxaddress = rxaddress;

        data_t * data = (data_t*) &xframe->data[0];
        frame.sender = data->sender;
        frame.serial = data->serial;
        frame.txpower = data->txpower;

        if (print){
            sfwrite(stdout,frame);
        }

        if (write_to_disk){
            sfwrite(filep, frame);
        }

        if (use_ros){
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

        xframe->start = 0x0;        
        prev = now;
    }

    if (write_to_disk){
        fclose(filep);
    }

    if (use_ros){
        ros::shutdown();
    }

    pthread_exit(0);
    return NULL;
}

void *txThread(void * ) {

    unsigned char buff[2500];
    memset(buff, 0, 2500);
    XBeeFrameTX * txframe = (XBeeFrameTX *) buff;
    txframe->start = 0x7e;
    txframe->frame_type = 0x01;
    txframe->frame_id = 0x01;

    txframe->address = dest_address;
    if (dest_address!=0xFFFF){
        txframe->options = 0x00;
    }else{
        txframe->options = 0x04;
    }

    txframe->lsb = 5 + size;
    txframe->msb = 0;

    int idx = 0, errors = 0;
    while (running && (count < 0 || idx < count)) {

        data_t * data = (data_t*) &txframe->data[0];

        data->serial++;
        data->sender = emitter;
        data->txpower = power;
        data->cs = 0x65679;

        txframe->data[size] = checksum(buff);

        errors += write(fpSerial,buff, size + sizeof(XBeeFrameTX) + 1) == 0;

        if (print_sending){
            fprintf(stderr, "Sending frame %5d: ", idx + 1);
            for (int i = 0; i < size + sizeof(XBeeFrameTX) + 1; i++){
                fprintf(stderr, "%2x ",buff[i]);
            }
            fprintf(stderr, "\n");
        } else{
            fprintf(stderr, "Sending frame %5d (errors: %d)\r", idx + 1, errors);
        }
        idx++;
        usleep(delay*1000);
    }
    fprintf(stderr, "\nDone.\n");
    return NULL;
}


int  configure(char * port, int baud){
    verbose && fprintf(stderr, "Configuring port...");
    int fp = serialInit(port, baud);
    if (fp < 0){
        verbose && fprintf(stderr, "failure!\n");
        return -1;
    }
    usleep(250000);
    verbose && fprintf(stderr, "done.\n");
    return fp;
}

int execute_command(std::string command, int parameter, int p_len, unsigned char * data, int * len){

    unsigned char buff[256];
    XBeeFrameCMD * cmd = (XBeeFrameCMD *) buff;

    cmd->command[0] = command.c_str()[0];
    cmd->command[1] = command.c_str()[1];
    cmd->start = 0x7e;
    cmd->msb = 0;

    cmd->lsb = 4 + p_len;
    memcpy(&cmd->data[0], (char*) &parameter, p_len);

    cmd->frame_type = 0x08;
    cmd->frame_id = 0x01;
    cmd->data[p_len] = checksum(buff);

    int idx = 0;

    write(fpSerial,buff,sizeof(XBeeFrameCMD) + p_len + 1);

    while (running) {
        do {
            idx = read(fpSerial, buff, sizeof(char));
        } while (running && (idx == 0 || cmd->start != 0x7e));

        while (running && idx < 4){
            idx += read(fpSerial, buff + idx, sizeof(char));
        }

        while (running && idx < cmd->lsb + 4){
            idx += read(fpSerial,buff + idx, sizeof(char));
        }

        /* To filter out data if the sender is transmitting */
        if (cmd->frame_type == 0x88){
            if (data != 0  && len != 0 && cmd->data[0] == 0x00){
                *len = cmd->lsb - 5;
                memcpy(data, cmd->data + 1, *len);
            }
            break;
        }
    }
    return cmd->data[0];

}
void  my_handler(int s){
    fprintf(stderr,"Exiting...\n");
    running = false;
}

int main(int argc, char **argv) {
    std::string port, command, sdest_address, sparameter;
    bool is_command, has_parameter, has_parameter_16;
    int parameter, baud;

    Argon argon;
    argon.addString('p',"port",port,"/dev/ttyUSB0", "XBee's USB port");
    argon.addString('w',"write",filename,"data.txt", "Write data to file", & write_to_disk);
    argon.addSwitch('s',"print", print, "Print received data to screen");
    argon.addSwitch('X',"print-hex", print_sending, "Print hex data send or received");
    argon.addInt('e',"emitter-id", emitter, -1, "Use as emitter with the specified id");
    argon.addInt('b', "buf-size",cb_size,10,"Circular buffer size");
    argon.addInt('D', "delay-ms",delay,100,"Delay in ms");
    argon.addInt('c', "count", count, -1,"Number of frames to send");
    argon.addInt('S', "size", size, 50,"Number of frames to send");
    argon.addToggle('R',"dont-use-ros",use_ros,true, "Don't use ROS");
    argon.addString('C',"command",command,"", "Send a command", &is_command);
    argon.addString('P',"parameter-byte",sparameter, "", "Command parameter (int or hexadecimal)", &has_parameter);
    argon.addString('W',"parameter-word",sparameter, "", "Command parameter (int or hexadecimal)", &has_parameter_16);
    argon.addInt('B',"baud",baud, 38400, "Serial port speed");
    argon.addInt('x',"power",power, -1, "Set emitter TX power (0 to 4)");
    argon.addString('a',"dest-address", sdest_address, "0xFFFF", "Set destination address");

    argon.addIncompatibility('e', "bswR");
    argon.addIncompatibility('C', "wsXxebDcSR");
    argon.addDependency('D', 'e');
    argon.addDependency('x', 'e');
    argon.addDependency('P', 'C');
    argon.addDependency('W', 'C');

    argon.process(argc, argv);

    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != NULL){
        verbose = strstr(cwd,".ros") == NULL;
    }

    use_ros &= emitter == -1;

    if (use_ros){
        ros::init(argc, argv, "xbee", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
        ros::NodeHandle hn("~");
        hn.getParam("buf_size", cb_size);
        hn.getParam("port", port);
        hn.getParam("baud", baud);
        hn.getParam("print", print);
        write_to_disk |= hn.getParam("write", filename);

    }

    signal (SIGINT, my_handler);

    fpSerial = configure( (char*) port.c_str(), baud);
    if (fpSerial == -1){
        ROS_ERROR("Unable to configure device");
        return -1;
    }

    if (is_command){
        int len, error;
        unsigned char response[256];

        fprintf(stderr,"Executing command...");

        bool ishex = sparameter.find("0x")!=std::string::npos || sparameter.find("0X")!=std::string::npos;

        if (has_parameter){
            parameter = ishex ? strtoul(sparameter.c_str(), NULL, 16) : atoi(sparameter.c_str());
            error = execute_command(command, parameter, 1, response, &len);
        }else if (has_parameter_16){
            parameter = ishex ? htons(strtoul(sparameter.c_str(), NULL, 16)): htons(atoi(sparameter.c_str()));
            error = execute_command(command, parameter, 2, response, &len);
        }else{
            error = execute_command(command, parameter, 0, response, &len);
        }

        if (error == -1){
            fprintf(stderr,"failure!\n");
        }else if (error == 0){
            fprintf(stderr,"OK\n");
            if (len > 0){
                fprintf(stderr,"Response: ");
                for (int i = 0; i<len; i++){
                    fprintf(stderr,"%x ", response[i]);
                }
                fprintf(stderr,"\n");
            }
        }else{
            fprintf(stderr,"error 0x%02x\n", error);
        }
        return 0;
    }

    if (emitter >= 0){

        dest_address = strtoul(sdest_address.c_str(), NULL, 16);

        if (power!=-1){
            if (execute_command("PL", power, 1, 0, 0)){
                fprintf(stderr,"*** ERROR: Unable to set power, exiting.\n");
                return -1;
            }
        }

        int len = 0; power = 0;
        if (execute_command("PL", 0xFFFF, 0, (unsigned char*) & power, & len)){
            fprintf(stderr,"*** ERROR: Unable to read power, exiting.\n");
            return -1;
        }

        txThread(NULL);

    }else{

        int len;
        if (execute_command("MY", 0xFFFF, 0, (unsigned char*) &rxaddress, &len)){
            fprintf(stderr,"*** ERROR: Unable to get local address, exiting.\n");
            return -1;
        }else{
            rxaddress = ntohs(rxaddress);
        }

        if (use_ros){

            ros::NodeHandle nh;
            pub_data = nh.advertise<wifi::XBee>("xbee/" + std::to_string(rxaddress) + "/out/data", 1);

            pthread_t thread;
            if (pthread_create(&thread, NULL, rcvThread, NULL) != 0) {
                ROS_ERROR("*** ERROR: Unable to create thread, exiting.");
                return -1;
            }
            ros::spin();
            pthread_join(thread, 0);
        }else{
            rcvThread(NULL);
        }
    }
    close(fpSerial);
    return 0;
}
