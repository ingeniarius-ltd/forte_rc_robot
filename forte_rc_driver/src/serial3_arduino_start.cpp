#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ros/ros.h>
#include "rs232/rs232.h"

#define ARDUINO_PORT 0
#define ARDUINO_BAURATE 115200
char mode[]={'8','N','1',0};  // 8bits, No Parity, 2 Stopbits,


bool setup_COM_port(){

    // wait 1 second to open port
    sleep (1);

    if(RS232_OpenComport(ARDUINO_PORT, ARDUINO_BAURATE, mode))
    {
        ROS_ERROR("Can not open comport: %d\n", ARDUINO_PORT);
        return false;
    }

    ROS_INFO("USB port Successfully open %d\n", ARDUINO_PORT);

    // wait 2 seconds for opened port
    sleep (2);

    return true;
}


int serialWrite(){

    unsigned char send_start_msg[3]={'@',//0x40,  //@
                                     '1',//0x31,  //1
                                     'e'};//0x65}; //e

    ROS_INFO("Command sent!!! \n");

    RS232_SendBuf(ARDUINO_PORT, send_start_msg, 3);  // Confirm to Spark that is alive too

    return 0;
}


int main()
{

    while(!setup_COM_port()){       // Retry every second if port is open
        sleep(1);
    }

    serialWrite();
    
    //RS232_CloseComport(ARDUINO_PORT);

    return 0;
}

