#include <iostream>
#include <ros/ros.h>
#include <time.h>
#include <string>
#include <sstream>
#include <queue>

// serial
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/poll.h>
#include <sys/time.h>
#include "serial_comm.h"


#define PI 3.141592

using namespace std;

int main(int argc, char **argv) {
    string serial_name = "/dev/arduinoMEGA";
    int message_length = 21;
    SerialCommunicator serial_com(serial_name, message_length);
    serial_com.runThread();
    
    ROS_INFO_STREAM("End of the program.\n");
    return -1;
}
