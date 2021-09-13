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

using namespace std;

char stringChecksum(char* s, int idx_start, int idx_end) {
  char c = 0;
  for(int i = idx_start; i <= idx_end; i++) c ^= s[i];
  return c;
};

typedef union USHORT_UNION_{
    uint16_t ushort_;
    uint8_t bytes_[2];
} USHORT_UNION;

typedef union UINT_UNION_{
    uint32_t uint_;
    uint8_t bytes_[4];
} UINT_UNION;


inline short decode2BytesToShort(char h_byte, char l_byte){
    return (short) ( (unsigned char)h_byte << 8 | (unsigned char)l_byte);
};

double decodeTime(char sec_l, char sec_h, char usec0, char usec1, char usec2, char usec3){
    unsigned short sec 
        = (unsigned short) ( (unsigned char)sec_h << 8 | (unsigned char)sec_l);
    unsigned int usec  
        = (unsigned int)   ( (unsigned char)usec3 << 24 | (unsigned char)usec2 << 16 | (unsigned char)usec1 << 8 | (unsigned char)usec0 );
    return ((double)sec + (double)usec/1000000.0);
};


using namespace std;
int main(int argc, char **argv) {
    int fd;
    struct termios newtio_rx;
    struct pollfd poll_events; // event struct ( interrupt )
    int poll_state;

    char buf[1024];

    printf("This is Serial (RX) node.\n");
    fd = open("/dev/arduinoMEGA", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        printf("Error - no serial port /dev/arduinoMEGA is opened! Check the serial device.\n");
        return -1;
    }
    else printf("/dev/arduinoMEGA is opened.\n");

    // newtio_rx initialization.
    memset(&newtio_rx, 0, sizeof(newtio_rx));
    newtio_rx.c_cflag = B230400 | CS8 | CLOCAL | CREAD;
    newtio_rx.c_oflag = 0;
    newtio_rx.c_lflag = 0;
    newtio_rx.c_cc[VTIME] = 0;
    newtio_rx.c_cc[VMIN] = 1;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio_rx);

    // poll preparation
    poll_events.fd = fd;
    poll_events.events = POLLIN | POLLERR; // if received or if there is error.
    poll_events.revents = 0;

    // Read serial data
    int MSG_LEN = 20;
    int stack_len = 0;
    char STX = '$';
    char ETX = '%';
    char CTX = '*';
    char serial_stack[1024];
    char message[20];

    int cnt = 0;
    while (1) {
        // polling stage. (when some data is received.)
        poll_state = poll(                 //poll() call. event check.
            (struct pollfd *)&poll_events, // event registration
            1,                             // the number of pollfd which we want to check.
            500);                          // time out [ms]

        if (poll_state > 0) {
            if (poll_events.revents & POLLIN) { // event is receiving data.
                //#define POLLIN 0x0001 // 읽을 데이터가 있다.
                //#define POLLPRI 0x0002 // 긴급한 읽을 데이타가 있다.
                //#define POLLOUT 0x0004 // 쓰기가 봉쇄(block)가 아니다.
                //#define POLLERR 0x0008 // 에러발생
                //#define POLLHUP 0x0010 // 연결이 끊겼음
                //#define POLLNVAL 0x0020 // 파일지시자가 열리지 않은 것 같은, Invalid request (잘못된 요청)
                int buf_len = read(fd, buf, 1024);
                
                for(int i = 0; i < buf_len; ++i) {
                    if(buf[i] == ETX){ // Test 1: ETX
                        serial_stack[stack_len] = '\n';
                        if(serial_stack[0] == STX && serial_stack[MSG_LEN+1] == CTX) { // Test 2: STX and CTX
                            char check_sum = stringChecksum(serial_stack, 1, MSG_LEN);
                            if(serial_stack[MSG_LEN+2] == check_sum) { // Test 3: checksum test.
                                // valid data.
                                short ax = decode2BytesToShort(serial_stack[2],serial_stack[1]);
                                short ay = decode2BytesToShort(serial_stack[4],serial_stack[3]);
                                short az = decode2BytesToShort(serial_stack[6],serial_stack[5]);
                                short temperature = decode2BytesToShort(serial_stack[8],serial_stack[7]);
                                short gx = decode2BytesToShort(serial_stack[10],serial_stack[9]);
                                short gy = decode2BytesToShort(serial_stack[12],serial_stack[11]);
                                short gz = decode2BytesToShort(serial_stack[14],serial_stack[13]);
                                
                                double t = decodeTime(serial_stack[15],serial_stack[16],serial_stack[17],serial_stack[18],serial_stack[19],serial_stack[20]);
                                cout << t <<" / " << ax <<", " << ay << ", " << az << ", " << gx <<", " << gy << ", " << gz << " / temp: "<<temperature << "\n";

                            }
                        }
                        stack_len = 0;
                    }
                    else serial_stack[stack_len++] = buf[i];
                }

                //printf("Data- cnt:%d /  %d %s\n", cnt, buf_len, buf);
                ++cnt;
            }
            if (poll_events.revents & POLLERR) {// The connection is destructed.
                printf("ERROR - There is communication error. program exit.\n");
                break;
            }
        }
        else if (poll_state == 0) { //timeout.
            printf("TIMEOUT-500 [ms].\n");
        }
        else if(poll_state < 0) { // Error occurs.
            //printf("ERROR-critical error occurs. Program is shutdown.\n");
            //return -1;
            continue;
        }

        // Decode 
       
    }

    close(fd);
    ROS_INFO_STREAM("End of the program.\n");
    return -1;
}
