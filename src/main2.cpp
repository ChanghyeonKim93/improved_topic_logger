#include <iostream>
#include <ros/ros.h>
#include <time.h>
#include <string>
#include <sstream>

#include "improved_topic_logger.h"

// keyboard input tool
#include "keyinput.h"
#include "topic_logger/imu_serial.h" // dedicated msgs for MPU6050 (arduino)

// Get current data/time, format is yyyy-mm-dd.hh:mm:ss
const std::string currentDateTime(){
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about data/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d_%H_%M_%S", &tstruct);

    return buf;
}

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "improved_topic_logger");
    ros::NodeHandle nh("~");

    int n_cams   = -1;
    int n_lidars = -1;

    string dir;    
    ros::param::get("~n_cameras", n_cams);
    ros::param::get("~n_lidars",  n_lidars);
    ros::param::get("~directory", dir);
    
    // Ground control system class
    ImprovedTopicLogger* itl = new ImprovedTopicLogger(nh, n_cams, n_lidars, dir);
    
    stringstream ss1;
    ss1 << dir << currentDateTime() << "/";
    string save_dir = ss1.str();
    cout << "save directory:[" << save_dir << "]\n";

    // user input manual.
    string user_manual;
    stringstream ss;
    ss << "\n==============================================\n|" 
    << "  Press a key..." 
    << "\n|    c: continuous save mode"
    << "\n|    s: save one scene (but IMU always logged in background.)" 
    << "\n|    q: cease the program"
    << "\n| Select an input: \n";
    user_manual = ss.str();
    cout << user_manual;

    string cam_configure_manual;
    ss.clear();
    ss.flush();

    ss << "\n |\n L\n";

    int cnt = 0;
    while(ros::ok())
    {
        // no need 'spinOnce()' for all loop!!
        int c = getch(); // call my own non-blocking input function
        if(c == 's') {
            cout << "\n\n[Operation]: save the current snapshot.\n";

            // send single query to all sensors.
            bool is_query_ok = itl->sendSingleQueryToAllSensors();
            
            // Save all data
            if(is_query_ok){
                // save and purge the current data!
                itl->saveAllData();
            }
            else {
                cout << "   fail to save...\n";
            }
            cout << user_manual;
        }
        else if(c == 'c') {
            cout << "\n\n[Operation]: save all streaming data.\n";
            
            // send single query to all sensors.

            // TODO!

            // Do something here! (3-D recon -> path planning)
            cout << user_manual;
        }
        else if(c == 'q') {
            itl->sendQuitMsgToAllSensors();
            cout << "\n\n[Operation]: quit program\n\n";
            break;
        }
        else if((c != 0)) {
            cout << ": Un-identified command...\n";
            cout << user_manual;
        }       
    }

// delete allocation.
delete itl;

ROS_INFO_STREAM("End of the program.\n");
return -1;
}
