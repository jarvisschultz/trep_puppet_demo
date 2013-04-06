// keyboard_interface.cpp
// Jarvis Schultz
// Spring 2013

//---------------------------------------------------------------------------
// INCLUDES
//---------------------------------------------------------------------------
// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

// STANDARD
#include <cstdlib>
#include <stdio.h>
#include <termios.h>
#include <kbhit.h>


//---------------------------------------------------------------------------
// GLOBAL VARIABLES
//---------------------------------------------------------------------------

class KeyboardNode {

private:
    ros::NodeHandle n_;
    ros::Timer timer;
    ros::ServiceClient reset_client;
    std_srvs::Empty reset_request;
  
public:
    KeyboardNode() {
	ROS_INFO("Starting Keyboard Node...");
	// wait for the simulator reset service to become available:
	ROS_DEBUG("Waiting for simulator_reset service to become available");
	ros::service::waitForService("simulator_reset");
	// create a service client
	reset_client = n_.serviceClient<std_srvs::Empty>("simulator_reset");
	timer = n_.createTimer(ros::Duration(0.02),
			       &KeyboardNode::timerCallback, this);
    }

    void timerCallback(const ros::TimerEvent& e) {
	// check kbhit() to see if there was a keyboard strike and
	// transfer_flag to see if there is a node sending serial data
	if(kbhit()) {
	    ROS_DEBUG("Keyboard Strike Detected");
	    // get key pressed
	    char c = fgetc(stdin);
      
	    // what key did we press?
	    if(c == 'r') {
		ROS_WARN("Reset of simulator requested through keyboard interface");
		if (!reset_client.call(reset_request))
		    ROS_WARN("Failed to call simulator reset service");
	    }
	}
	return;
    }
};


//---------------------------------------------------------------------------
// MAIN
//---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_node");

    // // turn on debugging
    // log4cxx::LoggerPtr my_logger =
    // 	log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    // my_logger->setLevel(
    // 	ros::console::g_level_lookup[ros::console::levels::Debug]);

    KeyboardNode keyboard;

    // Wait for new data:
    ros::spin();

    return 0;
}
