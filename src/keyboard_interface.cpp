// keyboard_main.cpp
// Jake Ware and Jarvis Schultz
// Spring 2011


//---------------------------------------------------------------------------
// NOTES
//---------------------------------------------------------------------------

/*
operating_condition
  0: Idle
  1: Calibrate
  2: Run
  3: Stop
  4: Emergency Stop
*/


//---------------------------------------------------------------------------
// INCLUDES
//---------------------------------------------------------------------------

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <float.h>
#include <time.h>
#include <assert.h>
#include <kbhit.h>


//---------------------------------------------------------------------------
// GLOBAL VARIABLES
//---------------------------------------------------------------------------

class KeyboardNode {

private:
    ros::NodeHandle n_;
    ros::Publisher serial_pub;
    ros::Timer timer, show;
    int operating_condition;
  
public:
    KeyboardNode() {
	timer = n_.createTimer(ros::Duration(0.02),
			       &KeyboardNode::timerCallback, this);
	show = n_.createTimer(ros::Duration(0.5),
			      &KeyboardNode::update_screen, this);
    
	ROS_INFO("Starting Keyboard Node...");
	operating_condition = 0;
    }

    void timerCallback(const ros::TimerEvent& e) {
	//ROS_DEBUG("timerCallback triggered");
  
	static bool emergency_flag = false;

	// initialize operating_condition to idle for safety
	if(ros::param::has("/operating_condition")) {
	    ros::param::getCached("/operating_condition", operating_condition);
	    // did we get an emergency stop request?
	    if(operating_condition == 4 && emergency_flag == false) {
		ROS_WARN("Emergency Stop Requested");
		emergency_flag = true;
	    }
	}
	else {
	    ROS_WARN("Cannot Find Parameter: operating_condition");
	    ROS_INFO("Setting operating_condition to IDLE");
	    ros::param::set("/operating_condition", 0);
	    return;
	}
    
	// check kbhit() to see if there was a keyboard strike and
	// transfer_flag to see if there is a node sending serial data
	if(kbhit()) {
	    ROS_DEBUG("Keyboard Strike Detected");
      
	    // get key pressed
	    char c = fgetc(stdin);
      
	    // what key did we press?
	    // did we enter an idle command?
	    if(c == 'I') {
		// can we move from our current state to this state?
		if(operating_condition > 1) {
		    ROS_INFO("Robots State: IDLE");
		    ros::param::set("/operating_condition", 0);
		}
		else if(operating_condition == 0) {
		    ROS_INFO("Already in IDLE state");
		}
		else {
		    ROS_INFO("Cannot enter IDLE state from current state: %i",
			     operating_condition);
		}
	    }
      
	    // did we enter a calibrate command?
	    else if(c == 'C')	{
		if(operating_condition < 1) {
		    ROS_INFO("Robots State Change: CALIBRATE");
		    ros::param::set("/operating_condition", 1);
		}
		else if(operating_condition == 1) {
		    ROS_INFO("Already in CALIBRATE state");
		}
		else
		    ROS_INFO("Cannot enter CALIBRATE state"
			     " from current state: %i", operating_condition);
	    }
      
	    // did we enter a run command?
	    else if(c == 'P')	{
		if(operating_condition < 2) {
		    ROS_INFO("Robots State Change: RUN");
		    ros::param::set("/operating_condition", 2);
		}
		else if(operating_condition == 2) {
		    ROS_INFO("Already in RUN state");
		}
		else {
		    ROS_INFO("Cannot enter RUN state from"
			     " current state: %i", operating_condition);
		}
	    }
      
	    // did we enter a stop command?
	    else
	    {
		if(operating_condition < 3)
		{
		    emergency_flag = false;
		    ros::param::set("/operating_condition", 4);
		    ROS_INFO("Robots State Change: STOP"); 
		}
		else if(operating_condition == 3 || operating_condition == 4)
		    ROS_INFO("Already in STOP or EMERGENCY STOP state");
	
		else
		
		    ROS_INFO("Cannot enter STOP state from current "
			     "state: %i", operating_condition);
	    }
	} 
    
	return;
    }

    void update_screen(const ros::TimerEvent& e)
	{
	    static int last = 0;

	    if (operating_condition != last)
	    {
		// then update the screen
		switch(operating_condition)
		{
		case 0:
		    ROS_INFO("Operating condition: IDLE");
		    break;
		case 1:
		    ROS_INFO("Operating condition: CALIBRATE");
		    break;
		case 2:
		    ROS_INFO("Operating condition: RUN");
		    break;
		case 3:
		    ROS_INFO("Operating condition: IDLE");
		    break;
		}
	    }
	    last = operating_condition;
	    return;
	}
	
};


//---------------------------------------------------------------------------
// MAIN
//---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_node");

    KeyboardNode keyboard;

    // Wait for new data:
    ros::spin();

    return 0;
}
