#!/usr/bin/env python
"""
Jarvis Schultz
March 2013

Simulate puppet and publish results so that ROS can be visualization.


##################
# SUBSCRIPTIONS  #
##################
# subscribe to three messages that define the inputs to the system

############
# SERVICES #
############
# provide a service to reset the system and go back to the initial pose

##############
# PUBLISHERS #
##############
# need to publish the joint states


"""

## define all imports:
import roslib; roslib.load_manifest('trep_puppet_demo')
import rospy
import tf
import trep
from trep import tx, ty, tz, rx, ry, rz
from math import sin, cos
from math import pi as mpi
import numpy as np
# import sys
# import scipy as sp
from puppet_definitions import *


## define global constants
DT = 1/30.
tf_freq = 100.
SHOULDER_LENGTH = 0.30 # default shoulder string lengths
ARM_LENGTH = 0.45 # default arm string lengths
BODY_HEIGHT = 1.50 # default location of the body in the z-axis



class PuppetSimulator:
    def __init__(self, sys):
        rospy.loginfo("Starting puppet simulator node")
        # first, let's just define the initial configuration of the system:
        self.sys = sys
        self.q0_guess = {
            # torso position
            'TorsoZ' : BODY_HEIGHT,
            # body robot
            'BodyRobotZ' : BODY_HEIGHT + SHOULDER_LENGTH,
            'LeftShoulderString' : SHOULDER_LENGTH,
            'RightShoulderString' : SHOULDER_LENGTH,
            # left robot
            'LeftRobotX' : torso_width/2,
            'LeftRobotY' : -(humerus_length + radius_length),
            'LeftRobotZ' : BODY_HEIGHT + ARM_LENGTH,
            'LeftArmString' : ARM_LENGTH,
            # right robot
            'RightRobotX' : -torso_width/2,
            'RightRobotY' : -(humerus_length + radius_length),
            'RightRobotZ' : BODY_HEIGHT + ARM_LENGTH,
            'RightArmString' : ARM_LENGTH,
            # left arm
            'LShoulderPhi' : -pi/2,
            # right arm
            'RShoulderPhi' : -pi/2,
            }
        self.sys.q = 0
        self.sys.q = self.q0_guess
        self.sys.satisfy_constraints()
        self.sys.minimize_potential_energy(keep_kinematic=True)
            
        
        
        
        




def main():
    """
    Run the main loop, by instatiating a PuppetSimulator, and then
    calling ros.spin
    """
    rospy.init_node('puppet_simulator', log_level=rospy.INFO)

    try:
        sim = PuppetSimulator(system)
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
