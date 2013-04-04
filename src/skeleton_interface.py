#!/usr/bin/env python
################
# ROS IMPORTS: #
################
import roslib; roslib.load_manifest('trep_puppet_demo')
import rospy
import tf
from skeletonmsgs_nu.msg import Skeletons
from skeletonmsgs_nu.msg import Skeleton
from skeletonmsgs_nu.msg import SkeletonJoint


####################
# NON ROS IMPORTS: #
####################
import sys
import os
import math
import copy
from math import fmod, pi, copysign
import numpy as np


####################
# GLOBAL VARIABLES #
####################
DT = 1/100. # rate at which we will send out all of the control frames
CONWF = 'world'
INPWF = 'camera_depth_optical_frame'
SCALING = 1
# filter parameters:
FALPHA = 0.25
GAMMA = 0.05
A_LOW = 0.01
A_HIGH = 0.50
V_HIGH = 0.008
V_LOW = 0.001



def makeMarker( msg, color ):
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.1
    marker.color.g = 0.1
    marker.color.b = 0.1
    marker.color.a = 0.75
    if color == 'red':
        marker.color.r += 0.4
    elif color == 'blue':
        marker.color.b += 0.4
    elif color == 'green':
        marker.color.g += 0.4
    else:
        rospy.warn("Marker color not recognized!")

    return marker


class SingleController:
    """
    This class will be used for each individual input.  I need to provide
    methods for doing the following things:

    1) Storing the location of the "key" joint at a current time.... this will
    be used for providing the mapping between the person's joints and the puppet
    inputs.
    2) Update the filter
    3) Send out the frames associated with the controls
    4) Take in/ update the nominal location of the puppet's kinematic inputs so
    that I can (re)calibrate whenever I need to
    5) Add marker and send its pose
    """
    def __init__(self, joint, frame, pos):
        # joint ~ user's  joint controlling the kinematic input
        # frame ~ the frame that we should publish to control the kinematic
        #       input
        # pos ~ nominal location of the kinematic config variable in trep
        #       simulation... used for determining offset
        self.joint = joint
        self.frame = frame
        self.pos = pos
        
        return


    def update_filter(self, skel):
        """
        Take in a skeleton message, and use self.joint to update the filter, and
        get an estimate of the pose of the joint
        """
        
        return


    def send_transform(self, br):
        """
        Just build and send the appropriate transform associated with this joint
        """
        
        return


    def update_pos(self, pos):
        
        return


class SkeletonController:
    def __init__(self):
        rospy.loginfo("Starting skeleton controller interface")
        # define frames that we will publish, and what the frames they listen
        # for are:

        # define tf broadcaster and listener:
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        # offer a service for resetting controls:
        self.reset_srv_provider = rospy.Service('simulator_reset', SS.Empty, self.reset_provider)

        
        # setup a timer to send out the key frames:
        rospy.Timer(rospy.Duration(DT), self.send_transforms)


        
        return

    def reset(self):

        pass

    def wait_and_update_frames(self):
        # wait for the frames to be available.

        # now store the nominal kinematic var locations

        return

    def send_transforms(self, event):

        return


def main():
    rospy.init_node('skeleton_interface', log_level=rospy.INFO)

    rospy.set_param('legs', False)

    try:
        sim = SkeletonController()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
