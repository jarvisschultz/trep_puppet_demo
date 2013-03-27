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
from sensor_msgs.msg import JointState as JS
import trep
from trep import tx, ty, tz, rx, ry, rz
from math import sin, cos, pi
import numpy as np
import re
# import sys
# import scipy as sp
import puppet_definitions as pd


## define global constants
DT = 1/30.
tf_freq = 100.
SHOULDER_LENGTH = 0.45 # default shoulder string lengths
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
            'BodyRobotTheta' : -pi/2,
            'BodyRobotZ' : BODY_HEIGHT + SHOULDER_LENGTH,
            'LeftShoulderString' : SHOULDER_LENGTH,
            'RightShoulderString' : SHOULDER_LENGTH,
            # left robot
            'LeftRobotTheta' : -pi/2,
            'LeftRobotX' : pd.torso_width/2,
            'LeftRobotY' : -(pd.humerus_length + pd.radius_length),
            'LeftRobotZ' : BODY_HEIGHT + ARM_LENGTH,
            'LeftArmString' : ARM_LENGTH,
            # right robot
            'RightRobotTheta' : -pi/2,
            'RightRobotX' : -pd.torso_width/2,
            'RightRobotY' : -(pd.humerus_length + pd.radius_length),
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
        self.q0 = self.sys.q
        self.mvi = trep.MidpointVI(self.sys)
        self.mvi.initialize_from_configs(0, self.sys.q, DT, self.sys.q)

        # fill out a message, and store it in the class so that I can update it
        # whenever necessary
        self.mappings = {
            'left_shoulder_psi' : 'LShoulderPsi',
            'left_shoulder_theta' : 'LShoulderTheta',
            'left_shoulder_phi' : 'LShoulderPhi',
            'right_shoulder_psi' : 'RShoulderPsi',
            'right_shoulder_theta' : 'RShoulderTheta',
            'right_shoulder_phi' : 'RShoulderPhi',
            'left_elbow_phi' : 'LElbowPhi',
            'right_elbow_phi' : 'RElbowPhi',
            'left_hip_psi' : 'LHipPsi',
            'left_hip_theta' : 'LHipTheta',
            'left_hip_phi' : 'LHipPhi',
            'right_hip_psi' : 'RHipPsi',
            'right_hip_theta' : 'RHipTheta',
            'right_hip_phi' : 'RHipPhi',
            'left_knee_phi' : 'LKneePhi',
            'right_knee_phi' : 'RKneePhi',
            }
        self.js = JS()
        self.names = [x for x in self.mappings.keys()]
        self.js.name = self.names
        self.js.header.frame_id = 'world'
        self.update_values()
        
        # define tf broadcaster and listener
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        # define a publisher for the joint states
        self.joint_pub = rospy.Publisher("joint_states", JS)
        # define a timer for publishing the frames and tf's
        rospy.Timer(rospy.Duration(1.0/tf_freq), self.send_joint_states)
        # define a timer for integrating the state of the VI
        rospy.Timer(rospy.Duration(DT), self.integrate_vi)

    def reset(self):
        self.sys.q = self.q0
        self.mvi.initialize_from_configs(0, self.sys.q, DT, self.sys.q)

    def integrate_vi(self, event):
        amp_func = lambda t: 0.1*sin(t*4/(pi))
        u = [0]*self.sys.nQk
        for i,qki in enumerate(self.sys.kin_configs):
            u[i] = qki.q
        u[self.sys.get_config('LeftRobotZ').index - self.sys.nQd] = \
          amp_func(self.mvi.t2) + self.q0[self.sys.get_config('LeftRobotZ').index]
        u[self.sys.get_config('RightRobotZ').index - self.sys.nQd] = \
          -amp_func(self.mvi.t2) + self.q0[self.sys.get_config('RightRobotZ').index]
        u[self.sys.get_config('LeftRobotY').index - self.sys.nQd] = \
          amp_func(self.mvi.t2) + self.q0[self.sys.get_config('LeftRobotY').index]
        u[self.sys.get_config('RightRobotY').index - self.sys.nQd] = \
          -amp_func(self.mvi.t2) + self.q0[self.sys.get_config('RightRobotY').index]
        try:
            self.mvi.step(self.mvi.t2 + DT, (), u)
        except:
            rospy.logwarn("No solution to DEL equations!")
            self.reset()
        self.update_values()
        
    def update_values(self):
        """
        Just fill in all of position array stuff for the JS message
        """
        pos = [self.sys.get_config(self.mappings[n]).q for n in self.names]
        self.js.position = pos

    def send_joint_states(self, event):
        tnow = rospy.Time.now()
        # first send the transform:
        quat = tuple(tf.transformations.quaternion_from_euler(
            self.sys.get_config('TorsoPsi').q,
            self.sys.get_config('TorsoTheta').q,
            self.sys.get_config('TorsoPhi').q,
            'rzyx'))
        point = tuple((
            self.sys.get_config('TorsoX').q,
            self.sys.get_config('TorsoY').q,
            self.sys.get_config('TorsoZ').q
            ))
        self.br.sendTransform(point, quat,
                              tnow,
                              'torso', 'world')
        self.js.header.stamp = tnow
        self.joint_pub.publish(self.js)

        # body input
        quat = tuple(tf.transformations.quaternion_from_euler(
            0, 0, self.sys.get_config('BodyRobotTheta').q, 'sxyz'))
        point = tuple((
            self.sys.get_config('BodyRobotX').q,
            self.sys.get_config('BodyRobotY').q,
            self.sys.get_config('BodyRobotZ').q
            ))
        self.br.sendTransform(point, quat,
                              tnow,
                              'input1', 'world')

        # left input
        quat = tuple(tf.transformations.quaternion_from_euler(
            0, 0, self.sys.get_config('LeftRobotTheta').q, 'sxyz'))
        point = tuple((
            self.sys.get_config('LeftRobotX').q,
            self.sys.get_config('LeftRobotY').q,
            self.sys.get_config('LeftRobotZ').q
            ))
        self.br.sendTransform(point, quat,
                              tnow,
                              'input2', 'world')

        
        # right input
        quat = tuple(tf.transformations.quaternion_from_euler(
            0, 0, self.sys.get_config('RightRobotTheta').q, 'sxyz'))
        point = tuple((
            self.sys.get_config('RightRobotX').q,
            self.sys.get_config('RightRobotY').q,
            self.sys.get_config('RightRobotZ').q
            ))
        self.br.sendTransform(point, quat,
                              tnow,
                              'input3', 'world')
        
def convert(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    s2 = re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()
    return s2



def main():
    """
    Run the main loop, by instatiating a PuppetSimulator, and then
    calling ros.spin
    """
    rospy.init_node('puppet_simulator', log_level=rospy.INFO)

    try:
        sim = PuppetSimulator(pd.system)
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
