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
import std_srvs.srv as SS
import visualization_msgs.msg as VM
import std_msgs.msg as SM
import geometry_msgs.msg as GM
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
SHOULDER_LENGTH = 2*0.45 # default shoulder string lengths
ARM_LENGTH = 2*0.45 # default arm string lengths
LEG_LENGTH = 2*0.65 # default leg length strings
BODY_HEIGHT = 1.50 # default location of the body in the z-axis
EXCEPTION_COUNT_MAX = 5
BLACK = (0,0,0,1)
BLUE = (0.1,0.1,0.5,1)
GREEN = (0.1,0.5,0.1,1)
RED = (0.5,0.1,0.1,1)
WHITE = (1.0,1.0,1.0,1)


class StringMarker:
    def __init__(self, f1, f2, listener, color='black'):
        self.marker = VM.Marker()
        self.marker.type = VM.Marker.LINE_LIST
        self.marker.header.frame_id = "world"
        self.marker.id = hash(f1+f2)%(2**16)
        self.marker.scale = GM.Vector3(*(0.01,0.01,0.01))
        self.set_color(color)
        self.listener = listener
        self.f1 = f1
        self.f2 = f2
        self.update_points()

    def update_points(self):
        self.marker.header.stamp = rospy.Time.now()
        e1, p1 = self.get_point_loc(self.f1)
        e2, p2 = self.get_point_loc(self.f2)
        if e1 or e2:
            p1 = (0,0,0)
            p2 = (0,0,0)
        self.marker.points = [
            GM.Point(*p1),
            GM.Point(*p2)
            ]

    def set_color(self, color):
        if color == 'black':
            self.marker.color = SM.ColorRGBA(*BLACK)
        elif color == 'blue':
            self.marker.color = SM.ColorRGBA(*BLUE)
        elif color == 'green':
            self.marker.color = SM.ColorRGBA(*GREEN)
        elif color == 'red':
            self.marker.color = SM.ColorRGBA(*RED)
        elif color == 'white':
            self.marker.color = SM.ColorRGBA(*WHITE)
        else:
            rospy.logwarn("String marker color not recognized!")
            self.marker.color = SM.ColorRGBA(*BLACK)
        
    def get_point_loc(self, frame):
        try:
            pos,quat = self.listener.lookupTransform("world", frame,
                                                     rospy.Time())
            err = False
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            pos = (0.0, 0.0, 0.0)
            err = True
        return err, pos
    

class PuppetSimulator:
    def __init__(self, sys):
        rospy.loginfo("Starting puppet simulator node")
        # first, let's just define the initial configuration of the system:
        self.sys = sys
        self.legs_bool = rospy.get_param('legs', False)
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
        if self.legs_bool:
            self.q0_guess.update( {
                # left leg robot
                'LeftLegRobotTheta' : -pi/2,
                'LeftLegRobotX' : pd.torso_width/4,
                'LeftLegRobotY' : -(pd.humerus_length + pd.radius_length),
                'LeftLegRobotZ' : BODY_HEIGHT + LEG_LENGTH,
                'LeftLegString' : LEG_LENGTH,
                # right leg robot
                'RightLegRobotTheta' : -pi/2,
                'RightLegRobotX' : -pd.torso_width/4,
                'RightLegRobotY' : -(pd.humerus_length + pd.radius_length),
                'RightLegRobotZ' : BODY_HEIGHT + LEG_LENGTH,
                'RightLegString' : LEG_LENGTH,
                })
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
        self.count_exceptions = 0

        # define tf broadcaster and listener
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        # add constraint visualization:
        self.con_vis = [StringMarker('input1', 'left_shoulder_hook', self.listener)]
        self.con_vis.append(StringMarker('input1', 'right_shoulder_hook', self.listener))
        self.con_vis.append(StringMarker('input2', 'left_hand_hook', self.listener))
        self.con_vis.append(StringMarker('input3', 'right_hand_hook', self.listener))
        if self.legs_bool:
            self.con_vis.append(StringMarker('input4', 'left_knee_hook', self.listener))
            self.con_vis.append(StringMarker('input5', 'right_knee_hook', self.listener))
        # define a publisher for the joint states
        self.joint_pub = rospy.Publisher("joint_states", JS)
        # define a publisher for the constraint Markers
        self.con_pub = rospy.Publisher("visualization_markers", VM.MarkerArray)
        # define a timer for publishing the frames and tf's
        rospy.Timer(rospy.Duration(1.0/tf_freq), self.send_joint_states)
        # request that we get a service handler:
        rospy.loginfo("Waiting for reset handling service...")
        rospy.wait_for_service("simulator_reset")
        self.reset_srv_client = rospy.ServiceProxy("simulator_reset", SS.Empty)
        # offer service to reset simulation
        # self.reset_srv_provider =
        # define a timer for integrating the state of the VI
        rospy.loginfo("Starting integration...")
        rospy.Timer(rospy.Duration(DT), self.integrate_vi)

    def reset(self):
        try:
            self.reset_srv_client(SS.EmptyRequest())
        except rospy.ServiceException, e:
            rospy.loginfo("Service did not process request: %s"%str(e))
        rospy.sleep(5/tf_freq)
        self.count_exceptions = 0
        self.sys.q = self.q0
        self.mvi.initialize_from_configs(0, self.sys.q, DT, self.sys.q)

    def integrate_vi(self, event):
        u = [0]*self.sys.nQk
        for i,qki in enumerate(self.sys.kin_configs):
            u[i] = qki.q

        ########
        # BODY #
        ########
        xkey = self.sys.get_config('BodyRobotX').index - self.sys.nQd
        ykey = self.sys.get_config('BodyRobotY').index - self.sys.nQd
        zkey = self.sys.get_config('BodyRobotZ').index - self.sys.nQd
        pos = None
        try:
            pos,quat = self.listener.lookupTransform("world", "body_input",
                                                     rospy.Time())
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            pass
        if pos:
            u[xkey] = pos[0]
            u[ykey] = pos[1]
            u[zkey] = pos[2]
        else:
            tmp = self.sys.get_frame('BodyRobotCenterPOV').p()
            u[xkey] = tmp[0]
            u[ykey] = tmp[1]
            u[zkey] = tmp[2]

        ########
        # LEFT #
        ########
        xkey = self.sys.get_config('LeftRobotX').index - self.sys.nQd
        ykey = self.sys.get_config('LeftRobotY').index - self.sys.nQd
        zkey = self.sys.get_config('LeftRobotZ').index - self.sys.nQd
        pos = None
        try:
            pos,quat = self.listener.lookupTransform("world", "left_input",
                                                     rospy.Time())
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            pass
        if pos:
            u[xkey] = pos[0]
            u[ykey] = pos[1]
            u[zkey] = pos[2]
        else:
            tmp = self.sys.get_frame('LeftRobotCenterPOV').p()
            u[xkey] = tmp[0]
            u[ykey] = tmp[1]
            u[zkey] = tmp[2]

        #########
        # RIGHT #
        #########
        xkey = self.sys.get_config('RightRobotX').index - self.sys.nQd
        ykey = self.sys.get_config('RightRobotY').index - self.sys.nQd
        zkey = self.sys.get_config('RightRobotZ').index - self.sys.nQd
        pos = None
        try:
            pos,quat = self.listener.lookupTransform("world", "right_input",
                                                     rospy.Time())
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            pass
        if pos:
            u[xkey] = pos[0]
            u[ykey] = pos[1]
            u[zkey] = pos[2]
        else:
            tmp = self.sys.get_frame('RightRobotCenterPOV').p()
            u[xkey] = tmp[0]
            u[ykey] = tmp[1]
            u[zkey] = tmp[2]

        if self.legs_bool:
            ############
            # LEFT LEG #
            ############
            xkey = self.sys.get_config('LeftLegRobotX').index - self.sys.nQd
            ykey = self.sys.get_config('LeftLegRobotY').index - self.sys.nQd
            zkey = self.sys.get_config('LeftLegRobotZ').index - self.sys.nQd
            pos = None
            try:
                pos,quat = self.listener.lookupTransform("world", "left_leg_input",
                                                         rospy.Time())
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                pass
            if pos:
                u[xkey] = pos[0]
                u[ykey] = pos[1]
                u[zkey] = pos[2]
            else:
                tmp = self.sys.get_frame('LeftLegRobotCenterPOV').p()
                u[xkey] = tmp[0]
                u[ykey] = tmp[1]
                u[zkey] = tmp[2]

            #############
            # RIGHT LEG #
            #############
            xkey = self.sys.get_config('RightLegRobotX').index - self.sys.nQd
            ykey = self.sys.get_config('RightLegRobotY').index - self.sys.nQd
            zkey = self.sys.get_config('RightLegRobotZ').index - self.sys.nQd
            pos = None
            try:
                pos,quat = self.listener.lookupTransform("world", "right_leg_input",
                                                         rospy.Time())
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                pass
            if pos:
                u[xkey] = pos[0]
                u[ykey] = pos[1]
                u[zkey] = pos[2]
            else:
                tmp = self.sys.get_frame('RightLegRobotCenterPOV').p()
                u[xkey] = tmp[0]
                u[ykey] = tmp[1]
                u[zkey] = tmp[2]

        try:
            self.mvi.step(self.mvi.t2 + DT, (), u)
        except:
            rospy.logwarn("No solution to DEL equations!")
            self.count_exceptions += 1
            if self.count_exceptions > EXCEPTION_COUNT_MAX:
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
        if self.legs_bool:
            # left leg input
            quat = tuple(tf.transformations.quaternion_from_euler(
                0, 0, self.sys.get_config('LeftLegRobotTheta').q, 'sxyz'))
            point = tuple((
                self.sys.get_config('LeftLegRobotX').q,
                self.sys.get_config('LeftLegRobotY').q,
                self.sys.get_config('LeftLegRobotZ').q
                ))
            self.br.sendTransform(point, quat,
                                  tnow,
                                  'input4', 'world')
            # right leg input
            quat = tuple(tf.transformations.quaternion_from_euler(
                0, 0, self.sys.get_config('RightLegRobotTheta').q, 'sxyz'))
            point = tuple((
                self.sys.get_config('RightLegRobotX').q,
                self.sys.get_config('RightLegRobotY').q,
                self.sys.get_config('RightLegRobotZ').q
                ))
            self.br.sendTransform(point, quat,
                                  tnow,
                                  'input5', 'world')
        # update the constraint visualization:
        mlist = []
        for c in self.con_vis:
            c.update_points()
            mlist.append(c.marker)
        ma = VM.MarkerArray()
        ma.markers = mlist
        self.con_pub.publish(mlist)



def main():
    """
    Run the main loop, by instatiating a PuppetSimulator, and then
    calling ros.spin
    """
    rospy.init_node('puppet_simulator', log_level=rospy.INFO)

    # check what the value of the legs param is, and if we are running the legs,
    # add the corresponding constraints
    legs_bool = rospy.get_param('legs', False)
    if not rospy.has_param('legs'):
        rospy.set_param('legs', legs_bool)

    # if we are using legs, add constraints:
    if legs_bool:
        trep.constraints.Distance(pd.system, 'LeftKneeHook', 'LeftLegRobotCenterPOV', 'LeftLegString')
        trep.constraints.Distance(pd.system, 'RightKneeHook', 'RightLegRobotCenterPOV', 'RightLegString')
    try:
        sim = PuppetSimulator(pd.system)
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
