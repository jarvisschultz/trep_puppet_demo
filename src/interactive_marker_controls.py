#!/usr/bin/env python
import roslib; roslib.load_manifest("trep_puppet_demo")
import rospy
import copy
from interactive_markers.interactive_marker_server import *
from geometry_msgs.msg import PoseStamped as PS
import std_srvs.srv as SS
from geometry_msgs.msg import Pose as P
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import tf
from math import sin

DT = 1/100.


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

def makeMarkerControl( msg , color ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeMarker(msg, color) )
    msg.controls.append( control )
    return control

# make a base class for defining a single marker:
class SingleControl:
    def __init__(self, pose, name, color):
        """
        pass in a PoseStamped defining the initial pose, and the name of the
        frame that control will point to
        """
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = pose.header.frame_id
        self.int_marker.pose = pose.pose
        self.int_marker.scale = 0.25
        self.int_marker.name = name
        self.int_marker.description = "Move string endpoint"

        makeMarkerControl(self.int_marker, color)

        self.control = InteractiveMarkerControl()
        self.control.orientation.w = 1
        self.control.orientation.x = 0
        self.control.orientation.y = 1
        self.control.orientation.z = 0
        self.control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(copy.deepcopy(self.control))
        self.control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        self.int_marker.controls.append(self.control)

    def set_pose(self, pose):
        self.int_marker.header.frame_id = pose.header.frame_id
        self.int_marker.pose = pose.pose


class MarkerControls:
    def __init__(self):
        # create marker server:
        self.server = InteractiveMarkerServer("puppet_controls")
        # create listener and broadcaster
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        # are we using the legs
        self.legs_bool = rospy.get_param('legs', False)
        # create control instances:
        pos1 = None; pos2 = None; pos3 = None; pos4 = None; pos5 = None; pos6 = None;
        quat1 = None; quat2 = None; quat3 = None; quat4 = None; quat5 = None; quat6 = None;
        for i in range(10):
            try:
                pos1,quat1 = self.listener.lookupTransform("world", "input1", rospy.Time())
                pos2,quat2 = self.listener.lookupTransform("world", "input2", rospy.Time())
                pos3,quat3 = self.listener.lookupTransform("world", "input3", rospy.Time())
                pos6,quat6 = self.listener.lookupTransform("world", "input6", rospy.Time())
                if self.legs_bool:
                    pos4,quat4 = self.listener.lookupTransform("world", "input4", rospy.Time())
                    pos5,quat5 = self.listener.lookupTransform("world", "input5", rospy.Time())
            except (tf.Exception):
                rospy.logwarn("Could not find input transforms!")
            rospy.sleep(0.5)
        if not all([pos1,pos2,pos3]):
            rospy.signal_shutdown("Could not initialize tf for inputs")
        # body controller
        ## ptmp = P(position=Point(*pos1), orientation=Quaternion(*quat1))
        ## self.p1 = PS(pose=ptmp)
        ## self.p1.header.frame_id = "world"
        ## self.c1 = SingleControl(self.p1, "body_input", 'blue')
        # shoulder controllers
        #left
        ptmp = P(position=Point(*pos1), orientation=Quaternion(*quat1))
        self.p1 = PS(pose=ptmp)
        self.p1.header.frame_id = "world"
        self.c1 = SingleControl(self.p1, "left_shoulder_input", 'blue')
        #right
        ptmp = P(position=Point(*pos6), orientation=Quaternion(*quat6))
        self.p6 = PS(pose=ptmp)
        self.p6.header.frame_id = "world"
        self.c6 = SingleControl(self.p6, "right_shoulder_input", 'blue')
        # left controller
        ptmp = P(position=Point(*pos2), orientation=Quaternion(*quat2))
        self.p2 = PS(pose=ptmp)
        self.p2.header.frame_id = "world"
        self.c2 = SingleControl(self.p2, "left_input", 'green')
        # right controller
        ptmp = P(position=Point(*pos3), orientation=Quaternion(*quat3))
        self.p3 = PS(pose=ptmp)
        self.p3.header.frame_id = "world"
        self.c3 = SingleControl(self.p3, "right_input", 'green')
        if self.legs_bool:
            # left leg controller
            ptmp = P(position=Point(*pos4), orientation=Quaternion(*quat4))
            self.p4 = PS(pose=ptmp)
            self.p4.header.frame_id = "world"
            self.c4 = SingleControl(self.p4, "left_leg_input", 'red')
            # right leg controller
            ptmp = P(position=Point(*pos5), orientation=Quaternion(*quat5))
            self.p5 = PS(pose=ptmp)
            self.p5.header.frame_id = "world"
            self.c5 = SingleControl(self.p5, "right_leg_input", 'red')
        
        # insert callbacks for controls
        self.server.insert(self.c1.int_marker, self.marker_cb)
        self.server.insert(self.c2.int_marker, self.marker_cb)
        self.server.insert(self.c3.int_marker, self.marker_cb)
        self.server.insert(self.c6.int_marker, self.marker_cb)
        if self.legs_bool:
            self.server.insert(self.c4.int_marker, self.marker_cb)
            self.server.insert(self.c5.int_marker, self.marker_cb)
        # actually update server for all inserted controls
        self.server.applyChanges()
        # offer a service for resetting controls:
        self.reset_srv_provider = rospy.Service('interface_reset', SS.Empty, self.reset_provider)
        # setup timer to publish transforms for all inputs
        rospy.Timer(rospy.Duration(DT), self.send_transforms)


    def reset_provider(self, req):
        rospy.loginfo("Controls reset!")
        # simulator has requested that we reset all controls
        self.c1.set_pose(self.p1)
        self.c2.set_pose(self.p2)
        self.c3.set_pose(self.p3)
        self.c6.set_pose(self.p6)
        if self.legs_bool:
            self.c4.set_pose(self.p4)
            self.c5.set_pose(self.p5)
        self.server.setPose(self.c1.int_marker.name, self.p1.pose)
        self.server.setPose(self.c2.int_marker.name, self.p2.pose)
        self.server.setPose(self.c3.int_marker.name, self.p3.pose)
        self.server.setPose(self.c6.int_marker.name, self.p6.pose)
        if self.legs_bool:
            self.server.setPose(self.c4.int_marker.name, self.p4.pose)
            self.server.setPose(self.c5.int_marker.name, self.p5.pose)
        self.server.applyChanges()
        rospy.sleep(5*DT)
        # reply with response:
        return SS.EmptyResponse()

    
    def marker_cb(self, feedback):
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # rospy.loginfo( s + ": pose changed")
            pass
        self.server.applyChanges()


    def send_transforms(self, event):
        tnow = rospy.Time.now()
        ## # body control
        ## pos = self.c1.int_marker.pose.position
        ## quat = self.c1.int_marker.pose.orientation
        ## frame = self.c1.int_marker.name
        ## self.br.sendTransform((pos.x, pos.y, pos.z),
        ##                       (quat.x, quat.y, quat.z, quat.w),
        ##                       tnow,
        ##                       frame, 'world')
        # shoulder controls
        #left
        pos = self.c1.int_marker.pose.position
        quat = self.c1.int_marker.pose.orientation
        frame = self.c1.int_marker.name
        self.br.sendTransform((pos.x, pos.y, pos.z),
                              (quat.x, quat.y, quat.z, quat.w),
                              tnow,
                              frame, 'world')
        #right
        pos = self.c6.int_marker.pose.position
        quat = self.c6.int_marker.pose.orientation
        frame = self.c6.int_marker.name
        self.br.sendTransform((pos.x, pos.y, pos.z),
                              (quat.x, quat.y, quat.z, quat.w),
                              tnow,
                              frame, 'world')
        # left control
        pos = self.c2.int_marker.pose.position
        quat = self.c2.int_marker.pose.orientation
        frame = self.c2.int_marker.name
        self.br.sendTransform((pos.x, pos.y, pos.z),
                              (quat.x, quat.y, quat.z, quat.w),
                              tnow,
                              frame, 'world')
        # right control
        pos = self.c3.int_marker.pose.position
        quat = self.c3.int_marker.pose.orientation
        frame = self.c3.int_marker.name
        self.br.sendTransform((pos.x, pos.y, pos.z),
                              (quat.x, quat.y, quat.z, quat.w),
                              tnow,
                              frame, 'world')
        if self.legs_bool:
            # left leg control
            pos = self.c4.int_marker.pose.position
            quat = self.c4.int_marker.pose.orientation
            frame = self.c4.int_marker.name
            self.br.sendTransform((pos.x, pos.y, pos.z),
                                  (quat.x, quat.y, quat.z, quat.w),
                                  tnow,
                                  frame, 'world')
            # right control
            pos = self.c5.int_marker.pose.position
            quat = self.c5.int_marker.pose.orientation
            frame = self.c5.int_marker.name
            self.br.sendTransform((pos.x, pos.y, pos.z),
                                  (quat.x, quat.y, quat.z, quat.w),
                                  tnow,
                                  frame, 'world')




def main():
    rospy.init_node('marker_controls')#, log_level=rospy.INFO)

    try:
        sim = MarkerControls()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
