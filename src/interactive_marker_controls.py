#!/usr/bin/env python
import rospy
import copy
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import PoseStamped as PS
import std_srvs.srv as SS
from geometry_msgs.msg import Pose as P
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import tf
from math import sin
from frame_mappings import frame_map as fm

# global constants
DT = 1/100.
SIMWF = 'world'


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
class SingleController:
    def __init__(self, conframe, simframe, simpos=(0.0,)*3,
                 simquat=(0.0,)*4, simpose=None, color='green'):
        """
        conframe ~ the frame that we should publish to control the kinematic
              input
        simframe ~ the frame that the simpos and simquat point to
        simpos ~ nominal location of the kinematic config variable in trep
              simulation... used for determining offset
        simquat ~ nominal orientation of the kinematic config var in the trep
              simulation... used for offset
        simpose ~ a pose message to define the pose
        color ~ color of the marker attached to the frame
        """
        self.conframe = conframe
        self.simframe = simframe
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = SIMWF
        if simpose != None:
            self.int_marker.pose = simpose
        else:
            # build pose from pos and quat
            self.set_pose(pos=simpos, quat=simquat)
        self.int_marker.scale = 0.25
        self.int_marker.name = simframe
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

    def set_pose(self, pose=None, pos=(0.0,)*3, quat=(0.0,)*4):
        if pose != None:
            self.int_marker.header.frame_id = pose.header.frame_id
            self.int_marker.pose = pose
            self.simpose = copy.deepcopy(self.int_marker.pose)
            self.simpos = tuple([pose.position.__getattribute__(x)
                                 for x in pose.position.__slots__])
            self.simquat = tuple([pose.orientation.__getattribute__(x)
                                  for x in pose.orientation.__slots__])
        else:
            self.simpos = pos
            self.simquat = quat
            self.int_marker.pose = P(position=Point(*pos),
                                     orientation=Quaternion(*quat))
            self.simpose = P(position=Point(*pos),
                                     orientation=Quaternion(*quat))


class MarkerControls:
    def __init__(self):
        self.legs_bool = rospy.get_param('legs', False)
        self.shoulders_bool = rospy.get_param('shoulders', False)
        # create marker server:
        self.server = InteractiveMarkerServer("puppet_controls")
        # create listener and broadcaster
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        # let's build all of the controllers, and then get the necessary transforms:
        self.controllers = []
        for i in 4,5:
            self.controllers.append(SingleController(fm[i].con_frame,
                                                     fm[i].input_frame,
                                                     color='green'))
        if self.shoulders_bool:
            for i in 2,3:
                self.controllers.append(SingleController(fm[i].con_frame,
                                                         fm[i].input_frame,
                                                         color='red'))
        else:
            self.controllers.append(SingleController(fm[1].con_frame,
                                                     fm[1].input_frame,
                                                     color='red'))
        if self.legs_bool:
            for i in 6,7:
                self.controllers.append(SingleController(fm[i].con_frame,
                                                         fm[i].input_frame,
                                                         color='blue'))
        while True:
            if self.wait_and_update_frames():
                self.running_flag = True
                break
            rospy.logwarn("Failure of waiting... trying again")
            rospy.sleep(50*DT)
        rospy.loginfo("Found all necessary frames")

        # insert callbacks for the controllers
        for con in self.controllers:
            self.server.insert(con.int_marker, self.marker_cb)
        # actually update server for all inserted controls
        self.server.applyChanges()
        # offer a service for resetting controls:
        self.reset_srv_provider = rospy.Service('interface_reset', SS.Empty, self.reset_provider)
        # setup timer to publish transforms for all inputs
        rospy.Timer(rospy.Duration(DT), self.send_transforms)


    def wait_and_update_frames(self):
        for i,con in enumerate(self.controllers):
            rospy.loginfo("Waiting for transform to "+con.simframe)
            for i in range(100):
                if self.listener.canTransform(SIMWF, con.simframe, rospy.Time()):
                    continue
                rospy.sleep(5*DT)
            rospy.loginfo("Found transform from {0:s} to {1:s}".format(
                    SIMWF, con.simframe))
            try:
                pos, quat = self.listener.lookupTransform(SIMWF, con.simframe, rospy.Time())
                con.set_pose(pos=pos, quat=quat)
            except (tf.Exception):
                rospy.loginfo("Could not find transform from {0:s} to {1:s}".format(
                    SIMWF, con.simframe))
                return False
        return True


    def reset_provider(self, req):
        rospy.loginfo("Controls reset!")
        # simulator has requested that we reset all controls
        for con in self.controllers:
            con.set_pose(pos=con.simpos, quat=con.simquat)
            self.server.setPose(con.int_marker.name, con.simpose)
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
        for con in self.controllers:
            pos = con.int_marker.pose.position
            quat = con.int_marker.pose.orientation
            frame = con.conframe
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
