#!/usr/bin/env python
import roslib; roslib.load_manifest("trep_puppet_demo")
import rospy
import copy
from interactive_markers.interactive_marker_server import *
from geometry_msgs.msg import PoseStamped as PS
from geometry_msgs.msg import Pose as P
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import tf
from math import sin

DT = 1/100.


def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.1
    marker.color.g = 0.5
    marker.color.b = 0.1
    marker.color.a = 0.75

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

# make a base class for defining a single marker:
class SingleControl:
    def __init__(self, pose, name):
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

        makeBoxControl(self.int_marker)

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
        self.int_marker.pose = pose


class MarkerControls:
    def __init__(self):
        # create marker server:
        self.server = InteractiveMarkerServer("puppet_controls")
        # create listener and broadcaster
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        # create control instances:
        pos = None
        quat = None
        for i in range(10):
            try:
                pos,quat = self.listener.lookupTransform("world", "input2", rospy.Time())
            except (tf.Exception):
                rospy.logwarn("Could not find transform to optimization_frame after waiting!")
            rospy.sleep(0.5)
        if pos is None or quat is None:
            rospy.signal_shutdown("Could not initialize tf for inputs")
        ptmp = P(position=Point(*pos), orientation=Quaternion(*quat))
        p = PS(pose=ptmp)
        p.header.frame_id = "world"
        self.c1 = SingleControl(p, "left_input")

        # insert callbacks for controls
        self.server.insert(self.c1.int_marker, self.marker_cb)
        # actually update server for all inserted controls
        self.server.applyChanges()

        # setup timer to publish transforms for all inputs
        rospy.Timer(rospy.Duration(DT), self.send_transforms)


    def marker_cb(self, feedback):
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # rospy.loginfo( s + ": pose changed")
            pass
        self.server.applyChanges()


    def send_transforms(self, event):
        tnow = rospy.Time.now()
        pos = self.c1.int_marker.pose.position
        quat = self.c1.int_marker.pose.orientation
        frame = self.c1.int_marker.name
        self.br.sendTransform((pos.x, pos.y, pos.z),
                              (quat.x, quat.y, quat.z, quat.w),
                              tnow,
                              frame, 'world')



def main():
    rospy.init_node('marker_controls', log_level=rospy.INFO)

    try:
        sim = MarkerControls()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
