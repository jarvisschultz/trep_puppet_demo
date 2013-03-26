#!/usr/bin/env python
import trep
from trep import tx, ty, tz, rx, ry, rz
from math import sin, cos
from math import pi as mpi


################################################################################
# Define all constants associated with the stl model, and the actual puppets
################################################################################
stl_torso_width = 2.6  # Based on shoulder joints
stl_torso_width_2 = 1.0  # Width between hip joints
stl_torso_height_1 = 0.4  # Based on shoulder joints
stl_torso_height_2 = 3.0
stl_torso_height = stl_torso_height_1 + stl_torso_height_2
stl_humerus_length = 1.9
stl_radius_length = 2.001
stl_femur_length = 3.0
stl_tibia_length = 3.0
stl_head_length = 2.0

torso_width = 0.3 # Width between shoulder joints
torso_width_1 = 0.25 # Width between the shoulder hooks
torso_width_2 = 0.13 # Width between hips
torso_height = 0.37 # Vertical distance between shoulder and hip joints
torso_height_1 = torso_height/stl_torso_height*stl_torso_height_1 # Height to shoulder joints
torso_height_2 = torso_height/stl_torso_height*stl_torso_height_2 # Height to hip joints
torso_height_3 = 0.41-torso_height_2 # Height to shoulder hooks
torso_height_4 = 0.47-torso_height_2 # Height to neck joint
humerus_length = 0.25
radius_length = 0.25
hand_length = 0.18
femur_length = 0.35
tibia_length = 0.32
head_length = 0.18
body_robot_width=0.18

humerus_length=0.2
radius_length=0.15
hand_length=0.05

stl_scaling = {}
scale = [0,0,0]
scale[0] = torso_width/stl_torso_width
scale[2] = torso_height/stl_torso_height
scale[1] = (scale[0] + scale[2])/2.0
stl_scaling['Torso'] = scale
scale = [0,0,0]
scale[0] = humerus_length/stl_humerus_length
scale[1] = scale[0]
scale[2] = scale[0]
stl_scaling['Humerus'] = scale
scale = [0,0,0]
scale[0] = radius_length/stl_radius_length
scale[1] = radius_length/stl_radius_length
scale[2] = (radius_length+hand_length)/stl_radius_length
stl_scaling['Radius'] = scale
scale = [0,0,0]
scale[0] = femur_length/stl_femur_length
scale[1] = scale[0]
scale[2] = scale[0]
stl_scaling['Femur'] = scale
scale = [0,0,0]
scale[0] = tibia_length/stl_tibia_length
scale[1] = scale[0]
scale[2] = scale[0]
stl_scaling['Tibia'] = scale
scale = [0,0,0]
scale[0] = head_length/stl_head_length
scale[1] = scale[0]
scale[2] = scale[0]
stl_scaling['Head'] = scale



################################################################################
# Define all of the constants associated with the geometry of the robots
################################################################################
W = 0.148/2.0  # 1/2 the track width in meters
D = 3.0*0.0254 # wheel diamater in meters
# transform from center of geometry of the cover out to the string hooks.  The
# base frame has x forward, y out the left side of the robot, and z out the top
# (all in the robot's pov).
cover_to_left_string = [-0.010, 0.039, 0.041]
cover_to_right_string = [-0.010, -0.039, 0.041]



################################################################################
# Now we are ready to define the system
################################################################################
system = trep.System()
frames = [
    ###### PUPPET ######
    tx('TorsoX'), [ty('TorsoY'), [tz('TorsoZ'), [
        rz('TorsoPsi'), [ry('TorsoTheta'), [rz('TorsoPhi',name='Torso'), [
            tz(-torso_height_2/2, mass=(0.227+0.146, 0.005, 0.005, 0.005)),
            tx(-torso_width_1/2), [tz(torso_height_3, name='Right Torso Hook')],
            tx( torso_width_1/2), [tz(torso_height_3, name= 'Left Torso Hook')],
            tz(torso_height_4, name='Head'), [tz(head_length/2, mass=0.081)],
            # Define the left arm
            tx(torso_width/2), [tz(torso_height_1), [
                rz('LShoulderPsi'), [ry('LShoulderTheta'), [rx('LShoulderPhi', name='Left Shoulder'), [
                    tz(-humerus_length/2, name='Left Humerus', mass=(0.019, 0.01, 0.01, 0.01)),
                    tz(-humerus_length), [
                        rx('LElbowPhi', name='Left Elbow'), [
                            tz(-radius_length/2, name='Left Radius', mass=(0.021, 0.01, 0.01, 0.01)),
                            tz(-radius_length), [
                                tz(-hand_length/2, mass=0.025),
                                tz(-hand_length, name='Left Finger')]]]]]]]],
            # Define the right arm
            tx(-torso_width/2), [tz(torso_height_1), [
                rz('RShoulderPsi'), [ry('RShoulderTheta'), [rx('RShoulderPhi', name='Right Shoulder'), [
                    tz(-humerus_length/2, name='Right Humerus', mass=(0.019, 0.01, 0.01, 0.01)),
                    tz(-humerus_length), [
                        rx('RElbowPhi', name='Right Elbow'), [
                            tz(-radius_length/2, name='Right Radius', mass=(0.021, 0.01, 0.01, 0.01)),
                            tz(-radius_length), [
                            tz(-hand_length/2, mass=0.025),
                            tz(-hand_length, name='Right Finger')]]]]]]]],

            # Define the left leg
            tx(torso_width_2/2), [tz(-torso_height_2), [
                rz('LHipPsi'), [ry('LHipTheta'), [rx('LHipPhi', name='Left Hip'), [
                    tz(-femur_length/2, name='Left Femur', mass=0.034),##
                    tz(-femur_length*0.86333), [ty(-femur_length*0.1073, name='Left Knee Hook')],##
                    tz(-femur_length), [##
                        rx('LKneeTheta', name='Left Knee'), [
                            tz(-tibia_length/2, name='Left Tibia', mass=0.031+0.023)]]]]]]],
            # Define the right leg
            tx(-torso_width_2/2), [tz(-torso_height_2), [
                rz('RHipPsi'), [ry('RHipTheta'), [rx('RHipPhi', name='Right Hip'), [
                    tz(-femur_length/2, name='Right Femur', mass=0.034),##
                    tz(-femur_length*0.86333), [ty(-femur_length*0.1073, name='Right Knee Hook')],##
                    tz(-femur_length), [##
                        rx('RKneeTheta', name='right Knee'), [
                            tz(-tibia_length/2, name='Right Tibia', mass=0.031+0.023)]]]]]]],
          ]]]]]],  # End of puppet definition

    # define all robots:
    # first do the body robot:
    tx('BodyRobotX', kinematic=True), [ty('BodyRobotY', kinematic=True), [
        tz('BodyRobotZ', kinematic=True),[rz('BodyRobotTheta', kinematic=True, name='Body Robot Center'), [
            rx(mpi, name='Body Robot Center POV'), [
                tx(cover_to_left_string[0]), [ty(cover_to_left_string[1]), [tz(cover_to_left_string[2], name='Body Robot Left Spindle')]],
                tx(cover_to_right_string[0]), [ty(cover_to_right_string[1]), [tz(cover_to_right_string[2], name='Body Robot Right Spindle')]]
                ]]]]],
    # left robot
    tx('LeftRobotX', kinematic=True), [ty('LeftRobotY', kinematic=True), [
        tz('LeftRobotZ', kinematic=True), [rz('LeftRobotTheta', kinematic=True, name='Left Robot Center'), [
            rx(mpi, name='Left Robot Center POV'), [
                tx(cover_to_left_string[0]), [ty(cover_to_left_string[1]), [tz(cover_to_left_string[2], name='Left Robot Left Spindle')]],
                tx(cover_to_right_string[0]), [ty(cover_to_right_string[1]), [tz(cover_to_right_string[2], name='Left Robot Right Spindle')]]
                ]]]]],
    # right robot:
    tx('RightRobotX', kinematic=True), [ty('RightRobotY', kinematic=True), [
        tz('RightRobotZ', kinematic=True), [rz('RightRobotTheta', kinematic=True, name='Right Robot Center'), [
            rx(mpi, name='Right Robot Center POV'), [
                tx(cover_to_left_string[0]), [ty(cover_to_left_string[1]), [tz(cover_to_left_string[2], name='Right Robot Left Spindle')]],
                tx(cover_to_right_string[0]), [ty(cover_to_right_string[1]), [tz(cover_to_right_string[2], name='Right Robot Right Spindle')]]
                ]]]]]
    ]

system.import_frames(frames)

trep.potentials.Gravity(system, (0, 0, -9.8))
trep.forces.Damping(system, 0.01)
# Define the strings
trep.constraints.Distance(system, 'Left Torso Hook', 'Body Robot Right Spindle', 'LeftShoulderString')
trep.constraints.Distance(system, 'Right Torso Hook', 'Body Robot Left Spindle', 'RightShoulderString')
trep.constraints.Distance(system, 'Left Finger', 'Left Robot Right Spindle', 'LeftArmString')
trep.constraints.Distance(system, 'Right Finger', 'Right Robot Left Spindle', 'RightArmString')
# trep.constraints.Distance(system, 'Left Knee Hook', 'Left Robot Left Spindle', 'Left Leg String')
# trep.constraints.Distance(system, 'Right Knee Hook', 'Right Robot Right Spindle', 'Right Leg String')
