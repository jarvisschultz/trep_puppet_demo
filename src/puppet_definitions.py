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

torso_width = 2*.09525 # Width between shoulder joints
torso_width_1 = 0.125 # Width between the shoulder hooks
torso_width_2 = 0.13 # Width between hips
torso_height = 0.249 # Vertical distance between shoulder and hip joints
torso_height_1 = torso_height/stl_torso_height*stl_torso_height_1 # Height to shoulder joints
torso_height_2 = torso_height/stl_torso_height*stl_torso_height_2 # Height to hip joints
torso_height_3 = 0.41-torso_height_2 # Height to shoulder hooks
torso_height_4 = 0.47-torso_height_2 # Height to neck joint
humerus_length = 0.1778
radius_length = 0.1524
hand_length = 0.098
femur_length = 0.238
tibia_length = 0.254
head_length = 0.142
body_robot_width=0.18

# humerus_length=0.2
# radius_length=0.15
# hand_length=0.05

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


#############################
# define all of the masses  #
#############################
torso_mass = (0.780, 0.005, 0.005, 0.005)
head_mass = 0.081
humerus_mass = 0.0742
radius_mass = 0.0578
hand_mass = 0.0581
femur_mass = 0.1054
tibia_mass = 0.1585



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
        rz('TorsoPsi'), [ry('TorsoTheta'), [rx('TorsoPhi',name='Torso'), [
            tz(-torso_height_2/2, mass=torso_mass),
            tx(-torso_width_1/2), [tz(torso_height_3, name='RightTorsoHook')],
            tx( torso_width_1/2), [tz(torso_height_3, name= 'LeftTorsoHook')],
            tz(torso_height_4, name='Head'), [tz(head_length/2, mass=head_mass)],
            # Define the left arm
            tx(torso_width/2), [tz(torso_height_1), [
                rz('LShoulderPsi'), [ry('LShoulderTheta'), [rx('LShoulderPhi', name='LeftShoulder'), [
                    tz(-humerus_length/2, name='LeftHumerus', mass=humerus_mass),
                    tz(-humerus_length), [
                        rx('LElbowPhi', name='LeftElbow'), [
                            tz(-radius_length/2, name='LeftRadius', mass=radius_mass),
                            tz(-radius_length), [
                                tz(-hand_length/2, mass=hand_mass),
                                tz(-hand_length, name='LeftFinger')]]]]]]]],
            # Define the right arm
            tx(-torso_width/2), [tz(torso_height_1), [
                rz('RShoulderPsi'), [ry('RShoulderTheta'), [rx('RShoulderPhi', name='RightShoulder'), [
                    tz(-humerus_length/2, name='RightHumerus', mass=humerus_mass),
                    tz(-humerus_length), [
                        rx('RElbowPhi', name='RightElbow'), [
                            tz(-radius_length/2, name='RightRadius', mass=radius_mass),
                            tz(-radius_length), [
                            tz(-hand_length/2, mass=hand_mass),
                            tz(-hand_length, name='RightFinger')]]]]]]]],

            # Define the left leg
            tx(torso_width_2/2), [tz(-torso_height_2), [
                rz('LHipPsi'), [ry('LHipTheta'), [rx('LHipPhi', name='LeftHip'), [
                    tz(-femur_length/2, name='LeftFemur', mass=femur_mass),##
                    tz(-femur_length*0.86333), [ty(-femur_length*0.1073, name='LeftKneeHook')],##
                    tz(-femur_length), [##
                        rx('LKneePhi', name='LeftKnee'), [
                            tz(-tibia_length/2, name='LeftTibia', mass=tibia_mass)]]]]]]],
            # Define the right leg
            tx(-torso_width_2/2), [tz(-torso_height_2), [
                rz('RHipPsi'), [ry('RHipTheta'), [rx('RHipPhi', name='RightHip'), [
                    tz(-femur_length/2, name='RightFemur', mass=femur_mass),
                    tz(-femur_length*0.86333), [ty(-femur_length*0.1073, name='RightKneeHook')],##
                    tz(-femur_length), [##
                        rx('RKneePhi', name='RightKnee'), [
                            tz(-tibia_length/2, name='RightTibia', mass=tibia_mass)]]]]]]],
          ]]]]]],  # End of puppet definition

    # define all robots:
    # first do the body robot:
    tx('BodyRobotX', kinematic=True), [ty('BodyRobotY', kinematic=True), [
        tz('BodyRobotZ', kinematic=True),[rz('BodyRobotTheta', kinematic=True, name='BodyRobotCenter'), [
            rx(mpi, name='BodyRobotCenterPOV'), [
                tx(cover_to_left_string[0]), [ty(cover_to_left_string[1]), [tz(cover_to_left_string[2], name='BodyRobotLeftSpindle')]],
                tx(cover_to_right_string[0]), [ty(cover_to_right_string[1]), [tz(cover_to_right_string[2], name='BodyRobotRightSpindle')]]
                ]]]]],
    # left robot
    tx('LeftRobotX', kinematic=True), [ty('LeftRobotY', kinematic=True), [
        tz('LeftRobotZ', kinematic=True), [rz('LeftRobotTheta', kinematic=True, name='LeftRobotCenter'), [
            rx(mpi, name='LeftRobotCenterPOV'), [
                tx(cover_to_left_string[0]), [ty(cover_to_left_string[1]), [tz(cover_to_left_string[2], name='LeftRobotLeftSpindle')]],
                tx(cover_to_right_string[0]), [ty(cover_to_right_string[1]), [tz(cover_to_right_string[2], name='LeftRobotRightSpindle')]]
                ]]]]],
    # right robot:
    tx('RightRobotX', kinematic=True), [ty('RightRobotY', kinematic=True), [
        tz('RightRobotZ', kinematic=True), [rz('RightRobotTheta', kinematic=True, name='RightRobotCenter'), [
            rx(mpi, name='RightRobotCenterPOV'), [
                tx(cover_to_left_string[0]), [ty(cover_to_left_string[1]), [tz(cover_to_left_string[2], name='RightRobotLeftSpindle')]],
                tx(cover_to_right_string[0]), [ty(cover_to_right_string[1]), [tz(cover_to_right_string[2], name='RightRobotRightSpindle')]]
                ]]]]],
    # left leg robot
    tx('LeftLegRobotX', kinematic=True), [ty('LeftLegRobotY', kinematic=True), [
        tz('LeftLegRobotZ', kinematic=True), [rz('LeftLegRobotTheta', kinematic=True, name='LeftLegRobotCenter'), [
            rx(mpi, name='LeftLegRobotCenterPOV'), [
                tx(cover_to_left_string[0]), [ty(cover_to_left_string[1]), [tz(cover_to_left_string[2], name='LeftLegRobotLeftSpindle')]],
                tx(cover_to_right_string[0]), [ty(cover_to_right_string[1]), [tz(cover_to_right_string[2], name='LeftLegRobotRightSpindle')]]
                ]]]]],
    # right leg robot:
    tx('RightLegRobotX', kinematic=True), [ty('RightLegRobotY', kinematic=True), [
        tz('RightLegRobotZ', kinematic=True), [rz('RightLegRobotTheta', kinematic=True, name='RightLegRobotCenter'), [
            rx(mpi, name='RightLegRobotCenterPOV'), [
                tx(cover_to_left_string[0]), [ty(cover_to_left_string[1]), [tz(cover_to_left_string[2], name='RightLegRobotLeftSpindle')]],
                tx(cover_to_right_string[0]), [ty(cover_to_right_string[1]), [tz(cover_to_right_string[2], name='RightLegRobotRightSpindle')]]
                ]]]]]
    ]

system.import_frames(frames)

trep.potentials.Gravity(system, (0, 0, -9.8))
# define the system damping:
zeta = 0.20
damp_dict = {
    #humerus 
    'LShoulderPsi' : humerus_mass/torso_mass[0]*zeta,
    'LShoulderTheta' : humerus_mass/torso_mass[0]*zeta,
    'LShoulderPhi' : humerus_mass/torso_mass[0]*zeta,
    'RShoulderPsi' : humerus_mass/torso_mass[0]*zeta,
    'RShoulderTheta' : humerus_mass/torso_mass[0]*zeta,
    'RShoulderPhi' : humerus_mass/torso_mass[0]*zeta,
    #radius
    'LElbowPhi' : (hand_mass+radius_mass)/torso_mass[0]*zeta,
    'RElbowPhi' : (hand_mass+radius_mass)/torso_mass[0]*zeta,
    #femur
    'LHipPsi' : femur_mass/torso_mass[0]*zeta,
    'LHipTheta' : femur_mass/torso_mass[0]*zeta,
    'LHipPhi' : femur_mass/torso_mass[0]*zeta,
    'RHipPsi' : femur_mass/torso_mass[0]*zeta,
    'RHipTheta' : femur_mass/torso_mass[0]*zeta,
    'RHipPhi' : femur_mass/torso_mass[0]*zeta,
    #tibia
    'LKneePhi' : tibia_mass/torso_mass[0]*zeta,
    'RKneePhi' : tibia_mass/torso_mass[0]*zeta,
}
trep.forces.Damping(system, zeta, damp_dict)

# Define the strings
trep.constraints.Distance(system, 'LeftTorsoHook', 'BodyRobotRightSpindle', 'LeftShoulderString')
trep.constraints.Distance(system, 'RightTorsoHook', 'BodyRobotLeftSpindle', 'RightShoulderString')
trep.constraints.Distance(system, 'LeftFinger', 'LeftRobotCenterPOV', 'LeftArmString')
trep.constraints.Distance(system, 'RightFinger', 'RightRobotCenterPOV', 'RightArmString')
