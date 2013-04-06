# this file just defines all of the frame mappings:

class FrameMap:
    def __init__(self, inp_f, pup_f, rob_n, rob_f, con_f, user_f):
        self.input_frame = inp_f
        self.puppet_frames = pup_f
        self.robot_name = rob_n
        self.robot_frame = rob_f
        self.con_frame = con_f
        self.user_frame = user_f

frame_map = {}
frame_map[1] = FrameMap('input1',
                        ['left_shoulder_hook', 'right_shoulder_hook'],
                        'BodyRobot',
                        'BodyRobotCenterPOV',
                        'body_input',
                        'head')
frame_map[2] = FrameMap('input2',
                        'left_shoulder_hook',
                        'LeftShoulderRobot',
                        'LeftShoulderRobotCenterPOV',
                        'left_shoulder_input',
                        'left_shoulder')
frame_map[3] = FrameMap('input3',
                        'right_shoulder_hook',
                        'RightShoulderRobot',
                        'RightShoulderRobotCenterPOV',
                        'right_shoulder_input',
                        'right_shoulder')
frame_map[4] = FrameMap('input4',
                        'left_hand_hook',
                        'LeftRobot',
                        'LeftRobotCenterPOV',
                        'left_hand_input',
                        'left_hand')
frame_map[5] = FrameMap('input5',
                        'right_hand_hook',
                        'RightRobot',
                        'RightRobotCenterPOV',
                        'right_hand_input',
                        'right_hand')
frame_map[6] = FrameMap('input6',
                        'left_knee_hook',
                        'LeftLegRobot',
                        'LeftLegRobotCenterPOV',
                        'left_knee_input',
                        'left_knee')
frame_map[7] = FrameMap('input7',
                        'right_knee_hook',
                        'RightLegRobot',
                        'RightLegRobotCenterPOV',
                        'right_knee_input',
                        'right_knee')
                        
