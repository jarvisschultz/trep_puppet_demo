import trep
from trep import rx, ry, rz, tx, ty, tz, const_txyz

from OpenGL.GL import *
from OpenGL.GLU import *


# Default dimensions, originated from Pygmalion motion capture data
DEFAULT_DIMENSIONS = {
    'ltibia_length' : 0.254,
    #ltibia_length - Distance between the left knee axis and left
    #ankle axis
    'upper_torso_length' : 0.249,
    # upper_torso_length - Distance between the center of the hips and
    # center of the shoulders
    'lshoulder_width' : 0.09525,
    # lshoulder_width - Distance from the center of the shoulders and
    # the left shoulder axis
    'final_head_length' : 0.0508,
    # final_head_length - Distance between the head's center of mass
    # and the top
    'lradius_length' : 0.1524,
    # lradius_length - Distance between the left elbow axis and left
    # wrist axis 
	### EDIT: We have no wrist axis right now.  This is the distance 
	### from the left elbow axis to the center of the wrist screw
    'rshoulder_width' : 0.09525,
    # rshoulder_width - Distance from the center of the shoulders and
    # the right shoulder axis
    'rfoot_length' : 0.09525,
    # rfoot_length - Length of the right foot
    'rradius_length' : 0.1524,
    # rradius_length - Distance between the right elbow axis and the
    # right wrist axis
    'lfoot_length' : 0.09525,
    # lfoot_length - Length of the left foot
    'rtibia_length' : 0.254,
    # rtibia_length - Distance between the right knee axis and right
    # ankle axis
    'rhip_width' : 0.0381,
    # rhip_width - Distance between the center of the hips and the
    # center of the right hip joint
    'neck_length' : 0.05715,
    # neck_length - Distance between the center of the shoulders and
    # the center of the neck joint
    'lhip_width' : 0.0381,
    # lhip_width - Distance between the center of the hips and the
    # center of the left hip joint
    'lhumerus_length' : 0.1778,
    # lhumerus_length - Distance between the center of the left
    # shoulder joint and the elbow axis
    'lower_head_length' : 0.034,
    # lower_head_length - Distance between the center of the neck
    # joint and the base of the head
    'rfemur_length' : 0.238,
    # rfemur_length - Distance between the center of the right hip
    # joint and the right knee axis
    'upper_head_length' : 0.0572,
    # upper_head_length - Distance from the base of the head to the
    # center of mass
    'lhand_length' : 0.098,
    # lhand_length - Length of the left hand
	### EDIT:  This is the length from the center of the screw to
	### the tip of the longest finger, string hole is part way
	### between the two.
    'rhumerus_length' : 0.1778,
    # rhumerus_length - Distance between the center of the right
    # shoulder joint and the elbow axis
    'rhand_length' : 0.098,
    # rhand_length - Length of the right hand
    'lfemur_length' : 0.238,
    # lfemur_length - Distance between the center of the left hip
    # joint and the left knee axis.

    # Mass properties of the puppet.  Each list is the mass and
    # rotational intertias: [M, Ixx, Iyy, Izz].
	### EDIT:  I'm not positive which axis is which, but it 
	### likely doesn't matter.  It really only shows up in the
	### Torso and Pelvis.  Also just a note that my units are kg 
	### and kg*m^2
    'pelvis_mass' : [0.401, 0.0014, 0.0014, 0.001],
    'head_mass' : [0.292, 0.0006, 0.0006, 0.0004],
    'femur_mass' : [0.1054, 0.0005, 0.0005, 0.0],
    'tibia_mass' : [0.1042, 0.0005, 0.0005, 0.0],
    'humerus_mass' : [0.0742, 0.0002, 0.0002, 0.0],
    'radius_mass' : [0.0578, 0.0001, 0.0001, 0.00],
	
	### EDIT:  In the puppets, the Torso mass and inertias, and 
	### the hand mass and inertias are likely quite important as 
	### well.
	'hand_mass' : [0.0581, 0.0001, 0.0001, 0.0001],
	'torso_mass' : [0.3787, 0.0013, 0.0008, 0.0007],
	'foot_mass' : [0.0543, 0.0001, 0.0001, 0.0001],
	

    # Damping constant for the dynamic configuration variables.
    'damping' : 0.2,

    # Height of the string plane along the Z-axis.
    'string_plane_height' : 2,

    # String definitions.  Each entry maps the name of a string to a
    # tuple describing how it attaches to the puppet.  The first entry
    # is the name of a frame in the system.  The second entry is a
    # tuple of the offset (x,y,z translations) of the attachment point
    # to the named frame.
    'strings' : {
        'upper_torso_string' : ('spine_top', (0, -0.1, 0)),
        'lower_torso_string' : ('pelvis', (0, -0.1, 0)),
        'left_arm_string' : ('lradius_end', (0, 0.1, 0)),
        'right_arm_string' : ('rradius_end', (0, 0.1, 0)),
        'left_leg_string' : ('lfemur_end', (0, 0.1, 0)),
        'right_leg_string' : ('rfemur_end', (0, 0.1, 0))
        }
    }

# Default joints, a string is the name of the configuration variable
# controlling the joint parameter, a constant is a fixed parameter.
DEFAULT_JOINTS = {
    'torso_tx' : 'torso_tx',
    'torso_ty' : 'torso_ty',
    'torso_tz' : 'torso_tz',
    'torso_rz' : 'torso_rz',
    'torso_ry' : 'torso_ry',
    'torso_rx' : 'torso_rx',
    'lhip_rz' : 'lhip_rz',
    'lhip_ry' : 'lhip_ry',
    'lhip_rx' : 'lhip_rx',
    'lknee_rx' : 'lknee_rx',
    'lfoot_rx' : 0.0,
    'lfoot_ry' : 0.0,
    'lfoot_rz' : 0.0,
    'rhip_rz' : 'rhip_rz',
    'rhip_ry' : 'rhip_ry',
    'rhip_rx' : 'rhip_rx',
    'rknee_rx' : 'rknee_rx',
    'rfoot_rx' : 0.0,
    'rfoot_ry' : 0.0,
    'rfoot_rz' : 0.0,
    'neck_rz' : 0.0,
    'neck_ry' : 0.0,
    'neck_rx' : 0.0,
    'lshoulder_rz' : 'lshoulder_rz',
    'lshoulder_ry' : 'lshoulder_ry',
    'lshoulder_rx' : 'lshoulder_rx',
    'lelbow_rx' : 'lelbow_rx',
    'lhand_rx' : 0.0,
    'lhand_ry' : 0.0,
    'lhand_rz' : 0.0,
    'rshoulder_rz' : 'rshoulder_rz',
    'rshoulder_ry' : 'rshoulder_ry',
    'rshoulder_rx' : 'rshoulder_rx',
    'relbow_rx' : 'relbow_rx',
    'rhand_rx' : 0.0,
    'rhand_ry' : 0.0,
    'rhand_rz' : 0.0
    }    

def fill_dimensions(dimensions={}):
    dim = dimensions.copy()
    for (key, default) in DEFAULT_DIMENSIONS.iteritems():
        if key not in dim:
            dim[key] = default
    return dim

def fill_joints(joints={}):
    joints = joints.copy()
    for (key, default) in DEFAULT_JOINTS.iteritems():
        if key not in joints:
            joints[key] = default
    return joints

def make_skeleton(dimensions={}, joints={}):
    dim = fill_dimensions(dimensions)
    joints = fill_joints(joints)

    frames = [
        tx(joints['torso_tx']), [ty(joints['torso_ty']), [tz(joints['torso_tz']), [
            rz(joints['torso_rz']), [ ry(joints['torso_ry']), [
                rx(joints['torso_rx'], name='pelvis',  mass=dim['pelvis_mass']), [
                    tx(-dim['lhip_width'], name='lhip'), [
                        rz(joints['lhip_rz']), [ry(joints['lhip_ry']), [rx(joints['lhip_rx'], name='lfemur'), [
                            tz(-dim['lfemur_length']/2, name='lfemur_mass', mass=dim['femur_mass']),
                            tz(-dim['lfemur_length'], name='lfemur_end'), [
                                rx(joints['lknee_rx'], name='ltibia'), [
                                    tz(-dim['ltibia_length']/2, name='ltibia_mass', mass=dim['tibia_mass']),
                                    tz(-dim['ltibia_length'], name='ltibia_end'), [
                                        rz(joints['lfoot_rz']), [ry(joints['lfoot_ry']), [
                                            rx(joints['lfoot_rx'], name='lfoot'), [
                                                ty(dim['lfoot_length'], name='lfoot_end')]]]]]]]]]],
                    tx(dim['rhip_width'], name='rhip'), [
                        rz(joints['rhip_rz']), [ry(joints['rhip_ry']), [rx(joints['rhip_rx'], name='rfemur'), [
                            tz(-dim['rfemur_length']/2, name='rfemur_mass', mass=dim['femur_mass']), 
                                tz(-dim['rfemur_length'], name='rfemur_end'), [
                                    rx(joints['rknee_rx'], name='rtibia'), [
                                        tz(-dim['rtibia_length']/2, name='rtibia_mass', mass=dim['tibia_mass']), 
                                    tz(-dim['rtibia_length'], name='rtibia_end'), [
                                        rz(joints['rfoot_rz']), [ry(joints['rfoot_ry']), [
                                            rx(joints['rfoot_rx'], name='r_foot'), [
                                                ty(dim['rfoot_length'], name='rfoot_end')]]]]]]]]]],
                    tz(dim['upper_torso_length'], name='spine_top'), [
                        tz(dim['neck_length'], name='neck'), [
                            rz(joints['neck_rz']), [ry(joints['neck_ry']), [
                                rx(joints['neck_rz'], name='neck_joint'), [
                                    tz(dim['lower_head_length'], name='head'), [
                                        tz(dim['upper_head_length'], name='head_center', mass=dim['head_mass']), [
                                            tz(dim['final_head_length'], name='head_end')]]]]]],
                        tx(-dim['lshoulder_width'], name='lshoulder'), [
                            rz(joints['lshoulder_rz']), [ry(joints['lshoulder_ry']), [
                                rx(joints['lshoulder_rx'], name='lhumerus'), [
                                    tz(-dim['lhumerus_length']/2, name='lhumerus_mass', mass=dim['humerus_mass']), 
                                    tz(-dim['lhumerus_length'], name='lhumerus_end'), [
                                        rx(joints['lelbow_rx'], name='lradius'), [
                                            tz(-dim['lradius_length']/2, name='lradius_mass', mass=dim['radius_mass']), 
                                            tz(-dim['lradius_length'], name='lradius_end'), [
                                                rz(joints['lhand_rz']), [ry(joints['lhand_ry']), [
                                                    rx(joints['lhand_rx'], name='lhand'), [
                                                        tz(-dim['lhand_length'], name='lhand_end')]]]]]]]]]],
                        tx(dim['rshoulder_width'], name='rshoulder'), [
                            rz(joints['rshoulder_rz']), [ry(joints['rshoulder_ry']), [
                                rx(joints['rshoulder_rx'], name='right_humerus'), [
                                    tz(-dim['rhumerus_length']/2, name='rhumerus_mass', mass=dim['humerus_mass']), 
                                    tz(-dim['rhumerus_length'], name='rhumerus_end'), [
                                        rx(joints['relbow_rx'], name='rradius'), [
                                            tz(-dim['rradius_length']/2, name='rradius_mass', mass=dim['radius_mass']), 
                                            tz(-dim['rradius_length'], name='rradius_end'), [
                                                rz(joints['rhand_rz']), [ry(joints['rhand_ry']), [
                                                    rx(joints['rhand_rx'], name='right_hand'), [
                                                        tz(-dim['rhand_length'], name='right_hand_end')]]]]]]]]]]
                    ]]]]]]]]
    return frames


class Puppet(trep.System):
    def __init__(self, dimensions={}, joints={}, 
                joint_forces=False,
                string_forces=False,
                string_constraints=False):

        trep.System.__init__(self)

        self.string_plane = None
        self.string_hooks = {}
        self.joint_forces = {}
        self.string_forces = {}
        self.string_constraints = {}

        self.dimensions = fill_dimensions(dimensions)
        self.joints = fill_joints(joints)

        # Create the skeleton frames
        self.import_frames(make_skeleton(self.dimensions, self.joints))

        # Add puppet string frames
        self.make_string_frames()    
        
        # Add desired forces/constraints
        if joint_forces:
            self.make_joint_forces()
        if string_forces:
            self.make_string_forces()
        if string_constraints:
            self.make_string_constraints()
                    
        # Add remaining forces/potentials
        trep.potentials.Gravity(self, (0, 0, -9.8))
        trep.forces.Damping(self, self.dimensions['damping'])

        
    def make_string_frames(self):
        # Add the string plane
        self.world_frame.import_frames([
            tz(self.dimensions['string_plane_height'], name='string_plane')])
        self.string_plane = self.get_frame('string_plane')

        # Add the string hook frames
        self.string_hooks = {}    
        for name, hook_location in self.dimensions['strings'].iteritems():
            self.string_hooks[name] = name + '_hook'
            self.get_frame(hook_location[0]).import_frames([
                const_txyz(hook_location[1], name=self.string_hooks[name])])

        
    def make_joint_forces(self):
        for config in self.dyn_configs:
            self.joint_forces[config.name] = \
                trep.forces.JointForce(self, config, config.name, config.name)
            

    def make_string_forces(self):
        for name, hook_point in self.string_hooks.iteritems():
            force = {
                'name' : name,
                'x' : name + '-x',
                'y' : name + '-y',
                'z' : name + '-z',
                'hook' : hook_point,
                }
            trep.forces.HybridWrench(self, force['hook'],
                                     (force['x'], force['y'], force['z'], 0, 0, 0),
                                     name=name)
            self.string_forces[name] = force


    def make_string_constraints(self):
        for name, hook_point in self.string_hooks.iteritems():
            info = {
                'name' : name,
                'x' : name + '-x',            # Name of X kinematic config variable
                'y' : name + '-y',            # Name of Y kinematic config variable
                'length' : name + '-length',  # Name of length kinematic config variable
                # Name of the frames connected by the strings
                'control_hook' : name + '_control',
                'hook' : hook_point
                }
            # Add frames from the control_hook
            self.string_plane.import_frames([
                tx(info['x'], kinematic=True), [
                    ty(info['y'], kinematic=True, name=info['control_hook'])
                    ]])
            trep.constraints.Distance(self, info['hook'],
                                      info['control_hook'], info['length'],
                                      name=name)
            self.string_constraints[name] = info


    def project_string_controls(self, strings=None):
        """
        Sets the location of each string control point to be directly
        above the corresponding puppet hook in the current
        configuration.  The string lengths are corrected as well.

        By default, it affects all the strings in the puppet.  Specify
        a list of string names to only project some strings.
        """
        if strings is None:
            infos = self.string_constraints.values()
        else:
            infos = [self.string_constraints[name] for name in strings]
        
        for info in infos:
            x_var = self.get_config(info['x'])
            y_var = self.get_config(info['y'])
            pos = self.get_frame(info['hook']).p()
            x_var.q = pos[0]
            y_var.q = pos[1]

        self.correct_string_lengths(strings)


    def correct_string_lengths(self, strings):
        """
        Sets the length of each string to the correct length.

        By default, it affects all the strings in the puppet.  Specify
        a list of string names to only project some strings.
        """
        if strings is None:
            infos = self.string_constraints.values()
        else:
            infos = [self.string_constraints[name] for name in strings]
        
        for info in infos:
            length_var = self.get_config(info['length'])
            length_var.q = self.get_constraint(info['name']).get_actual_distance()


class PuppetPainter(object):

    def __init__(self, puppet, color=(0.2, 0.2, 0.2, 1.0)):
        self.puppet = puppet
        self.color = color

        self.draw_funcs = [
            ('pelvis', self.draw_torso),
            ('left_humerus',  self.draw_left_humerus),
            ('left_radius',   self.draw_left_radius),
            ('left_hand',     self.draw_left_hand),
            ('right_humerus', self.draw_right_humerus),
            ('right_radius',  self.draw_right_radius),
            ('right_hand',    self.draw_right_hand),
            ('left_femur',    self.draw_left_femur),
            ('left_tibia',    self.draw_left_tibia),
            ('left_foot',     self.draw_left_foot),
            ('right_femur',   self.draw_right_femur),
            ('right_tibia',   self.draw_right_tibia),
            ('right_foot',    self.draw_right_foot),
            ('head_center',   self.draw_head),
            ('neck_joint', self.draw_neck_joint)            
            ]

        self.draw_funcs = [(self.puppet.get_frame(a), b) for (a,b) in self.draw_funcs]
        self.quad = gluNewQuadric()
        gluQuadricNormals(self.quad, GLU_SMOOTH)
        
    def draw(self):
        glPushAttrib(GL_CURRENT_BIT)            
        glColor4f(*self.color)

        glPushMatrix()
        #glScale(self.scale, self.scale, self.scale)
        
        for (frame, func) in self.draw_funcs:
            if frame:
                glPushMatrix()
                frame_g = frame.g()
                glMultMatrixf(gl_flatten_matrix(frame_g))
                func()
                glPopMatrix()
            else:
                func()

        glPopMatrix()
        glPopAttrib()

    def draw_neck_joint(self):
        radius = self.puppet.dimensions['neck_length']/2
        gluSphere(self.quad, radius, 10, 10)
        
    def draw_head(self):
        radius = self.puppet.dimensions['final_head_length']

        glPushMatrix()
        glScalef(0.5, 0.8, 1.0)
        gluSphere(self.quad, radius, 10, 10)
        glPopMatrix()
        
    def draw_torso(self):
        radius1 = (self.puppet.dimensions['lfemur_length'] +
                  self.puppet.dimensions['rfemur_length'])/16*0.9

        radius2 = (self.puppet.dimensions['lhumerus_length'] +
                  self.puppet.dimensions['rhumerus_length'])/16*0.9
        
        glPushMatrix()
        glRotatef(90, 0, 1, 0)
        glTranslatef(0, 0, -self.puppet.dimensions['lhip_width'])
        gluCylinder(self.quad, radius1, radius1,
                    self.puppet.dimensions['lhip_width'] +
                    self.puppet.dimensions['rhip_width'], 10, 1)
        glPopMatrix()

        gluCylinder(self.quad, radius1, radius2,
                    self.puppet.dimensions['upper_torso_length'], 10, 1)

        glTranslatef(0, 0, self.puppet.dimensions['upper_torso_length'])
            
        glPushMatrix()
        glRotatef(90, 0, 1, 0)
        glTranslatef(0, 0, -self.puppet.dimensions['lshoulder_width'])
        gluCylinder(self.quad, radius2, radius2,
                    self.puppet.dimensions['lshoulder_width'] +
                    self.puppet.dimensions['rshoulder_width'], 10, 1)
        glPopMatrix()

    def draw_left_humerus(self):
        radius = self.puppet.dimensions['lhumerus_length']/8
        
        gluSphere(self.quad, radius, 10, 10)
        glRotatef(180, 1, 0, 0)
        gluCylinder(self.quad, radius*0.9, radius*0.6,
                    self.puppet.dimensions['lhumerus_length'], 10, 1)
        
    def draw_left_radius(self):
        radius = self.puppet.dimensions['lhumerus_length']/8*0.6

        glPushMatrix()
        glRotatef(90, 0, 1, 0)
        glTranslatef(0, 0, -radius)
        gluCylinder(self.quad, radius, radius, 2*radius, 10, 1)
        gluDisk(self.quad, 0, radius, 10, 1)
        glTranslatef(0, 0, 2*radius)
        gluDisk(self.quad, 0, radius, 10, 1)
        glPopMatrix()
        glRotatef(180, 1, 0, 0)
        gluCylinder(self.quad, radius*0.9, radius*0.6,
                    self.puppet.dimensions['lradius_length'], 10, 1)
        
    def draw_left_hand(self):
        radius = self.puppet.dimensions['lhumerus_length']/8*0.6*0.6

        glPushMatrix()
        glRotatef(90, 0, 1, 0)
        glTranslatef(0, 0, -radius)
        gluCylinder(self.quad, radius, radius, 2*radius, 10, 1)
        gluDisk(self.quad, 0, radius, 10, 1)
        glTranslatef(0, 0, 2*radius)
        gluDisk(self.quad, 0, radius, 10, 1)
        glPopMatrix()
        glRotatef(180, 1, 0, 0)
        gluCylinder(self.quad, radius*0.9, radius*0.6,
                    self.puppet.dimensions['lhand_length'], 10, 1)
        glTranslatef(0,0,self.puppet.dimensions['lhand_length'])
        gluDisk(self.quad, 0, radius*0.6, 10, 1)
        
    def draw_right_humerus(self):
        radius = self.puppet.dimensions['rhumerus_length']/8
        
        gluSphere(self.quad, radius, 10, 10)
        glRotatef(180, 1, 0, 0)
        gluCylinder(self.quad, radius*0.9, radius*0.6,
                    self.puppet.dimensions['rhumerus_length'], 10, 1)
        
    def draw_right_radius(self):
        radius = self.puppet.dimensions['rhumerus_length']/8*0.6

        glPushMatrix()
        glRotatef(90, 0, 1, 0)
        glTranslatef(0, 0, -radius)
        gluCylinder(self.quad, radius, radius, 2*radius, 10, 1)
        gluDisk(self.quad, 0, radius, 10, 1)
        glTranslatef(0, 0, 2*radius)
        gluDisk(self.quad, 0, radius, 10, 1)
        glPopMatrix()
        glRotatef(180, 1, 0, 0)
        gluCylinder(self.quad, radius*0.9, radius*0.6,
                    self.puppet.dimensions['rradius_length'], 10, 1)
        
    def draw_right_hand(self):
        radius = self.puppet.dimensions['rhumerus_length']/8*0.6*0.6

        glPushMatrix()
        glRotatef(90, 0, 1, 0)
        glTranslatef(0, 0, -radius)
        gluCylinder(self.quad, radius, radius, 2*radius, 10, 1)
        gluDisk(self.quad, 0, radius, 10, 1)
        glTranslatef(0, 0, 2*radius)
        gluDisk(self.quad, 0, radius, 10, 1)
        glPopMatrix()
        glRotatef(180, 1, 0, 0)
        gluCylinder(self.quad, radius*0.9, radius*0.6,
                    self.puppet.dimensions['rhand_length'], 10, 1)
        glTranslatef(0,0,self.puppet.dimensions['rhand_length'])
        gluDisk(self.quad, 0, radius*0.6, 10, 1)

    def draw_left_femur(self):
        radius = self.puppet.dimensions['lfemur_length']/8
        
        gluSphere(self.quad, radius, 10, 10)
        glRotatef(180, 1, 0, 0)
        gluCylinder(self.quad, radius*0.9, radius*0.6,
                    self.puppet.dimensions['lfemur_length'], 10, 1)
        
    def draw_left_tibia(self):
        radius = self.puppet.dimensions['lfemur_length']/8*0.6

        glPushMatrix()
        glRotatef(90, 0, 1, 0)
        glTranslatef(0, 0, -radius)
        gluCylinder(self.quad, radius, radius, 2*radius, 10, 1)
        gluDisk(self.quad, 0, radius, 10, 1)
        glTranslatef(0, 0, 2*radius)
        gluDisk(self.quad, 0, radius, 10, 1)
        glPopMatrix()
        glRotatef(180, 1, 0, 0)
        gluCylinder(self.quad, radius*0.9, radius*0.6,
                    self.puppet.dimensions['ltibia_length'], 10, 1)
        
    def draw_left_foot(self):
        radius = self.puppet.dimensions['lfemur_length']/8*0.6*0.6

        glPushMatrix()
        glRotatef(90, 0, 1, 0)
        glTranslatef(0, 0, -radius)
        gluCylinder(self.quad, radius, radius, 2*radius, 10, 1)
        gluDisk(self.quad, 0, radius, 10, 1)
        glTranslatef(0, 0, 2*radius)
        gluDisk(self.quad, 0, radius, 10, 1)
        glPopMatrix()
        glRotatef(-90, 1, 0, 0)
        gluCylinder(self.quad, radius*0.9, radius*0.6,
                    self.puppet.dimensions['lfoot_length'], 10, 1)
        glTranslatef(0,0,self.puppet.dimensions['lfoot_length'])
        gluDisk(self.quad, 0, radius*0.6, 10, 1)
        
    def draw_right_femur(self):
        radius = self.puppet.dimensions['rfemur_length']/8
        
        gluSphere(self.quad, radius, 10, 10)
        glRotatef(180, 1, 0, 0)
        gluCylinder(self.quad, radius*0.9, radius*0.6,
                    self.puppet.dimensions['rfemur_length'], 10, 1)
        
    def draw_right_tibia(self):
        radius = self.puppet.dimensions['rfemur_length']/8*0.6

        glPushMatrix()
        glRotatef(90, 0, 1, 0)
        glTranslatef(0, 0, -radius)
        gluCylinder(self.quad, radius, radius, 2*radius, 10, 1)
        gluDisk(self.quad, 0, radius, 10, 1)
        glTranslatef(0, 0, 2*radius)
        gluDisk(self.quad, 0, radius, 10, 1)
        glPopMatrix()
        glRotatef(180, 1, 0, 0)
        gluCylinder(self.quad, radius*0.9, radius*0.6,
                    self.puppet.dimensions['rtibia_length'], 10, 1)
        
    def draw_right_foot(self):
        radius = self.puppet.dimensions['rfemur_length']/8*0.6*0.6

        glPushMatrix()
        glRotatef(90, 0, 1, 0)
        glTranslatef(0, 0, -radius)
        gluCylinder(self.quad, radius, radius, 2*radius, 10, 1)
        gluDisk(self.quad, 0, radius, 10, 1)
        glTranslatef(0, 0, 2*radius)
        gluDisk(self.quad, 0, radius, 10, 1)
        glPopMatrix()
        glRotatef(-90, 1, 0, 0)
        gluCylinder(self.quad, radius*0.9, radius*0.6,
                    self.puppet.dimensions['rfoot_length'], 10, 1)
        glTranslatef(0,0,self.puppet.dimensions['rfoot_length'])
        gluDisk(self.quad, 0, radius*0.6, 10, 1)
    

        


if __name__ == '__main__':
    puppet = Puppet(joint_forces=False, string_forces=False, string_constraints=True)
    puppet.get_config('torso_rx').q = 0.1
    puppet.get_config('torso_ry').q = 0.1
    puppet.project_string_controls()

    q0 = puppet.get_q()
    u0 = tuple([0.0]*len(puppet.inputs))
    qk2 = puppet.get_qk()
    dt = 0.01
    # Create and initialize a variational integrator for the system.
    gmvi = trep.GenMidpoint(puppet)
    gmvi.initialize_from_configs(0.0, q0, u0, dt, q0)


    q = [gmvi.q2]
    t = [gmvi.t2]
    while gmvi.t1 < 10.0:
        gmvi.step(gmvi.t2+dt, u0, qk2)
        q.append(gmvi.q2)
        t.append(gmvi.t2)
        # The puppet can take a while to simulate, so print out the time
        # occasionally to indicate our progress.
        if abs(gmvi.t2 - round(gmvi.t2)) < dt/2.0:
            print "t =",gmvi.t2


    

    import trep.visual

    painter = trep.visual.SystemTrajectoryViewer(puppet, t, q)
    painter.density = 500
    painter.run()


    
