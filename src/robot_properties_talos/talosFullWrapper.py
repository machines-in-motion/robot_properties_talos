"""talosFullWrapper

Talos full pybullet interface using pinocchio's convention.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft, LAAS-CNRS
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""


import pybullet 
from py_pinocchio_bullet.wrapper import PinBulletWrapper
from robot_properties_talos.config import TalosFullConfig
dt = 1e-3


class TalosFullRobot(PinBulletWrapper):
    '''
    Pinocchio-PyBullet wrapper class for the Talos full robot
    '''
    def __init__(self, pos=None, orn=None): 

        # Load the robot
        if pos is None:
            pos = [0.0, 0, 0.0]
        if orn is None:
            orn = pybullet.getQuaternionFromEuler([0, 0, 0])

        pybullet.setAdditionalSearchPath(TalosFullConfig.meshes_path)
        self.urdf_path = TalosFullConfig.urdf_path
        self.robotId = pybullet.loadURDF(
                self.urdf_path,
                pos, orn,
                flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
                useFixedBase=True)
        pybullet.getBasePositionAndOrientation(self.robotId)
        
        # Create the robot wrapper in pinocchio.
        self.pin_robot = TalosFullConfig.buildRobotWrapper()

        # Query all the joints.
        num_joints = pybullet.getNumJoints(self.robotId)

        for ji in range(num_joints):
            pybullet.changeDynamics(self.robotId, 
                                    ji, 
                                    linearDamping=.04,
                                    angularDamping=0.04, 
                                    restitution=0.0, 
                                    lateralFriction=0.5)

        self.base_link_name = "base_link"
        self.end_eff_ids = []
        controlled_joints = [ "leg_left_1_joint",
                              "leg_left_2_joint",
                              "leg_left_3_joint",
                              "leg_left_4_joint",
                              "leg_left_5_joint",
                              "leg_left_6_joint",
                              "leg_right_1_joint",
                              "leg_right_2_joint",
                              "leg_right_3_joint",
                              "leg_right_4_joint",
                              "leg_right_5_joint",
                              "leg_right_6_joint",
                              "torso_1_joint ",
                              "torso_2_joint ",
                              "arm_left_1_joint ",
                              "arm_left_2_joint ",
                              "arm_left_3_joint ",
                              "arm_left_4_joint ",
                              "arm_left_5_joint ",
                              "arm_left_6_joint ",
                              "arm_left_7_joint ",
                              "gripper_left_joint",
                              "arm_right_1_joint ",
                              "arm_right_2_joint ",
                              "arm_right_3_joint ",
                              "arm_right_4_joint ",
                              "arm_right_5_joint ",
                              "arm_right_6_joint ",
                              "arm_right_7_joint ",
                              "gripper_right_joint ",
                              "head_1_joint"
                              "head_2_joint"]

        self.end_eff_ids.append(self.pin_robot.model.getFrameId('arm_right_7_link'))
        self.end_eff_ids.append(self.pin_robot.model.getFrameId('arm_left_7_link'))
        self.end_eff_ids.append(self.pin_robot.model.getFrameId('leg_right_sole_fix_joint'))
        self.end_eff_ids.append(self.pin_robot.model.getFrameId('leg_left_sole_fix_joint'))

        self.joint_names = controlled_joints
        
        # Creates the wrapper by calling the super.__init__.
        super(TalosFullRobot, self).__init__(
                    self.robotId, 
                    self.pin_robot,
                    controlled_joints,
                    ['arm_right_7_joint', 'arm_left_7_joint', 'leg_right_sole_fix_joint', 'leg_left_sole_fix_joint'],
                    useFixedBase=True)
        self.nb_dof = self.nv
        
    def forward_robot(self, q=None, dq=None):
        if q is None:
            q, dq = self.get_state()
        elif dq is None:
            raise ValueError("Need to provide q and dq or non of them.")

        self.pin_robot.forwardKinematics(q, dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)

    def start_recording(self, file_name):
        self.file_name = file_name
        pybullet.startStateLogging(pybullet.STATE_LOGGING_VIDEO_MP4, self.file_name)

    def stop_recording(self):
        pybullet.stopStateLogging(pybullet.STATE_LOGGING_VIDEO_MP4, self.file_name)

