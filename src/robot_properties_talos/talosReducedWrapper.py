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


class TalosReducedRobot(PinBulletWrapper):
    '''
    Pinocchio-PyBullet wrapper class for the Talos torso + right arm
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

        controlled_joints = ['torso_1_joint',   
                             'torso_2_joint', 
                             'arm_right_1_joint', 
                             'arm_right_2_joint', 
                             'arm_right_3_joint', 
                             'arm_right_4_joint']
        controlled_joints_ids = []
        for joint_name in controlled_joints:
            controlled_joints_ids.append(self.pin_robot .model.getJointId(joint_name))
        locked_joints_ids = []
        for joint_name in self.pin_robot .model.names:
            if(joint_name not in controlled_joints):
                locked_joints_ids.append(self.pin_robot .model.getJointId(joint_name))
        locked_joints_ids.pop(0) # excl. root joint
        import pinocchio as pin
        qref = self.pin_robot.model.referenceConfigurations['half_sitting']
        reduced_model = pin.buildReducedModel(self.pin_robot .model, locked_joints_ids, qref)      
        self.pin_robot = pin.robot_wrapper.RobotWrapper(reduced_model)  

        uncontrolled_joints = []
        for joint_name in controlled_joints:
            if(joint_name not in self.pin_robot.model.names):
                uncontrolled_joints.append(joint_name)


        self.base_link_name = "base_link"
        self.end_eff_ids = []

        self.end_eff_ids.append(self.pin_robot.model.getFrameId('arm_right_7_link'))

        self.joint_names = controlled_joints
        
        # Creates the wrapper by calling the super.__init__.
        super(TalosReducedRobot, self).__init__(
                    self.robotId, 
                    self.pin_robot,
                    controlled_joints,
                    ['arm_right_7_joint'],
                    useFixedBase=True)
        self.nb_dof = self.nv



        pybullet.setJointMotorControlArray(self.robotId, 
                                    jointIndices= locked_joints_ids, 
                                    controlMode= pybullet.POSITION_CONTROL,
                                    forces = [0.0 for m in locked_joints_ids]) 



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

