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
import pinocchio as pin
dt = 1e-3
import numpy as np


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

        # Load full robot in PyBullet from urdf + mesh
        pybullet.setAdditionalSearchPath(TalosFullConfig.meshes_path)
        self.urdf_path = TalosFullConfig.urdf_path
        self.robotId = pybullet.loadURDF(
                self.urdf_path,
                pos, orn,
                flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
                useFixedBase=False)
        pybullet.getBasePositionAndOrientation(self.robotId)
        
        # Create the robot wrapper in pinocchio (reduced model)
        robot_full = TalosFullConfig.buildRobotWrapper()
        # 34 joints = universe + root + grippers + legs,arms,torso,head 
        #              (1)        (1)    (2)         (30)
        # print("fullmodel.names = ", str(len([name for name in robot_full.model.names])))
        # print(robot_full.model)
        # Query all the joints.
        num_joints = pybullet.getNumJoints(self.robotId)

        for ji in range(num_joints):
            pybullet.changeDynamics(self.robotId, 
                                    ji, 
                                    linearDamping=.04,
                                    angularDamping=0.04, 
                                    restitution=0.0, 
                                    lateralFriction=0.5)
        # controlled joints (names and pinocchio ids) #6
        controlled_joints = ['torso_1_joint',   
                             'torso_2_joint', 
                             'arm_right_1_joint', 
                             'arm_right_2_joint', 
                             'arm_right_3_joint', 
                             'arm_right_4_joint']
        controlled_joints_ids = []
        for joint_name in controlled_joints:
            controlled_joints_ids.append(robot_full.model.getJointId(joint_name))
        # print('Controlled joint names '+'('+str(len(controlled_joints))+') : ')
        # print(controlled_joints)
        # print('Controlled joint pinocchio ids '+'('+str(len(controlled_joints_ids))+') : ')
        # print(controlled_joints_ids)

        # Joint names to lock
        uncontrolled_joints = [] # 27 = 34 - 6 (controlled) - 1(universe)
        for joint_name in robot_full.model.names[1:]:
            if(joint_name not in controlled_joints):
                uncontrolled_joints.append(joint_name)
        # print('uncontrolled joint names '+'('+str(len(uncontrolled_joints))+') : ')
        # print(uncontrolled_joints)
        # Pinocchio joint ids to lock
        locked_joints_ids = [robot_full.model.getJointId(joint_name) for joint_name in uncontrolled_joints]
        # locked_joints_ids.pop(0) # exclude root joint : 27 ids (removed root because reduced model is fixed-base)
        # print('Locked joints ids in pinocchio '+'('+str(len(locked_joints_ids))+') : ')
        # print(locked_joints_ids)
        # Build reduced model with ref posture for locked joints
        qref = robot_full.model.referenceConfigurations['half_sitting'] # 39 : 7 dof difference = quaternion (base orientation in WORLD)
        # print('ref config '+'('+str(len(qref))+') : ')
        # print(qref)
        # 7 quat (root_joint) # leg left_1-6 # leg_right_1-6 # torso_1-2 # arm_left_1-7 # arm_right_1-7 # head_1-2 # ??? 
        
        qref_locked_map = {}
        if('root_joint' in uncontrolled_joints):
            # print("map root joint "+str(qref[0:7]))
            qref_locked_map['root_joint'] = qref[0:7]
        for joint_name in uncontrolled_joints:
            if(joint_name != 'root_joint'):
                idx = 6+robot_full.model.getJointId(joint_name)-1
                # print("map "+str(joint_name) + " (id = "+str(idx) +") : " + str(qref[idx]))
                qref_locked_map[joint_name] = qref[idx]

        reduced_model, [visual_model, collision_model] = pin.buildReducedModel(robot_full.model, 
                                                                               [robot_full.visual_model, robot_full.collision_model], 
                                                                               locked_joints_ids, 
                                                                               qref)   
        self.pin_robot = pin.robot_wrapper.RobotWrapper(reduced_model, collision_model, visual_model)  

        print("[pinbullet wrapper] REDUCED MODEL : ", self.pin_robot.model)
        # base and EE
        self.base_link_name = "base_link"
        self.end_eff_ids = []
        self.end_eff_ids.append(self.pin_robot.model.getFrameId('arm_right_7_link'))
        self.joint_names = controlled_joints

        # Get bullet map joint_name<->bullet_index
        bullet_joint_map = {}
        for ji in range(pybullet.getNumJoints(self.robotId)):
            # print("bullet map joint name = ", pybullet.getJointInfo(self.robotId, ji)[1].decode("UTF-8"))
            bullet_joint_map[pybullet.getJointInfo(self.robotId, ji)[1].decode("UTF-8")] = ji
        # Get bullet ids of locked joints + subconfig
        uncontrolled_joints.remove('root_joint') # base treated in sim
        locked_joint_ids_bullet = np.array([bullet_joint_map[name] for name in uncontrolled_joints])
        qref_locked = [qref_locked_map[joint_name] for joint_name in uncontrolled_joints]
        # print('bullet locked joint ids '+'('+str(len(locked_joint_ids_bullet))+') : ')
        # print(locked_joint_ids_bullet)
        # print('ref config LOCKED '+'('+str(len(qref_locked))+') : ')
        # print(qref_locked)
        # Lock the uncontrolled joints in position control in PyBullet multibody (full robot)
        for joint_name in uncontrolled_joints:
            print("joint name : " + joint_name + " , bullet joint id = ", bullet_joint_map[joint_name])
            pybullet.resetJointState(self.robotId, bullet_joint_map[joint_name], qref_locked_map[joint_name], 0.)
        pybullet.setJointMotorControlArray(self.robotId, 
                                           jointIndices = locked_joint_ids_bullet, 
                                           controlMode = pybullet.POSITION_CONTROL,
                                           targetPositions = qref_locked,
                                           targetVelocities = np.zeros(len(locked_joint_ids_bullet)))


        # Creates the wrapper by calling the super.__init__.
        # wrapper created from REDUCED model i.e. will only map controlled pin joints to bullet joints 
        # and consider fixed base 
        super(TalosReducedRobot, self).__init__(
                    self.robotId, 
                    self.pin_robot,
                    controlled_joints,
                    ['arm_right_7_joint'],
                    useFixedBase=True) # no floating base for reduced model
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

