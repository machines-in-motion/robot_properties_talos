"""config

Store the configuration of the Talos family robots.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import numpy as np
import pinocchio as se3
from pinocchio.utils import zero
import example_robot_data 

class TalosAbstract(object):
    """ Abstract class for KUKA robots """

    @classmethod
    def buildRobotWrapper(cls):
        # Rebuild the robot wrapper instead of using the existing model to
        # also load the visuals.
        robot = example_robot_data.load(cls.robot_name) 
        return robot

    def joint_name_in_single_string(self):
        joint_names = ""
        for name in self.robot_model.names[2:]:
            joint_names += name + " "
        return joint_names




class TalosArmConfig(TalosAbstract):
    '''
    Config class for the Talos left arm 
    '''
    loader = example_robot_data.robots_loader.TalosArmLoader()
    urdf_path   = loader.urdf_path
    meshes_path = loader.model_path
    robot_name  = 'talos_arm'
    # Pinocchio model.
    robot_model = se3.buildModelFromUrdf(urdf_path)
    # robot_model.rotorInertia[:] = motor_inertia
    # robot_model.rotorGearRatio[:] = motor_gear_ration
    mass = np.sum([i.mass for i in robot_model.inertias])
    base_name = robot_model.frames[2].name
    # The number of motors, here they are the same as there are only revolute joints.
    nb_joints = robot_model.nv
    joint_names = ['arm_left_2_joint',
                   'arm_left_3_joint',
                   'arm_left_4_joint',
                   'arm_left_5_joint',
                   'arm_left_6_joint',
                   'arm_left_7_joint',
                   'gripper_left_joint']
    end_effector_names = ["gripper_left_motor_single_link"]
    map_joint_name_to_id = {}
    map_joint_limits = {}
    for i, (name, lb, ub) in enumerate(
        zip(
            robot_model.names[1:],
            robot_model.lowerPositionLimit,
            robot_model.upperPositionLimit,
        )
    ):
        map_joint_name_to_id[name] = i
        map_joint_limits[i] = [float(lb), float(ub)]
    # Define the initial state.
    initial_configuration = [0.]*robot_model.nq
    initial_velocity = [0.]*robot_model.nv
    q0 = zero(robot_model.nq)
    q0[:] = initial_configuration
    v0 = zero(robot_model.nv)
    a0 = zero(robot_model.nv)



class TalosFullConfig(TalosAbstract):
    '''
    Config class for the Talos full model 
    '''
    loader = example_robot_data.robots_loader.TalosLoader()
    urdf_path   = loader.urdf_path
    meshes_path = loader.model_path
    robot_name  = 'talos'
    # Pinocchio model.
    robot_model = se3.buildModelFromUrdf(urdf_path)
    mass = np.sum([i.mass for i in robot_model.inertias])
    base_name = robot_model.frames[2].name
    # The number of motors, here they are the same as there are only revolute joints.
    nb_joints = robot_model.nv
    joint_names = [ "leg_left_1_joint",
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
    end_effector_names = ["arm_right_7_link",
                          "arm_left_7_link", 
                          "leg_right_sole_fix_joint", 
                          "leg_left_sole_fix_joint"]
    map_joint_name_to_id = {}
    map_joint_limits = {}
    for i, (name, lb, ub) in enumerate(
        zip(
            robot_model.names[1:],
            robot_model.lowerPositionLimit,
            robot_model.upperPositionLimit,
        )
    ):
        map_joint_name_to_id[name] = i
        map_joint_limits[i] = [float(lb), float(ub)]
    # Define the initial state.
    initial_configuration = [0.]*robot_model.nq
    initial_velocity = [0.]*robot_model.nv
    q0 = zero(robot_model.nq)
    q0[:] = initial_configuration
    v0 = zero(robot_model.nv)
    a0 = zero(robot_model.nv)
