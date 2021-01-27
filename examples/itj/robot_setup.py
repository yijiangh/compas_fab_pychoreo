import numpy as np
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh
from termcolor import cprint

from .utils import convert_rfl_robot_conf_unit

# 2 for prismatic, 0 for revoluted
RFL_SINGLE_ARM_JOINT_TYPES = [2, 2, 0, 0, 0, 0, 0, 0,]

############################################

R11_START_CONF_VALS = convert_rfl_robot_conf_unit([22700.0, 0.0, -4900.0,
    0.0, -80.0, 65.0, 65.0, 20.0, -20.0])
R12_START_CONF_VALS = convert_rfl_robot_conf_unit([-4056.0883789999998, -4000.8486330000001,
    0.0, -22.834741999999999, -30.711554, 0.0, 57.335655000000003, 0.0])

# ! we only use robot11 and robot12
R21_IDLE_CONF_VALS = convert_rfl_robot_conf_unit([38000, 0, -4915,
    0, 0, 0, 0, 0, 0])
R22_IDLE_CONF_VALS = convert_rfl_robot_conf_unit([-12237, -4915,
    0, 0, 0, 0, 0, 0])

# meter
GANTRY_X_LIMIT = (25, 30) # (20.5, 25.0)
GANTRY_Y_LIMIT = (-12.237, -5) # (-12.237, -9.5)
GANTRY_Z_LIMIT = (-4.915, -3.5)

R11_GANTRY_CUSTOM_LIMITS = {
    'robot11_joint_EA_X' : GANTRY_X_LIMIT,
    'robot11_joint_EA_Y' : GANTRY_Y_LIMIT,
    'robot11_joint_EA_Z' : GANTRY_Z_LIMIT,
}

############################################

def to_rlf_robot_full_conf(robot11_confval, robot12_confval):
    return Configuration(
        values = (*robot11_confval, *robot12_confval,
                  *R21_IDLE_CONF_VALS, *R22_IDLE_CONF_VALS),
        types = (
            *([2] + RFL_SINGLE_ARM_JOINT_TYPES), *RFL_SINGLE_ARM_JOINT_TYPES,
            *([2] + RFL_SINGLE_ARM_JOINT_TYPES), *RFL_SINGLE_ARM_JOINT_TYPES,),
        joint_names = (
            'bridge1_joint_EA_X', 'robot11_joint_EA_Y', 'robot11_joint_EA_Z',
            'robot11_joint_1', 'robot11_joint_2', 'robot11_joint_3', 'robot11_joint_4', 'robot11_joint_5', 'robot11_joint_6',
            'robot12_joint_EA_Y', 'robot12_joint_EA_Z',
            'robot12_joint_1', 'robot12_joint_2', 'robot12_joint_3', 'robot12_joint_4', 'robot12_joint_5', 'robot12_joint_6',
            'bridge2_joint_EA_X', 'robot21_joint_EA_Y', 'robot21_joint_EA_Z',
            'robot21_joint_1', 'robot21_joint_2', 'robot21_joint_3', 'robot21_joint_4', 'robot21_joint_5', 'robot21_joint_6',
            'robot22_joint_EA_Y', 'robot22_joint_EA_Z',
            'robot22_joint_1', 'robot22_joint_2', 'robot22_joint_3', 'robot22_joint_4', 'robot22_joint_5', 'robot22_joint_6'
            )
        )
