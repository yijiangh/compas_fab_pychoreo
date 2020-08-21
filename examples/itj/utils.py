import numpy as np
from math import radians as rad
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh

from plyer import notification

def to_rlf_robot_full_conf(robot11_confval, robot12_confval, scale=1e-3):
    robot21_confval = [38000,
                       0, -4915,
                       0, 0, 0, 0, 0, 0]
    robot22_confval = [-12237, -4915,
                       0, 0, 0, 0, 0, 0]
    return Configuration(
        values = (
            # robot11[0],
            # robot11[1], robot11[2],
            # rad(robot11[3]), rad(robot11[4]), rad(robot11[5]), rad(robot11[6]), rad(robot11[7]), rad(robot11[8]),
            *[robot11_confval[i]*scale for i in range(0, 3)],
            *[rad(robot11_confval[i]) for i in range(3, 9)],
            # robot12[0], robot12[1],
            # rad(robot12[2]), rad(robot12[3]), rad(robot12[4]), rad(robot12[5]), rad(robot12[6]), rad(robot12[7]),
            *[robot12_confval[i]*scale for i in range(0, 2)],
            *[rad(robot12_confval[i]) for i in range(2, 8)],
            # below fixed
            *[robot21_confval[i]*scale for i in range(0, 3)],
            *[rad(robot21_confval[i]) for i in range(3, 9)],
            *[robot22_confval[i]*scale for i in range(0, 2)],
            *[rad(robot22_confval[i]) for i in range(2, 8)],
            ),
        types = (
            2,
            2, 2,
            0, 0, 0, 0, 0, 0,
            2, 2,
            0, 0, 0, 0, 0, 0,
            2,
            2, 2,
            0, 0, 0, 0, 0, 0,
            2, 2,
            0, 0, 0, 0, 0, 0
            ),
        joint_names = (
            'bridge1_joint_EA_X',
            'robot11_joint_EA_Y', 'robot11_joint_EA_Z',
            'robot11_joint_1', 'robot11_joint_2', 'robot11_joint_3', 'robot11_joint_4', 'robot11_joint_5', 'robot11_joint_6',
            'robot12_joint_EA_Y', 'robot12_joint_EA_Z',
            'robot12_joint_1', 'robot12_joint_2', 'robot12_joint_3', 'robot12_joint_4', 'robot12_joint_5', 'robot12_joint_6',
            'bridge2_joint_EA_X',
            'robot21_joint_EA_Y', 'robot21_joint_EA_Z',
            'robot21_joint_1', 'robot21_joint_2', 'robot21_joint_3', 'robot21_joint_4', 'robot21_joint_5', 'robot21_joint_6',
            'robot22_joint_EA_Y', 'robot22_joint_EA_Z',
            'robot22_joint_1', 'robot22_joint_2', 'robot22_joint_3', 'robot22_joint_4', 'robot22_joint_5', 'robot22_joint_6'
            )
        )

def rfl_camera(scale=1e-3):
    camera = {
        'location': np.array([14830.746366, 17616.580504, 9461.594828])*scale,
        'target' : np.array([24470.185559, 7976.896428, 2694.413294])*scale,
        'lens' : 50.0*scale,
        'up_direction':np.array([0.314401,-0.314409,0.895712])*scale,
    }
    return camera

##########################################

def notify(msg=''):
    notification.notify(
        title='pybullet planning',
        message=msg,
        app_icon=None,  # e.g. 'C:\\icon_32x32.ico'
        timeout=1e3,  # seconds
    )
