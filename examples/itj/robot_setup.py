import numpy as np
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh
from termcolor import cprint

from compas_fab_pychoreo.client import PyChoreoClient
from pybullet_planning import draw_pose, set_camera_pose, GREY, unit_pose

from .utils import convert_rfl_robot_conf_unit
from .visualization import rfl_camera
from .parsing import rfl_setup, itj_rfl_obstacle_cms

############################################

# Beam-pick up configuration
R11_INTER_CONF_VALS = convert_rfl_robot_conf_unit([21000.0, 0.0, -4900.0,
    0.0, -80.0, 65.0, 65.0, 20.0, -20.0])
R12_INTER_CONF_VALS = convert_rfl_robot_conf_unit([-4056.0883789999998, -4000.8486330000001,
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

############################################

# 2 for prismatic, 0 for revoluted
RFL_SINGLE_ARM_JOINT_TYPES = [2, 2, 0, 0, 0, 0, 0, 0,]

try:
    import ikfast_abb_irb4600_40_255
    IK_MODULE = ikfast_abb_irb4600_40_255
except ImportError as e:
    IK_MODULE = None
    cprint('{}, Using pybullet ik fn instead'.format(e), 'red')

############################################

def rfl_robot_joint_names(robot_id='robot12', include_gantry=False):
    template = ['robot_joint_EA_Y', 'robot_joint_EA_Z', 'robot_joint_1', 'robot_joint_2',
        'robot_joint_3', 'robot_joint_4', 'robot_joint_5', 'robot_joint_6']
    joint_names = [robot_id + jn.split('robot')[1] for jn in template]
    if include_gantry:
        bridge_id = 1 if '11' in robot_id or '12' in robot_id else 2
        bridge_joint_name = 'bridge{}_joint_EA_X'.format(bridge_id)
        return [bridge_joint_name] + joint_names
    else:
        return joint_names

def to_rlf_robot_full_conf(robot11_confval, robot12_confval):
    return Configuration(
        values = (*robot11_confval, *robot12_confval,
                  *R21_IDLE_CONF_VALS, *R22_IDLE_CONF_VALS),
        types = (
            *([2] + RFL_SINGLE_ARM_JOINT_TYPES), *RFL_SINGLE_ARM_JOINT_TYPES,
            *([2] + RFL_SINGLE_ARM_JOINT_TYPES), *RFL_SINGLE_ARM_JOINT_TYPES,),
        joint_names = (
            *rfl_robot_joint_names('robot11', include_gantry=True),
            *rfl_robot_joint_names('robot12', include_gantry=False),
            *rfl_robot_joint_names('robot21', include_gantry=True),
            *rfl_robot_joint_names('robot22', include_gantry=False),
            # 'bridge1_joint_EA_X', 'robot11_joint_EA_Y', 'robot11_joint_EA_Z',
            # 'robot11_joint_1', 'robot11_joint_2', 'robot11_joint_3', 'robot11_joint_4', 'robot11_joint_5', 'robot11_joint_6',
            # 'robot12_joint_EA_Y', 'robot12_joint_EA_Z',
            # 'robot12_joint_1', 'robot12_joint_2', 'robot12_joint_3', 'robot12_joint_4', 'robot12_joint_5', 'robot12_joint_6',
            # 'bridge2_joint_EA_X', 'robot21_joint_EA_Y', 'robot21_joint_EA_Z',
            # 'robot21_joint_1', 'robot21_joint_2', 'robot21_joint_3', 'robot21_joint_4', 'robot21_joint_5', 'robot21_joint_6',
            # 'robot22_joint_EA_Y', 'robot22_joint_EA_Z',
            # 'robot22_joint_1', 'robot22_joint_2', 'robot22_joint_3', 'robot22_joint_4', 'robot22_joint_5', 'robot22_joint_6'
            )
        )

############################################

CARTESIAN_CONTROL_JOINTS = rfl_robot_joint_names('robot12', False)[2:]

########################################

def load_RFL_world(viewer=True, disable_env=False):
    urdf_filename, semantics = rfl_setup()
    client = PyChoreoClient(viewer=viewer)
    client.connect()

    robot = client.load_robot(urdf_filename)
    robot.semantics = semantics
    # robot's unique body index in pybullet
    robot_uid = client.get_robot_pybullet_uid(robot)

    # * draw base frame and locate camera in pybullet
    draw_pose(unit_pose(), length=1.)
    cam = rfl_camera()
    set_camera_pose(cam['target'], cam['location'])

    # * pipe attachments
    # if not disable_attachment:
    #     pipe_cms = itj_rfl_pipe_cms()
    #     attached_cm_pipe2 = AttachedCollisionMesh(pipe_cms[0], 'robot12_link_2', ['robot12_link_2'])
    #     attached_cm_pipe3 = AttachedCollisionMesh(pipe_cms[1], 'robot12_link_3', ['robot12_link_3'])
    #     client.add_attached_collision_mesh(attached_cm_pipe2, options={'robot': robot, 'mass': 1, 'color': BLUE})
    #     client.add_attached_collision_mesh(attached_cm_pipe3, options={'robot': robot, 'mass': 1, 'color': BLUE})

    # # * Add static collision mesh to planning scene
    if not disable_env:
        for o_cm in itj_rfl_obstacle_cms():
            client.add_collision_mesh(o_cm, {'color':GREY})

    # * collision sanity check
    full_start_conf = to_rlf_robot_full_conf(R11_INTER_CONF_VALS, R12_INTER_CONF_VALS)
    client.set_robot_configuration(robot, full_start_conf)
    assert not client.check_collisions(robot, full_start_conf, options={'diagnosis':True})

    return client, robot, robot_uid