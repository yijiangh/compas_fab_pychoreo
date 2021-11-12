import os
import argparse
import numpy as np
from math import radians as rad
from numpy.testing import assert_almost_equal
from termcolor import cprint
from itertools import product

from compas.datastructures import Mesh
from compas.robots import RobotModel
from compas_fab.robots import RobotSemantics
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh

from pybullet_planning import link_from_name, get_link_pose, draw_pose, get_bodies, multiply, Pose, Euler, set_joint_positions, \
    joints_from_names, quat_angle_between, get_collision_fn, create_obj, unit_pose, set_camera_pose
from pybullet_planning import wait_if_gui, wait_for_duration
from pybullet_planning import plan_cartesian_motion, plan_cartesian_motion_lg
from pybullet_planning import randomize, elapsed_time, GREY, LockRenderer

# from compas_fab.backends import PyBulletClient
from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose

def to_rlf_robot_full_conf(robot11_confval, robot12_confval, scale=1e-3):
    # convert to full configuration of the RFL robot
    robot21_confval = [38000,
                       0, -4915,
                       0, 0, 0, 0, 0, 0]
    robot22_confval = [-12237, -4915,
                       0, 0, 0, 0, 0, 0]
    return Configuration(
        values = (
            *[robot11_confval[i]*scale for i in range(0, 3)],
            *[rad(robot11_confval[i]) for i in range(3, 9)],
            *[robot12_confval[i]*scale for i in range(0, 2)],
            *[rad(robot12_confval[i]) for i in range(2, 8)],
            # below fixed
            *[robot21_confval[i]*1e-3 for i in range(0, 3)],
            *[rad(robot21_confval[i]) for i in range(3, 9)],
            *[robot22_confval[i]*1e-3 for i in range(0, 2)],
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

def debug_rfl_camera(scale=1e-3):
    camera = {
        'location': np.array([14830.746366, 17616.580504, 9461.594828])*scale,
        'target' : np.array([24470.185559, 7976.896428, 2694.413294])*scale,
        'lens' : 50.0*scale,
        'up_direction':np.array([0.314401,-0.314409,0.895712])*scale,
    }
    return camera

#####################################
def rfl_setup():
    HERE = os.path.dirname(__file__)
    data_dir = os.path.abspath(os.path.join(HERE, "..", "..", "data", 'robots'))
    urdf_filename = os.path.join(data_dir, 'rfl_description', 'rfl_description', "urdf", "rfl_pybullet.urdf")
    srdf_filename = os.path.join(data_dir, 'rfl_description', 'rfl_description', "urdf", "rfl.srdf")
    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    return urdf_filename, semantics

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning, default False')

    args = parser.parse_args()

    viewer = args.viewer
    diagnosis=True

    urdf_filename, semantics = rfl_setup()
    move_group = 'robot11_eaXYZ' # 'robot12_eaYZ'

    with PyChoreoClient(viewer=viewer) as client:
        cprint(urdf_filename, 'green')
        with LockRenderer():
            robot = client.load_robot(urdf_filename)
        robot.semantics = semantics
        robot_uid = client.get_robot_pybullet_uid(robot)

        ik_joint_names = robot.get_configurable_joint_names(group=move_group)
        ik_joint_types = robot.get_joint_types_by_names(ik_joint_names)
        flange_link_name = robot.get_end_effector_link_name(group=move_group)

        ee_touched_link_names = ['robot12_tool0', 'robot12_link_6']
        # ee_acms = [AttachedCollisionMesh(ee_cm, flange_link_name, ee_touched_link_names) for ee_cm in itj_TC_PG500_cms]
        # beam_acm = AttachedCollisionMesh(itj_beam_cm, flange_link_name, ee_touched_link_names)

        draw_pose(unit_pose(), length=1.)

        cam = debug_rfl_camera()
        set_camera_pose(cam['target'], cam['location'])

        ik_joints = joints_from_names(robot_uid, ik_joint_names)
        flange_link = link_from_name(robot_uid, flange_link_name)
        # robot_base_link = link_from_name(robot_uid, base_link_name)

        r11_start_conf_vals = np.array([22700.0, 0.0, -4900.0, \
            0.0, -80.0, 65.0, 65.0, 20.0, -20.0])
        r12_start_conf_vals = np.array([-4056.0883789999998, -4000.8486330000001, \
            0.0, -22.834741999999999, -30.711554, 0.0, 57.335655000000003, 0.0])
        full_start_conf = to_rlf_robot_full_conf(r11_start_conf_vals, r12_start_conf_vals)
        # full_joints = joints_from_names(robot_uid, full_start_conf.joint_names)

        client.set_robot_configuration(robot, full_start_conf)

        # # * add attachment
        # for ee_acm in ee_acms:
        #     client.add_attached_collision_mesh(ee_acm, options={'robot' : robot, 'mass': 1})
        # client.add_attached_collision_mesh(beam_acm, options={'robot' : robot, 'mass': 1})

        # safe start conf
        start_confval = np.hstack([r11_start_conf_vals[:3]*1e-3, np.radians(r11_start_conf_vals[3:])])
        conf = Configuration(values=start_confval.tolist(), types=ik_joint_types, joint_names=ik_joint_names)
        assert not client.check_collisions(robot, conf, options={'diagnosis':diagnosis})

        # TODO add more tests
        from pybullet_planning import get_link_subtree, prune_fixed_joints, clone_body, get_movable_joints, get_all_links, \
            set_color, child_link_from_joint, get_link_name
        first_joint = ik_joints[0]
        target_link = flange_link

        # front_selected_links = [child_link_from_joint(joint) for joint in ik_joints]
        # rear_selected_links = get_link_subtree(robot_uid, ik_joints[2])
        # selected_links = list(set(front_selected_links) | set(rear_selected_links))
        selected_links = [link_from_name(robot_uid, l) for l in robot.get_link_names(group=move_group)]
        selected_movable_joints = prune_fixed_joints(robot_uid, selected_links)

        print('selected_links: ', selected_links)
        print('selected_links names: ', [get_link_name(robot_uid, l) for l in selected_links])
        print('all_links names: ', [get_link_name(robot_uid, l) for l in get_all_links(robot_uid)])
        print('get_link_names: ', robot.get_link_names(group=move_group))

        assert(target_link in selected_links)
        selected_target_link = selected_links.index(target_link)
        sub_robot = clone_body(robot_uid, links=selected_links, visual=True, collision=True) # TODO: joint limits
        sub_movable_joints = get_movable_joints(sub_robot)

        # conf = Configuration(values=[0.]*6, types=ik_joint_types, joint_names=ik_joint_names)
        # assert not client.configuration_in_collision(conf, group=move_group, options={'diagnosis':True})
        wait_if_gui()

if __name__ == '__main__':
    main()
