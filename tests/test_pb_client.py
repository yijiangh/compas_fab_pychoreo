import pytest
import os
import numpy as np
from numpy.testing import assert_almost_equal
import time
import random
from termcolor import cprint
from itertools import product

import cProfile
import pstats

from compas.datastructures import Mesh
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh

from pybullet_planning import link_from_name, get_link_pose, draw_pose, get_bodies, multiply, Pose, Euler, set_joint_positions, \
    joints_from_names, quat_angle_between, get_collision_fn, create_obj, unit_pose
from pybullet_planning import wait_if_gui, wait_for_duration
from pybullet_planning import plan_cartesian_motion, plan_cartesian_motion_lg
from pybullet_planning import randomize, elapsed_time

from compas_fab_pychoreo.client import PyBulletClient
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo_examples.ik_solver import ik_abb_irb4600_40_255, InverseKinematicsSolver, get_ik_fn_from_ikfast

def compute_circle_path(circle_center=np.array([2, 0, 0.2]), circle_r=0.2, angle_range=(-0.5*np.pi, 0.5*np.pi)):
    ee_poses = []
    n_pt = int(abs(angle_range[1]-angle_range[0]) / (np.pi/180 * 5))
    for a in np.linspace(*angle_range, num=n_pt):
        pt = circle_center + circle_r*np.array([np.cos(a), np.sin(a), 0])
        circ_pose = multiply(Pose(point=pt, euler=Euler(yaw=a+np.pi/2)), Pose(euler=Euler(roll=np.pi*3/4)))
        draw_pose(circ_pose, length=0.01)
        ee_poses.append(circ_pose)
    return ee_poses

def ik_wrapper(compas_fab_ik_fn):
    def fn(pose):
        configurations = compas_fab_ik_fn(frame_from_pose(pose))
        return [np.array(configuration.values) for configuration in configurations if configuration is not None]
    return fn

#####################################

@pytest.mark.collision_check_abb
def test_collision_checker(abb_irb4600_40_255_setup, itj_TC_PG500_cms, itj_beam_cm, column_obstacle_cm, base_plate_cm, viewer):
    # modified from https://github.com/yijiangh/pybullet_planning/blob/dev/tests/test_collisions.py
    urdf_filename, robot = abb_irb4600_40_255_setup

    move_group = 'bare_arm'
    ik_joint_names = robot.get_configurable_joint_names(group=move_group)
    ik_joint_types = robot.get_joint_types_by_names(ik_joint_names)
    flange_link_name = robot.get_end_effector_link_name(group=move_group)

    ee_touched_link_names = ['link_5', 'link_6']

    ee_acms = [AttachedCollisionMesh(ee_cm, flange_link_name, ee_touched_link_names) for ee_cm in itj_TC_PG500_cms]
    beam_acm = AttachedCollisionMesh(itj_beam_cm, flange_link_name, ee_touched_link_names)

    with PyBulletClient(viewer=viewer) as client:
        client.load_robot_from_urdf(urdf_filename)
        client.compas_fab_robot = robot

        # ik_joints = joints_from_names(client.robot_uid, ik_joint_names)
        # tool_link = link_from_name(client.robot_uid, tool_link_name)
        # robot_base_link = link_from_name(client.robot_uid, base_link_name)

        # * add static obstacles
        client.add_collision_mesh(base_plate_cm)
        client.add_collision_mesh(column_obstacle_cm)

        # * add attachment
        for ee_acm in ee_acms:
            client.add_attached_collision_mesh(ee_acm)
        client.add_attached_collision_mesh(beam_acm)

        # safe start conf
        conf = Configuration(values=[0.]*6, types=ik_joint_types, joint_names=ik_joint_names)
        assert not client.configuration_in_collision(conf, group=move_group, options={'diagnosis':True})

        # joint over limit
        conf = Configuration(values=[0., 0., 1.5, 0, 0, 0], types=ik_joint_types, joint_names=ik_joint_names)
        assert client.configuration_in_collision(conf, group=move_group, options={'diagnosis':True})

        # attached beam-robot body self collision
        vals = [0.73303828583761843, -0.59341194567807209, 0.54105206811824214, -0.17453292519943295, 1.064650843716541, 1.7278759594743862]
        conf = Configuration(values=vals, types=ik_joint_types, joint_names=ik_joint_names)
        assert client.configuration_in_collision(conf, group=move_group, options={'diagnosis':True})

        # attached beam-obstacle collision
        # column
        vals = [0.087266462599716474, -0.19198621771937624, 0.20943951023931956, 0.069813170079773182, 1.2740903539558606, 0.069813170079773182]
        conf = Configuration(values=vals, types=ik_joint_types, joint_names=ik_joint_names)
        assert client.configuration_in_collision(conf, group=move_group, options={'diagnosis':True})
        # ground
        vals = [-0.017453292519943295, 0.6108652381980153, 0.20943951023931956, 1.7627825445142729, 1.2740903539558606, 0.069813170079773182]
        conf = Configuration(values=vals, types=ik_joint_types, joint_names=ik_joint_names)
        assert client.configuration_in_collision(conf, group=move_group, options={'diagnosis':True})

        # robot link-obstacle collision
        # column
        vals = [-0.41887902047863912, 0.20943951023931956, 0.20943951023931956, 1.7627825445142729, 1.2740903539558606, 0.069813170079773182]
        conf = Configuration(values=vals, types=ik_joint_types, joint_names=ik_joint_names)
        assert client.configuration_in_collision(conf, group=move_group, options={'diagnosis':True})
        # ground
        vals = [0.33161255787892263, 1.4660765716752369, 0.27925268031909273, 0.17453292519943295, 0.22689280275926285, 0.54105206811824214]
        conf = Configuration(values=vals, types=ik_joint_types, joint_names=ik_joint_names)
        assert client.configuration_in_collision(conf, group=move_group, options={'diagnosis':True})

        # wait_if_gui()

#####################################

@pytest.mark.skip(reason='not done yet')
@pytest.mark.circle_path
def test_circle_cartesian(fixed_waam_setup, viewer):
    urdf_filename, robot, robotA_tool = fixed_waam_setup

    move_group = 'robotA'
    base_link_name = robot.get_base_link_name(group=move_group)
    ik_joint_names = robot.get_configurable_joint_names(group=move_group)
    tool_link_name = robot.get_end_effector_link_name(group=move_group)

    # lower_limits, upper_limits = get_custom_limits(robot, ik_joints)
    # print('joint lower limit: {}'.format(lower_limits))
    # print('joint upper limit: {}'.format(upper_limits))
    # we can also read these velocities from the SRDF file (using e.g. COMPAS_FAB)
    # I'm a bit lazy, just handcode the values here
    vel_limits = {0 : 6.28318530718,
                  1 : 5.23598775598,
                  2 : 6.28318530718,
                  3 : 6.6497044501,
                  4 : 6.77187749774,
                  5 : 10.7337748998}

    base_frame = robot.get_base_frame(group=move_group)
    # base_frame = robot.get_base_frame(group="robotA", full_configuration=init_configuration)
    ik_solver = InverseKinematicsSolver(robot, move_group, ik_abb_irb4600_40_255, base_frame, robotA_tool.frame)

    with PyBulletClient(viewer=viewer) as client:
        client.load_robot_from_urdf(urdf_filename)

        # robot_start_conf = [0,-np.pi/2,np.pi/2,0,0,0]
        # set_joint_positions(robot, ik_joints, robot_start_conf)

        tool_link = link_from_name(client.robot_uid, tool_link_name)
        robot_base_link = link_from_name(client.robot_uid, base_link_name)
        ik_joints = joints_from_names(client.robot_uid, ik_joint_names)

        # * draw EE pose
        tcp_pose = get_link_pose(client.robot_uid, tool_link)
        draw_pose(tcp_pose)

        circle_center = np.array([2, 0, 0.2])
        circle_r = 0.2
        # * generate multiple circles
        # full_angle = np.pi
        # full_angle = 2*2*np.pi
        angle_range = (-0.5*np.pi, 0.5*np.pi)
        # total num of path pts, one path point per 5 degree
        ee_poses = compute_circle_path(circle_center, circle_r, angle_range)

        # # * baseline, keeping the EE z axis rotational dof fixed
        # st_time = time.time()
        # path = plan_cartesian_motion(client.robot_uid, robot_base_link, tool_link, ee_poses)
        # print('Solving time: {}'.format(elapsed_time(st_time)))
        # if path is None:
        #     cprint('Gradient-based ik cartesian planning cannot find a plan!', 'red')
        # else:
        #     cprint('Gradient-based ik cartesian planning find a plan!', 'green')
        #     time_step = 0.03
        #     for conf in path:
        #         set_joint_positions(client.robot_uid, ik_joints, conf)
        #         wait_for_duration(time_step)
        # print('='*20)

        sample_ik_fn = ik_wrapper(ik_solver.inverse_kinematics_function())

        collision_fn = get_collision_fn(client.robot_uid, ik_joints, obstacles=[],
                                               attachments=[], self_collisions=False,
                                            #    disabled_collisions=disabled_collisions,
                                            #    extra_disabled_collisions=extra_disabled_collisions,
                                               custom_limits={})

        ee_vel = 0.005 # m/s

        def get_ee_sample_fn(roll_gen, pitch_gen, yaw_gen):
            def ee_sample_fn(ee_pose):
                # a finite generator
                for roll, pitch, yaw in product(roll_gen, pitch_gen, yaw_gen):
                    yield multiply(ee_pose, Pose(euler=Euler(roll=roll, pitch=pitch, yaw=yaw)))
            return ee_sample_fn

        # by increasing this number we can see the cost go down
        roll_sample_size = 3
        pitch_sample_size = 3
        yaw_sample_size = 3
        delta_roll = np.pi/6
        delta_pitch = np.pi/6
        delta_yaw = np.pi/6
        roll_gen = np.linspace(-delta_roll, delta_roll, num=roll_sample_size)
        pitch_gen = np.linspace(-delta_pitch, delta_pitch, num=pitch_sample_size)
        yaw_gen = np.linspace(-delta_yaw, delta_yaw, num=yaw_sample_size)
        ee_sample_fn = get_ee_sample_fn(roll_gen, pitch_gen, yaw_gen)

        pr = cProfile.Profile()
        pr.enable()

        st_time = time.time()
        path, cost = plan_cartesian_motion_lg(client.robot_uid, ik_joints, ee_poses, sample_ik_fn, collision_fn, sample_ee_fn=ee_sample_fn, \
            custom_vel_limits=vel_limits, ee_vel=ee_vel)
        cprint('Solving time: {}'.format(elapsed_time(st_time)), 'yellow')

        pr.disable()
        pstats.Stats(pr).sort_stats('tottime').print_stats(10)

        if path is None:
            cprint('ladder graph (releasing EE z dof) cartesian planning cannot find a plan!', 'red')
        else:
            cprint('ladder graph (releasing EE z dof) cartesian planning find a plan!', 'cyan')
            cprint('Cost: {}'.format(cost), 'yellow')
            time_step = 0.03
            wait_if_gui('start sim.')
            for conf in path:
                # cprint('conf: {}'.format(conf))
                set_joint_positions(client.robot_uid, ik_joints, conf)
                wait_for_duration(time_step)
        print('='*20)

        wait_if_gui()

#####################################

@pytest.mark.ik
@pytest.mark.parametrize("ik_engine", [('default-single'), ('lobster-analytical'), ('ikfast-analytical')])
def test_ik(fixed_waam_setup, viewer, ik_engine):
    urdf_filename, robot, robotA_tool = fixed_waam_setup

    move_group = 'robotA'
    base_link_name = robot.get_base_link_name(group=move_group)
    ik_joint_names = robot.get_configurable_joint_names(group=move_group)
    tool_link_name = robot.get_end_effector_link_name(group=move_group)
    base_frame = robot.get_base_frame(group=move_group)

    if ik_engine == 'default-single':
        ik_solver = None
    elif ik_engine == 'lobster-analytical':
        ik_solver = InverseKinematicsSolver(robot, move_group, ik_abb_irb4600_40_255, base_frame, robotA_tool.frame)
    elif ik_engine == 'ikfast-analytical':
        import ikfast_abb_irb4600_40_255
        ikfast_fn = get_ik_fn_from_ikfast(ikfast_abb_irb4600_40_255.get_ik)
        ik_solver = InverseKinematicsSolver(robot, move_group, ikfast_fn, base_frame, robotA_tool.frame)
    else:
        raise ValueError('invalid ik engine name.')

    with PyBulletClient(viewer=viewer) as client:
        client.load_robot_from_urdf(urdf_filename)
        client.compas_fab_robot = robot

        ik_joints = joints_from_names(client.robot_uid, ik_joint_names)
        tool_link = link_from_name(client.robot_uid, tool_link_name)
        # robot_base_link = link_from_name(client.robot_uid, base_link_name)

        ee_poses = compute_circle_path()

        if ik_solver is not None:
            client.inverse_kinematics = ik_solver.inverse_kinematics_function()

        ik_time = 0
        # ik function sanity check
        for p in ee_poses:
            frame_WCF = frame_from_pose(p)
            st_time = time.time()
            qs = client.inverse_kinematics(frame_WCF, group=move_group)
            ik_time += elapsed_time(st_time)

            if qs is None:
                # cprint('no ik solution found!', 'red')
                assert False, 'no ik solution found!'
            elif isinstance(qs, list):
                assert len(qs) > 0 and any([qv is not None for qv in qs]), 'no ik solution found'
                if len(qs) > 0:
                    # cprint('{} solutions found!'.format(len(qs)), 'green')
                    for q in randomize(qs):
                        if q is not None:
                            assert isinstance(q, Configuration)
                            set_joint_positions(client.robot_uid, ik_joints, q.values)
                            tcp_pose = get_link_pose(client.robot_uid, tool_link)
                            assert_almost_equal(tcp_pose[0], p[0], decimal=3)
                            assert_almost_equal(quat_angle_between(tcp_pose[1], p[1]), 0, decimal=3)
            elif isinstance(qs, Configuration):
                # cprint('Single solutions found!', 'green')
                q = qs
                set_joint_positions(client.robot_uid, ik_joints, q.values)
                tcp_pose = get_link_pose(client.robot_uid, tool_link)
                assert_almost_equal(tcp_pose[0], p[0], decimal=3)
                assert_almost_equal(quat_angle_between(tcp_pose[1], p[1]), 0, decimal=3)
            else:
                raise ValueError('invalid ik return.')
            # cprint('FK - IK agrees.')
            # wait_if_gui()
        cprint('{} | Average ik time: {} | avg over {} calls.'.format(ik_engine, ik_time/len(ee_poses), len(ee_poses)), 'cyan')

###################################################

@pytest.mark.client
def test_client(fixed_waam_setup, viewer):
    # https://github.com/gramaziokohler/algorithmic_details/blob/e1d5e24a34738822638a157ca29a98afe05beefd/src/algorithmic_details/accessibility/reachability_map.py#L208-L231
    urdf_filename, robot, _ = fixed_waam_setup
    # print('disabled collision: ', get_disabled_collisions(robot.semantics))
    move_group = 'robotA'
    tool_link_name = robot.get_end_effector_link_name(group=move_group)

    with PyBulletClient(viewer=viewer) as client:
        client.load_robot_from_urdf(urdf_filename)

        # * draw EE pose
        tool_link = link_from_name(client.robot_uid, tool_link_name)
        tcp_pose = get_link_pose(client.robot_uid, tool_link)
        draw_pose(tcp_pose)

        assert client.robot_uid in get_bodies()

        wait_if_gui()
