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
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh, JointConstraint

from pybullet_planning import link_from_name, get_link_pose, draw_pose, get_bodies, multiply, Pose, Euler, set_joint_positions, \
    joints_from_names, quat_angle_between, get_collision_fn, create_obj, unit_pose
from pybullet_planning import wait_if_gui, wait_for_duration, remove_all_debug
from pybullet_planning import plan_cartesian_motion, plan_cartesian_motion_lg
from pybullet_planning import randomize, elapsed_time, BLUE, GREEN, RED

import ikfast_abb_irb4600_40_255

# from compas_fab.backends import PyBulletClient
from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.utils import values_as_list
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo.backend_features.pychoreo_frame_variant_generator import PyChoreoFiniteEulerAngleVariantGenerator

from compas_fab_pychoreo_examples.ik_solver import ik_abb_irb4600_40_255, InverseKinematicsSolver, get_ik_fn_from_ikfast

def compute_circle_path(circle_center=np.array([2, 0, 0.2]), circle_r=0.2, angle_range=(-0.5*np.pi, 0.5*np.pi)):
    # generate a circle path to test IK and Cartesian planning
    ee_poses = []
    n_pt = int(abs(angle_range[1]-angle_range[0]) / (np.pi/180 * 5))
    for a in np.linspace(*angle_range, num=n_pt):
        pt = circle_center + circle_r*np.array([np.cos(a), np.sin(a), 0])
        circ_pose = multiply(Pose(point=pt, euler=Euler(yaw=a+np.pi/2)), Pose(euler=Euler(roll=np.pi*3/4)))
        draw_pose(circ_pose, length=0.01)
        ee_poses.append(circ_pose)
    return ee_poses

def ik_wrapper(compas_fab_ik_fn):
    # convert a compas_fab ik solver to a function that conforms with pybullet_planning convention
    def fn(pose):
        configurations = compas_fab_ik_fn(frame_from_pose(pose))
        return [np.array(configuration.values) for configuration in configurations if configuration is not None]
    return fn

def compute_trajectory_cost(trajectory, init_conf_val=np.zeros(6)):
    cost = np.linalg.norm(init_conf_val - np.array(trajectory.points[0].values))
    for traj_pt1, traj_pt2 in zip(trajectory.points[:-1], trajectory.points[1:]):
        cost += np.linalg.norm(np.array(traj_pt1.values) - np.array(traj_pt2.values))
    return cost

#####################################

@pytest.mark.collision_check_abb
def test_collision_checker(abb_irb4600_40_255_setup, itj_TC_PG500_cms, itj_beam_cm, column_obstacle_cm, base_plate_cm, viewer, diagnosis):
    # modified from https://github.com/yijiangh/pybullet_planning/blob/dev/tests/test_collisions.py
    urdf_filename, semantics = abb_irb4600_40_255_setup

    move_group = 'bare_arm'
    ee_touched_link_names = ['link_5', 'link_6']

    with PyChoreoClient(viewer=viewer) as client:
        robot = client.load_robot(urdf_filename)
        robot.semantics = semantics
        client.disabled_collisions = robot.semantics.disabled_collisions

        ik_joint_names = robot.get_configurable_joint_names(group=move_group)
        ik_joint_types = robot.get_joint_types_by_names(ik_joint_names)
        flange_link_name = robot.get_end_effector_link_name(group=move_group)

        ee_acms = [AttachedCollisionMesh(ee_cm, flange_link_name, ee_touched_link_names) for ee_cm in itj_TC_PG500_cms]
        beam_acm = AttachedCollisionMesh(itj_beam_cm, flange_link_name, ee_touched_link_names)

        # * add static obstacles
        client.add_collision_mesh(base_plate_cm, {})
        client.add_collision_mesh(column_obstacle_cm, {})

        # * add attachment
        for ee_acm in ee_acms:
            client.add_attached_collision_mesh(ee_acm, {'robot': robot, 'mass': 1})
        client.add_attached_collision_mesh(beam_acm, {'robot': robot, 'mass': 1})

        cprint('safe start conf', 'green')
        conf = Configuration(values=[0.]*6, types=ik_joint_types, joint_names=ik_joint_names)
        assert not client.check_collisions(robot, conf, options={'diagnosis':diagnosis})

        cprint('joint over limit', 'cyan')
        conf = Configuration(values=[0., 0., 1.5, 0, 0, 0], types=ik_joint_types, joint_names=ik_joint_names)
        assert client.check_collisions(robot, conf, options={'diagnosis':diagnosis})

        cprint('attached beam-robot body self collision', 'cyan')
        vals = [0.73303828583761843, -0.59341194567807209, 0.54105206811824214, -0.17453292519943295, 1.064650843716541, 1.7278759594743862]
        conf = Configuration(values=vals, types=ik_joint_types, joint_names=ik_joint_names)
        assert client.check_collisions(robot, conf, options={'diagnosis':diagnosis})

        cprint('attached beam-obstacle collision - column', 'cyan')
        vals = [0.087266462599716474, -0.19198621771937624, 0.20943951023931956, 0.069813170079773182, 1.2740903539558606, 0.069813170079773182]
        conf = Configuration(values=vals, types=ik_joint_types, joint_names=ik_joint_names)
        assert client.check_collisions(robot, conf, options={'diagnosis':diagnosis})

        cprint('attached beam-obstacle collision - ground', 'cyan')
        vals = [-0.017453292519943295, 0.6108652381980153, 0.20943951023931956, 1.7627825445142729, 1.2740903539558606, 0.069813170079773182]
        conf = Configuration(values=vals, types=ik_joint_types, joint_names=ik_joint_names)
        assert client.check_collisions(robot, conf, options={'diagnosis':diagnosis})

        cprint('robot link-obstacle collision - column', 'cyan')
        vals = [-0.41887902047863912, 0.20943951023931956, 0.20943951023931956, 1.7627825445142729, 1.2740903539558606, 0.069813170079773182]
        conf = Configuration(values=vals, types=ik_joint_types, joint_names=ik_joint_names)
        assert client.check_collisions(robot, conf, options={'diagnosis':diagnosis})
        cprint('robot link-obstacle collision - ground', 'cyan')
        vals = [0.33161255787892263, 1.4660765716752369, 0.27925268031909273, 0.17453292519943295, 0.22689280275926285, 0.54105206811824214]
        conf = Configuration(values=vals, types=ik_joint_types, joint_names=ik_joint_names)
        assert client.check_collisions(robot, conf, options={'diagnosis':diagnosis})

        wait_if_gui("Finished.")

#####################################
@pytest.mark.frame_gen
def test_frame_variant_generator(viewer):
    pose = unit_pose()
    frame = frame_from_pose(pose)

    options = {'delta_yaw' : np.pi/6, 'yaw_sample_size' : 30}
    frame_gen = PyChoreoFiniteEulerAngleVariantGenerator(options).generate_frame_variant
    with PyChoreoClient(viewer=viewer) as client:
        draw_pose(pose)
        cnt = 0
        for frame in frame_gen(frame):
            draw_pose(pose_from_frame(frame))
            cnt += 1
        assert cnt == options['yaw_sample_size']
        wait_if_gui()
        remove_all_debug()

        # overwrite class options
        cnt = 0
        new_options = {'delta_yaw' : np.pi/3, 'yaw_sample_size' : 60}
        for frame in frame_gen(frame, new_options):
            draw_pose(pose_from_frame(frame))
            cnt += 1
        assert cnt == new_options['yaw_sample_size']
        wait_if_gui()

#####################################
@pytest.mark.circle_cartesian
@pytest.mark.parametrize("planner_ik_conf", [
    ('IterativeIK', 'default-single'),
    ('LadderGraph', 'default-single'),
    ('LadderGraph', 'lobster-analytical'),
    ('LadderGraph', 'ikfast-analytical')
    ])
def test_circle_cartesian(fixed_waam_setup, viewer, planner_ik_conf):
    urdf_filename, semantics, robotA_tool = fixed_waam_setup
    planner_id, ik_engine = planner_ik_conf

    move_group = 'robotA'
    print('\n')
    print('='*10)
    cprint('Cartesian planner {} with IK engine {}'.format(planner_id, ik_engine), 'yellow')

    with PyChoreoClient(viewer=viewer) as client:
        robot = client.load_robot(urdf_filename)
        robot.semantics = semantics
        robot_uid = client.get_robot_pybullet_uid(robot)

        base_link_name = robot.get_base_link_name(group=move_group)
        ik_joint_names = robot.get_configurable_joint_names(group=move_group)
        tool_link_name = robot.get_end_effector_link_name(group=move_group)
        base_frame = robot.get_base_frame(group=move_group)

        if ik_engine == 'default-single':
            ik_solver = None
        elif ik_engine == 'lobster-analytical':
            ik_solver = InverseKinematicsSolver(robot, move_group, ik_abb_irb4600_40_255, base_frame, robotA_tool.frame)
        elif ik_engine == 'ikfast-analytical':
            ikfast_fn = get_ik_fn_from_ikfast(ikfast_abb_irb4600_40_255.get_ik)
            ik_solver = InverseKinematicsSolver(robot, move_group, ikfast_fn, base_frame, robotA_tool.frame)
        else:
            raise ValueError('invalid ik engine name.')

        init_conf = Configuration.from_revolute_values(np.zeros(6), ik_joint_names)

        # replace default ik function with a customized one
        if ik_solver is not None:
            client.planner.inverse_kinematics = ik_solver.inverse_kinematics_function()

        tool_link = link_from_name(robot_uid, tool_link_name)
        robot_base_link = link_from_name(robot_uid, base_link_name)
        ik_joints = joints_from_names(robot_uid, ik_joint_names)

        # * draw EE pose
        tcp_pose = get_link_pose(robot_uid, tool_link)
        draw_pose(tcp_pose)

        # * generate multiple circles
        circle_center = np.array([2, 0, 0.2])
        circle_r = 0.2
        # full_angle = np.pi
        # full_angle = 2*2*np.pi
        angle_range = (-0.5*np.pi, 0.5*np.pi)
        # total num of path pts, one path point per 5 degree
        ee_poses = compute_circle_path(circle_center, circle_r, angle_range)
        ee_frames_WCF = [frame_from_pose(ee_pose) for ee_pose in ee_poses]

        options = {
            'planner_id' : planner_id
            }
        if planner_id == 'LadderGraph':
            client.set_robot_configuration(robot, init_conf)
            st_time = time.time()
            trajectory = client.plan_cartesian_motion(robot, ee_frames_WCF, group=move_group, options=options)
            cprint('W/o frame variant solving time: {}'.format(elapsed_time(st_time)), 'blue')
            cprint('Cost: {}'.format(compute_trajectory_cost(trajectory, init_conf_val=init_conf.values)), 'blue')
            print('-'*5)

            f_variant_options = {'delta_yaw' : np.pi/3, 'yaw_sample_size' : 5}
            options.update({'frame_variant_generator' : PyChoreoFiniteEulerAngleVariantGenerator(options=f_variant_options)})
            print('With frame variant config: {}'.format(f_variant_options))

        client.set_robot_configuration(robot, init_conf)
        st_time = time.time()
        trajectory = client.plan_cartesian_motion(robot, ee_frames_WCF, group=move_group, options=options)
        cprint('{} solving time: {}'.format('With frame variant ' if planner_id == 'LadderGraph' else 'Direct', elapsed_time(st_time)), 'cyan')
        cprint('Cost: {}'.format(compute_trajectory_cost(trajectory, init_conf_val=init_conf.values)), 'cyan')

        if trajectory is None:
            cprint('Client Cartesian planner {} CANNOT find a plan!'.format(planner_id), 'red')
        else:
            cprint('Client Cartesian planning {} find a plan!'.format(planner_id), 'green')
            wait_if_gui('Start sim.')
            time_step = 0.03
            for traj_pt in trajectory.points:
                client.set_robot_configuration(robot, traj_pt)
                wait_for_duration(time_step)

        wait_if_gui()

#####################################
@pytest.mark.plan_motion
def test_plan_motion(abb_irb4600_40_255_setup, itj_TC_PG500_cms, itj_beam_cm, column_obstacle_cm, base_plate_cm, viewer, diagnosis):
    # modified from https://github.com/yijiangh/pybullet_planning/blob/dev/tests/test_collisions.py
    urdf_filename, semantics = abb_irb4600_40_255_setup

    move_group = 'bare_arm'
    ee_touched_link_names = ['link_5', 'link_6']

    with PyChoreoClient(viewer=viewer) as client:
    # with PyBulletClient(connection_type='gui' if viewer else 'direct') as client:
        robot = client.load_robot(urdf_filename)
        robot.semantics = semantics
        # client.disabled_collisions = robot.semantics.disabled_collisions

        ik_joint_names = robot.get_configurable_joint_names(group=move_group)
        ik_joint_types = robot.get_joint_types_by_names(ik_joint_names)
        flange_link_name = robot.get_end_effector_link_name(group=move_group)

        ee_acms = [AttachedCollisionMesh(ee_cm, flange_link_name, ee_touched_link_names) for ee_cm in itj_TC_PG500_cms]
        beam_acm = AttachedCollisionMesh(itj_beam_cm, flange_link_name, ee_touched_link_names)

        # * add static obstacles
        client.add_collision_mesh(base_plate_cm, {})
        client.add_collision_mesh(column_obstacle_cm, {})

        # * add attachment
        for ee_acm in ee_acms:
            client.add_attached_collision_mesh(ee_acm, {'robot': robot, 'mass': 1})
        client.add_attached_collision_mesh(beam_acm, {'robot': robot, 'mass': 1})

        vals = [-1.4660765716752369, -0.22689280275926285, 0.27925268031909273, 0.17453292519943295, 0.22689280275926285, -0.22689280275926285]
        start_conf = Configuration(values=vals, types=ik_joint_types, joint_names=ik_joint_names)
        # client.set_robot_configuration(robot, start_conf)
        # wait_if_gui()

        vals = [0.05235987755982989, -0.087266462599716474, -0.05235987755982989, 1.7104226669544429, 0.13962634015954636, -0.43633231299858238]
        end_conf = Configuration(values=vals, types=ik_joint_types, joint_names=ik_joint_names)
        # client.set_robot_configuration(robot, end_conf)
        # wait_if_gui()

        plan_options = {
            'diagnosis' : True,
            'resolutions' : 0.05
            }

        goal_constraints = robot.constraints_from_configuration(end_conf, [0.01], [0.01])

        st_time = time.time()
        trajectory = client.plan_motion(robot, goal_constraints, start_configuration=start_conf, group=move_group, options=plan_options)
        print('Solving time: {}'.format(elapsed_time(st_time)))

        if trajectory is None:
            cprint('Client motion planner CANNOT find a plan!', 'red')
            # assert False, 'Client motion planner CANNOT find a plan!'
            # TODO warning
        else:
            cprint('Client motion planning find a plan!', 'green')
            wait_if_gui('Start sim.')
            time_step = 0.03
            for traj_pt in trajectory.points:
                client.set_robot_configuration(robot, traj_pt, group=move_group)
                wait_for_duration(time_step)

        wait_if_gui("Finished.")

#####################################

@pytest.mark.ik_abb
@pytest.mark.parametrize("ik_engine", [
        ('default-single'),
        ('lobster-analytical'),
        ('ikfast-analytical')
        ])
def test_ik(fixed_waam_setup, viewer, ik_engine):
    urdf_filename, semantics, robotA_tool = fixed_waam_setup
    move_group = 'robotA'

    with PyChoreoClient(viewer=viewer) as client:
        robot = client.load_robot(urdf_filename)
        robot.semantics = semantics
        robot_uid = client.get_robot_pybullet_uid(robot)

        base_link_name = robot.get_base_link_name(group=move_group)
        ik_joint_names = robot.get_configurable_joint_names(group=move_group)
        tool_link_name = robot.get_end_effector_link_name(group=move_group)
        base_frame = robot.get_base_frame(group=move_group)

        if ik_engine == 'default-single':
            ik_solver = None
        elif ik_engine == 'lobster-analytical':
            ik_solver = InverseKinematicsSolver(robot, move_group, ik_abb_irb4600_40_255, base_frame, robotA_tool.frame)
        elif ik_engine == 'ikfast-analytical':
            ikfast_fn = get_ik_fn_from_ikfast(ikfast_abb_irb4600_40_255.get_ik)
            ik_solver = InverseKinematicsSolver(robot, move_group, ikfast_fn, base_frame, robotA_tool.frame)
        else:
            raise ValueError('invalid ik engine name.')

        ik_joints = joints_from_names(robot_uid, ik_joint_names)
        tool_link = link_from_name(robot_uid, tool_link_name)
        ee_poses = compute_circle_path()

        if ik_solver is not None:
            # replace default ik function with a customized one
            client.planner.inverse_kinematics = ik_solver.inverse_kinematics_function()

        ik_time = 0
        failure_cnt = 0
        # ik function sanity check
        for p in ee_poses:
            frame_WCF = frame_from_pose(p)
            st_time = time.time()
            qs = client.inverse_kinematics(robot, frame_WCF, group=move_group, options={})
            ik_time += elapsed_time(st_time)

            if qs is None:
                cprint('no ik solution found!', 'red')
                # assert False, 'no ik solution found!'
                failure_cnt += 1
            elif isinstance(qs, list):
                if not(len(qs) > 0 and any([qv is not None for qv in qs])):
                    cprint('no ik solution found', 'red')
                    failure_cnt += 1
                if len(qs) > 0:
                    # cprint('{} solutions found!'.format(len(qs)), 'green')
                    for q in randomize(qs):
                        if q is not None:
                            assert isinstance(q, Configuration)
                            client.set_robot_configuration(robot, q)
                            # set_joint_positions(robot_uid, ik_joints, q.values)
                            tcp_pose = get_link_pose(robot_uid, tool_link)
                            assert_almost_equal(tcp_pose[0], p[0], decimal=3)
                            assert_almost_equal(quat_angle_between(tcp_pose[1], p[1]), 0, decimal=3)
            elif isinstance(qs, Configuration):
                # cprint('Single solutions found!', 'green')
                q = qs
                # set_joint_positions(robot_uid, ik_joints, q.values)
                client.set_robot_configuration(robot, q)
                tcp_pose = get_link_pose(robot_uid, tool_link)
                assert_almost_equal(tcp_pose[0], p[0], decimal=3)
                assert_almost_equal(quat_angle_between(tcp_pose[1], p[1]), 0, decimal=3)
            else:
                raise ValueError('invalid ik return.')
            wait_if_gui('FK - IK agrees.')
        cprint('{} | Success {}/{} | Average ik time: {} | avg over {} calls.'.format(ik_engine, len(ee_poses)-failure_cnt, len(ee_poses),
            ik_time/len(ee_poses), len(ee_poses)), 'cyan')

###################################################

@pytest.mark.client
def test_client(fixed_waam_setup, viewer):
    # https://github.com/gramaziokohler/algorithmic_details/blob/e1d5e24a34738822638a157ca29a98afe05beefd/src/algorithmic_details/accessibility/reachability_map.py#L208-L231
    urdf_filename, semantics, _ = fixed_waam_setup
    move_group = 'robotA'

    with PyChoreoClient(viewer=viewer) as client:
        robot = client.load_robot(urdf_filename)
        robot.semantics = semantics
        robot_uid = client.get_robot_pybullet_uid(robot)
        tool_link_name = robot.get_end_effector_link_name(group=move_group)

        # * draw EE pose
        tool_link = link_from_name(robot_uid, tool_link_name)
        tcp_pose = get_link_pose(robot_uid, tool_link)
        draw_pose(tcp_pose)

        assert robot_uid in get_bodies()

        wait_if_gui()
