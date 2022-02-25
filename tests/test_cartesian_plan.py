import pytest
import numpy as np
import time

from compas.geometry import Frame, Transformation
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh

from pybullet_planning import link_from_name, get_link_pose, draw_pose, multiply, Pose, Euler, \
    joints_from_names, quat_angle_between, unit_pose
from pybullet_planning import wait_if_gui, remove_all_debug
from pybullet_planning import randomize, elapsed_time, LockRenderer

from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo.backend_features.pychoreo_frame_variant_generator import PyChoreoFiniteEulerAngleVariantGenerator
from compas_fab_pychoreo.utils import LOGGER

import ikfast_abb_irb4600_40_255
from custom_ik_solver import ik_abb_irb4600_40_255, InverseKinematicsSolver, get_ik_fn_from_ikfast

from testing_utils import compute_trajectory_cost, compute_circle_path

####################################################

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
    LOGGER.debug('Cartesian planner {} with IK engine {}'.format(planner_id, ik_engine))

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

            # options.update({'ik_function' : lambda pose: compute_inverse_kinematics(ikfast_abb_irb4600_40_255.get_ik, pose, sampled=[])})

            with LockRenderer():
                trajectory = client.plan_cartesian_motion(robot, ee_frames_WCF, group=move_group, options=options)
            LOGGER.debug('W/o frame variant solving time: {:.2f}'.format(elapsed_time(st_time)),)
            LOGGER.debug('Cost: {}'.format(compute_trajectory_cost(trajectory, init_conf_val=init_conf.joint_values)))
            print('-'*5)

            f_variant_options = {'delta_yaw' : np.pi/3, 'yaw_sample_size' : 5}
            options.update({'frame_variant_generator' : PyChoreoFiniteEulerAngleVariantGenerator(options=f_variant_options)})
            print('With frame variant config: {}'.format(f_variant_options))

        client.set_robot_configuration(robot, init_conf)
        st_time = time.time()
        with LockRenderer():
            trajectory = client.plan_cartesian_motion(robot, ee_frames_WCF, group=move_group, options=options)
        LOGGER.debug('{} solving time: {:.2f}'.format('With frame variant ' if planner_id == 'LadderGraph' else 'Direct', elapsed_time(st_time)))
        LOGGER.debug('Cost: {}'.format(compute_trajectory_cost(trajectory, init_conf_val=init_conf.joint_values)))

        if trajectory is None:
            LOGGER.debug('Client Cartesian planner {} CANNOT find a plan!'.format(planner_id))
        else:
            LOGGER.debug('Client Cartesian planning {} find a plan!'.format(planner_id))
            wait_if_gui('Start sim.')
            for traj_pt in trajectory.points:
                client.set_robot_configuration(robot, traj_pt)
                wait_if_gui()
