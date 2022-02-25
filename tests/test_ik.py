import time
import pytest

from numpy.testing import assert_almost_equal
from pybullet_planning import link_from_name, get_link_pose, joints_from_names, quat_angle_between, elapsed_time, \
    randomize, wait_if_gui

from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.conversions import frame_from_pose
from compas_fab_pychoreo.utils import LOGGER
from compas_fab.robots import Configuration

import ikfast_abb_irb4600_40_255
from custom_ik_solver import ik_abb_irb4600_40_255, InverseKinematicsSolver, get_ik_fn_from_ikfast

from testing_utils import compute_circle_path

#####################################

@pytest.mark.ik
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
                LOGGER.error('no ik solution found!')
                # assert False, 'no ik solution found!'
                failure_cnt += 1
            elif isinstance(qs, list):
                if not(len(qs) > 0 and any([qv is not None for qv in qs])):
                    LOGGER.error('no ik solution found')
                    failure_cnt += 1
                if len(qs) > 0:
                    # cprint('{} solutions found!'.format(len(qs)), 'green')
                    for q in randomize(qs):
                        if q is not None:
                            assert isinstance(q, Configuration)
                            client.set_robot_configuration(robot, q)
                            tcp_pose = get_link_pose(robot_uid, tool_link)
                            assert_almost_equal(tcp_pose[0], p[0], decimal=3)
                            assert_almost_equal(quat_angle_between(tcp_pose[1], p[1]), 0, decimal=3)
            elif isinstance(qs, Configuration):
                # cprint('Single solutions found!', 'green')
                q = qs
                client.set_robot_configuration(robot, q)
                tcp_pose = get_link_pose(robot_uid, tool_link)
                assert_almost_equal(tcp_pose[0], p[0], decimal=3)
                assert_almost_equal(quat_angle_between(tcp_pose[1], p[1]), 0, decimal=3)
            else:
                raise ValueError('invalid ik return.')
            wait_if_gui('FK - IK agrees.')
        LOGGER.debug('{} | Success {}/{} | Average ik time: {:.2f} | avg over {} calls.'.format(ik_engine, len(ee_poses)-failure_cnt, len(ee_poses),
            ik_time/len(ee_poses), len(ee_poses)))
