import os, time
import pytest

import pybullet_planning as pp
from pybullet_planning import wait_if_gui
from pybullet_planning import elapsed_time, LockRenderer

from compas.geometry import Frame, Transformation
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh
from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.utils import is_configurations_close, verify_trajectory, LOGGER

from testing_utils import get_data_path

#####################################

@pytest.mark.plan_motion
@pytest.mark.parametrize("smooth_iterations", [
    (None),
    (200),
    ])
def test_plan_motion(abb_irb4600_40_255_setup, itj_TC_g1_cms, itj_beam_cm, column_obstacle_cm, base_plate_cm,
    itj_tool_changer_grasp_transf, itj_gripper_grasp_transf, itj_beam_grasp_transf, smooth_iterations,
    itj_tool_changer_urdf_path, itj_g1_urdf_path, abb_tolerances,
    viewer, diagnosis, attempt_iters):
    # modified from https://github.com/yijiangh/pybullet_planning/blob/dev/tests/test_collisions.py
    urdf_filename, semantics = abb_irb4600_40_255_setup

    move_group = 'bare_arm'
    ee_touched_link_names = ['link_6']
    tool_type = 'actuated'

    # compare_seed = False
    # if not compare_seed:
    #     seed = hash(time.time())
    # else:
    #     seed = 961685647856407830
    # pp.set_numpy_seed(seed)
    # pp.set_random_seed(seed)
    seed = pp.get_numpy_seed()
    LOGGER.debug(f'Seed {seed}')

    with PyChoreoClient(viewer=viewer) as client:
        with LockRenderer():
            robot = client.load_robot(urdf_filename)
            robot.semantics = semantics
            client.disabled_collisions = robot.semantics.disabled_collisions

            if tool_type == 'static':
                for _, ee_cm in itj_TC_g1_cms.items():
                    client.add_collision_mesh(ee_cm)
            else:
                client.add_tool_from_urdf('TC', itj_tool_changer_urdf_path)
                client.add_tool_from_urdf('g1', itj_g1_urdf_path)

            # * add static obstacles
            client.add_collision_mesh(base_plate_cm)
            client.add_collision_mesh(column_obstacle_cm)

        ik_joint_names = robot.get_configurable_joint_names(group=move_group)
        ik_joint_types = robot.get_joint_types_by_names(ik_joint_names)
        flange_link_name = robot.get_end_effector_link_name(group=move_group)

        tool0_tf = Transformation.from_frame(client.get_link_frame_from_name(robot, flange_link_name))
        tool0_from_tool_changer_base = itj_tool_changer_grasp_transf
        tool0_from_gripper_base = itj_gripper_grasp_transf
        client.set_object_frame('^{}'.format('TC'), Frame.from_transformation(tool0_tf*tool0_from_tool_changer_base))
        client.set_object_frame('^{}'.format('g1'), Frame.from_transformation(tool0_tf*tool0_from_gripper_base))

        names = client._get_collision_object_names('^{}'.format('g1')) + \
            client._get_collision_object_names('^{}'.format('TC'))
        for ee_name in names:
            attach_options = {'robot' : robot}
            if tool_type == 'actuated':
                attached_child_link_name = 'toolchanger_base' if 'TC' in ee_name else 'gripper_base'
                attach_options.update({'attached_child_link_name' : attached_child_link_name})
            client.add_attached_collision_mesh(AttachedCollisionMesh(CollisionMesh(None, ee_name),
                flange_link_name, touch_links=ee_touched_link_names), options=attach_options)

        #* attach beam
        client.add_collision_mesh(itj_beam_cm)
        tool0_tf = Transformation.from_frame(client.get_link_frame_from_name(robot, flange_link_name))
        tool0_from_beam_base = itj_beam_grasp_transf
        client.set_object_frame('^{}$'.format('itj_beam_b2'), Frame.from_transformation(tool0_tf*tool0_from_beam_base))
        client.add_attached_collision_mesh(AttachedCollisionMesh(CollisionMesh(None, 'itj_beam_b2'),
            flange_link_name, touch_links=[]), options={'robot' : robot})
        # wait_if_gui('beam attached.')

        vals = [-1.4660765716752369, -0.22689280275926285, 0.27925268031909273, 0.17453292519943295, 0.22689280275926285, -0.22689280275926285]
        start_conf = Configuration(joint_values=vals, joint_types=ik_joint_types, joint_names=ik_joint_names)
        # client.set_robot_configuration(robot, start_conf)
        # wait_if_gui()

        # vals = [0.05235987755982989, -0.087266462599716474, -0.05235987755982989, 1.7104226669544429, 0.13962634015954636, -0.43633231299858238]
        vals = [0.034906585039886591, 0.68067840827778847, 0.15707963267948966, -0.89011791851710809, -0.034906585039886591, -2.2514747350726849]
        end_conf = Configuration(joint_values=vals, joint_types=ik_joint_types, joint_names=ik_joint_names)
        # client.set_robot_configuration(robot, end_conf)
        # wait_if_gui()

        options = {
            'diagnosis' : diagnosis,
            'rrt_restarts' : 20,
            'mp_algorithm' : 'birrt',
            'smooth_iterations' : smooth_iterations,
            'verbose' : True,
            'check_sweeping_collision' : True,
            }
        options.update(abb_tolerances)

        goal_constraints = robot.constraints_from_configuration(end_conf, [0.01], [0.01], group=move_group)

        for attempt_i in range(attempt_iters):
            st_time = time.time()
            trajectory = client.plan_motion(robot, goal_constraints, start_configuration=start_conf, group=move_group, options=options)
            if trajectory is None:
                assert False, 'Client motion planner CANNOT find a plan!'
            else:
                # LOGGER.debug('Solve time: {:.2f}, plan length {}'.format(elapsed_time(st_time), len(trajectory.points)))
                assert is_configurations_close(start_conf, trajectory.points[0], fallback_tol=1e-8)
                assert is_configurations_close(end_conf, trajectory.points[-1], fallback_tol=1e-8)
                assert verify_trajectory(client, robot, trajectory, options,
                    failed_traj_save_filename=os.path.join(get_data_path(), 'plan_motion')), \
                    f'-- #{attempt_i}/{attempt_iters}'

                # for traj_pt in trajectory.points:
                #     client.set_robot_configuration(robot, traj_pt)
                #     wait_if_gui()
                # if not compare_seed:
                #     save_trajectory(trajectory, file_name=f'plan_motion_traj_{seed}.json')
                # else:
                #     trajectory_seed = parse_trajectory(file_name=f'plan_motion_traj_{seed}.json')
                #     compare_trajectories(trajectory, trajectory_seed)

######################################################

@pytest.mark.plan_motion_with_polyline
@pytest.mark.parametrize("smooth_iterations", [
    (None),
    (200),
    ])
def test_plan_motion_with_polyline(abb_irb4600_40_255_setup, column_obstacle_cm, base_plate_cm, tube_cms, thin_panel_cm,
    itj_s1_urdf_path, itj_s1_grasp_transf, smooth_iterations,
    viewer, diagnosis, attempt_iters):
    urdf_filename, semantics = abb_irb4600_40_255_setup

    move_group = 'bare_arm'
    ee_touched_link_names = ['link_6']

    seed = pp.get_numpy_seed()
    LOGGER.debug(f'Seed {seed}')

    with PyChoreoClient(viewer=viewer) as client:
        with LockRenderer():
            robot = client.load_robot(urdf_filename)
            robot.semantics = semantics
            client.disabled_collisions = robot.semantics.disabled_collisions

            client.add_tool_from_urdf('s1', itj_s1_urdf_path)

            # * add static obstacles
            client.add_collision_mesh(base_plate_cm)
            client.add_collision_mesh(column_obstacle_cm)
            client.add_collision_mesh(thin_panel_cm)
            # for tube_cm in tube_cms:
            #     client.add_collision_mesh(tube_cm)

        ik_joint_names = robot.get_configurable_joint_names(group=move_group)
        ik_joint_types = robot.get_joint_types_by_names(ik_joint_names)
        flange_link_name = robot.get_end_effector_link_name(group=move_group)

        options = {
            'verbose' : diagnosis,
            'diagnosis' : diagnosis,
            'joint_resolutions' : {jn : 0.3 for jn in ik_joint_names},
            'joint_jump_tolerances' : {jn : 0.3 for jn in ik_joint_names},
            'rrt_restarts' : 2,
            'mp_algorithm' : 'birrt',
            'smooth_iterations' : smooth_iterations,
            }

        tool0_from_s1_base = itj_s1_grasp_transf

        names = client._get_collision_object_names('^{}$'.format('s1'))
        for ee_name in names:
            attach_options = {'robot' : robot}
            attach_options.update({
                'parent_link_from_child_link_transformation' : tool0_from_s1_base,
                })
            client.add_attached_collision_mesh(
                AttachedCollisionMesh(CollisionMesh(None, ee_name),
                                      flange_link_name,
                                      touch_links=ee_touched_link_names),
                                      options=attach_options)

        vals = [0.0, 0.0, 0.0, 0.0, 0.0, 1.2391837689159739]
        start_conf = Configuration(joint_values=vals, joint_types=ik_joint_types, joint_names=ik_joint_names)
        assert not client.check_collisions(robot, start_conf, options=options)
        # client.set_robot_configuration(robot, start_conf)
        # wait_if_gui()

        vals = [0.0, 0.0, 0.59341194567807209, 0.0, 0.0, 1.2391837689159739]
        end_conf = Configuration(joint_values=vals, joint_types=ik_joint_types, joint_names=ik_joint_names)
        assert not client.check_collisions(robot, end_conf, options=options)
        # client.set_robot_configuration(robot, end_conf)
        # wait_if_gui()

        goal_constraints = robot.constraints_from_configuration(end_conf, [0.01], [0.01], group=move_group)

        # LOGGER.info('Linear interpolation without polyline check')
        options['check_sweeping_collision'] = False
        trajectory = client.plan_motion(robot, goal_constraints, start_configuration=start_conf, group=move_group, options=options)
        assert is_configurations_close(start_conf, trajectory.points[0], fallback_tol=1e-8)
        assert is_configurations_close(end_conf, trajectory.points[-1], fallback_tol=1e-8)
        options['check_sweeping_collision'] = True
        assert not verify_trajectory(client, robot, trajectory, options)

        for attempt_i in range(attempt_iters):
            options['check_sweeping_collision'] = True
            st_time = time.time()
            trajectory = client.plan_motion(robot, goal_constraints, start_configuration=start_conf, group=move_group, options=options)

            if trajectory is None:
                assert False, 'Client motion planner CANNOT find a plan!'
            else:
                # LOGGER.debug('Solve time: {:.2f}, path length {}'.format(elapsed_time(st_time), len(trajectory.points)))
                assert is_configurations_close(start_conf, trajectory.points[0], fallback_tol=1e-8)
                assert is_configurations_close(end_conf, trajectory.points[-1], fallback_tol=1e-8)
                assert verify_trajectory(client, robot, trajectory, options), \
                    f'-- #{attempt_i}/{attempt_iters}'
                # if diagnosis:
                #     wait_if_gui('Start sim.')
                #     for traj_pt in trajectory.points:
                #         client.set_robot_configuration(robot, traj_pt)
                #         wait_if_gui()

        wait_if_gui("Finished.")
