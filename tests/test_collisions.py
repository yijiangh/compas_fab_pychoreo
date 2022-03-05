import pytest
import json

from pybullet_planning import wait_if_gui, LockRenderer

from compas.utilities import DataDecoder
from compas.geometry import Frame, Transformation
from compas.robots import Joint
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh
from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.utils import LOGGER, verify_trajectory

#####################################

@pytest.mark.collision_check
@pytest.mark.parametrize("tool_type", [
    ('static'),
    ('actuated'),
    ])
def test_collision_checker(abb_irb4600_40_255_setup, itj_TC_g1_cms, itj_beam_cm, column_obstacle_cm, base_plate_cm,
    itj_tool_changer_grasp_transf, itj_gripper_grasp_transf, itj_beam_grasp_transf, tool_type,
    itj_tool_changer_urdf_path, itj_g1_urdf_path,
    viewer, diagnosis):
    # modified from https://github.com/yijiangh/pybullet_planning/blob/dev/tests/test_collisions.py
    urdf_filename, semantics = abb_irb4600_40_255_setup

    move_group = 'bare_arm'
    ee_touched_link_names = ['link_6']

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
        # wait_if_gui()

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
        # client._print_object_summary()
        # wait_if_gui('EE attached.')

        if tool_type == 'actuated':
            # * attached tool sublink collision
            # lower 0.0008 upper 0.01
            vals = [-0.36302848441482055, -0.24434609527920614, -0.12217304763960307, 0.0, 0.0, 1.5707963267948966]
            conf = Configuration(vals, ik_joint_types, ik_joint_names)
            # client.set_robot_configuration(robot, conf)

            tool_bodies = client._get_bodies('^{}'.format('g1'))
            tool_conf = Configuration(joint_values=[0.0008, 0.0008], joint_types=[Joint.PRISMATIC, Joint.PRISMATIC],
                joint_names=['joint_gripper_jaw_l', 'joint_gripper_jaw_r'])
            for b in tool_bodies:
                client._set_body_configuration(b, tool_conf)
            # wait_if_gui('Close Gripper')
            LOGGER.debug('no-collision when gripper closed')
            assert not client.check_collisions(robot, conf, options={'diagnosis':diagnosis})

            tool_conf = Configuration(joint_values=[0.01, 0.01], joint_types=[Joint.PRISMATIC, Joint.PRISMATIC],
                joint_names=['joint_gripper_jaw_l', 'joint_gripper_jaw_r'])
            for b in tool_bodies:
                client._set_body_configuration(b, tool_conf)
            # wait_if_gui('Open Gripper')
            LOGGER.debug('in-collision when gripper opened')
            assert client.check_collisions(robot, conf, options={'diagnosis':diagnosis})

        LOGGER.debug('safe start conf')
        conf = Configuration([0.]*6, ik_joint_types, ik_joint_names)
        assert not client.check_collisions(robot, conf, options={'diagnosis':diagnosis})

        LOGGER.debug('joint over limit')
        conf = Configuration([0., 0., 1.5, 0, 0, 0], ik_joint_types, ik_joint_names)
        assert client.check_collisions(robot, conf, options={'diagnosis':diagnosis})

        LOGGER.debug('attached gripper-obstacle collision - column')
        vals = [-0.33161255787892263, -0.43633231299858238, 0.43633231299858238, -1.0471975511965976, 0.087266462599716474, 0.0]
        # conf = Configuration(vals, ik_joint_types, ik_joint_names)
        # client.set_robot_configuration(robot, conf)
        # wait_if_gui()
        assert client.check_collisions(robot, conf, options={'diagnosis':diagnosis})

        #* attach beam
        client.add_collision_mesh(itj_beam_cm)
        tool0_tf = Transformation.from_frame(client.get_link_frame_from_name(robot, flange_link_name))
        tool0_from_beam_base = itj_beam_grasp_transf
        client.set_object_frame('^{}$'.format('itj_beam_b2'), Frame.from_transformation(tool0_tf*tool0_from_beam_base))
        client.add_attached_collision_mesh(AttachedCollisionMesh(CollisionMesh(None, 'itj_beam_b2'),
            flange_link_name, touch_links=[]), options={'robot' : robot})
        # wait_if_gui('beam attached.')

        if diagnosis:
            client._print_object_summary()

        LOGGER.debug('attached beam-robot body self collision')
        vals = [0.73303828583761843, -0.59341194567807209, 0.54105206811824214, -0.17453292519943295, 1.064650843716541, 1.7278759594743862]
        conf = Configuration(vals, ik_joint_types, ik_joint_names)
        assert client.check_collisions(robot, conf, options={'diagnosis':diagnosis})

        LOGGER.debug('attached beam-obstacle collision - column')
        vals = [0.087266462599716474, -0.19198621771937624, 0.20943951023931956, 0.069813170079773182, 1.2740903539558606, 0.069813170079773182]
        conf = Configuration(vals, ik_joint_types, ik_joint_names)
        assert client.check_collisions(robot, conf, options={'diagnosis':diagnosis})

        LOGGER.debug('attached beam-obstacle collision - ground')
        vals = [-0.017453292519943295, 0.6108652381980153, 0.20943951023931956, 1.7627825445142729, 1.2740903539558606, 0.069813170079773182]
        conf = Configuration(vals, ik_joint_types, ik_joint_names)
        assert client.check_collisions(robot, conf, options={'diagnosis':diagnosis})

        LOGGER.debug('robot link-obstacle collision - column')
        vals = [-0.41887902047863912, 0.20943951023931956, 0.20943951023931956, 1.7627825445142729, 1.2740903539558606, 0.069813170079773182]
        conf = Configuration(vals, ik_joint_types, ik_joint_names)
        assert client.check_collisions(robot, conf, options={'diagnosis':diagnosis})

        LOGGER.debug('robot link-obstacle collision - ground')
        vals = [0.33161255787892263, 1.4660765716752369, 0.27925268031909273, 0.17453292519943295, 0.22689280275926285, 0.54105206811824214]
        conf = Configuration(vals, ik_joint_types, ik_joint_names)
        assert client.check_collisions(robot, conf, options={'diagnosis':diagnosis})

        wait_if_gui("Finished.")

#####################################

@pytest.mark.sensitive_collision
def test_sensitive_collision(abb_irb4600_40_255_setup, column_obstacle_cm, base_plate_cm, thin_panel_cm,
    itj_tool_changer_urdf_path, itj_g1_urdf_path, itj_tool_changer_grasp_transf, itj_gripper_grasp_transf,
    itj_beam_cm, itj_beam_grasp_transf, collided_traj_path,
    abb_tolerances, viewer, diagnosis):
    urdf_filename, semantics = abb_irb4600_40_255_setup

    move_group = 'bare_arm'
    ee_touched_link_names = ['link_6']
    attempt_iters = 1

    with PyChoreoClient(viewer=viewer) as client:
        with LockRenderer():
            robot = client.load_robot(urdf_filename)
            robot.semantics = semantics
            client.disabled_collisions = robot.semantics.disabled_collisions
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

        options = {
            'diagnosis' : diagnosis,
            'verbose' : True,
            'check_sweeping_collision' : True,
            }
        options.update(abb_tolerances)

        for _ in range(attempt_iters):
            conf_vals = [
                [-0.3681982760197108, 0.14979060281981388, -0.5807809741625514, -1.0349889615656802, -0.38781559783097475, -3.020799755151662],
                [-0.9910463102538172, -0.06965373786911955, -0.5600690622479962, -2.4668820270868905, 0.2548571223428986, 1.5310114818423113],
            ]
            for vals in conf_vals:
                conf = Configuration(joint_values=vals, joint_types=ik_joint_types, joint_names=ik_joint_names)
                assert client.check_collisions(robot, conf, options=options)
                # client.set_robot_configuration(robot, start_conf)
                # wait_if_gui()

            with open(collided_traj_path, 'r') as f:
                trajectory = json.load(f, cls=DataDecoder)
                assert not verify_trajectory(client, robot, trajectory, options)[0]
