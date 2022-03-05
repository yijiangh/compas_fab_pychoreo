import os, time
import numpy as np
import pytest
from collections import defaultdict

import pybullet_planning as pp
from pybullet_planning import wait_if_gui
from pybullet_planning import elapsed_time, LockRenderer

from compas.robots import Joint
from compas.geometry import Frame, Transformation
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh
from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.utils import is_configurations_close, verify_trajectory, LOGGER

from testing_utils import get_data_path

###################################

@pytest.mark.vertices_update
@pytest.mark.parametrize("body_type", [
    ('tool'),
    ('robot'),
    ('static'),
    ])
def test_vertices_update(abb_irb4600_40_255_setup, itj_g1_urdf_path, itj_beam_cm,
        viewer, body_type):
    with PyChoreoClient(viewer=viewer) as client:
        tool_id = 'g1'
        pp.draw_pose(pp.unit_pose())

        if body_type == 'tool':
            client.add_tool_from_urdf(tool_id, itj_g1_urdf_path)
            body = client._get_bodies('^{}$'.format(tool_id))[0]
            conf = Configuration([0.01, 0.01], [Joint.PRISMATIC, Joint.PRISMATIC],
                ["joint_gripper_jaw_l", "joint_gripper_jaw_r"])
        elif body_type == 'robot':
            urdf_filename, _ = abb_irb4600_40_255_setup
            robot = client.load_robot(urdf_filename)
            body = client.get_robot_pybullet_uid(robot)

            ik_joint_names = robot.get_configurable_joint_names(group='bare_arm')
            ik_joint_types = robot.get_joint_types_by_names(ik_joint_names)
            vals = [0.1,0,0,0,0,0]
            conf = Configuration(joint_values=vals, joint_types=ik_joint_types, joint_names=ik_joint_names)
        elif body_type == 'static':
            client.add_collision_mesh(itj_beam_cm)
            body = client._get_collision_object_bodies('^itj_beam_b2$')[0]

        # print('initial joint conf: ', pp.get_joint_positions(body, pp.get_movable_joints(body)))
        pp.set_pose(body, pp.Pose(point=(2,0,0)))
        for link in pp.get_all_links(body):
            pp.set_color(body, link=link, color=pp.apply_alpha(pp.GREY, 0.6))

        old_link_from_vertices = pp.get_body_collision_vertices(body)
        with LockRenderer():
            for body_link, vertices in old_link_from_vertices.items():
                old_link_from_vertices[body_link] = vertices
                for vert in vertices:
                    pp.draw_point(vert, size=0.01, color=pp.BLUE)

        wait_if_gui('first conf')

        if body_type != 'static':
            client._set_body_configuration(body, conf)
        else:
            pp.set_pose(body, pp.Pose(point=(2.1,0,0)))

        new_link_from_vertices = pp.get_body_collision_vertices(body)
        with LockRenderer():
            for body_link, vertices in new_link_from_vertices.items():
                new_link_from_vertices[body_link] = vertices
                for old_vert, vert in zip(old_link_from_vertices[body_link], vertices):
                    pp.draw_point(vert, size=0.01, color=pp.RED)
                    pp.add_line(old_vert, vert)

        wait_if_gui('Finish')

@pytest.mark.polyline_collision_check
def test_polyline_collision_check(abb_irb4600_40_255_setup, itj_TC_g1_cms, itj_beam_cm, column_obstacle_cm, base_plate_cm,
    itj_tool_changer_grasp_transf, itj_gripper_grasp_transf, itj_beam_grasp_transf,
    itj_tool_changer_urdf_path, itj_g1_urdf_path,
    viewer, diagnosis):
    urdf_filename, semantics = abb_irb4600_40_255_setup

    move_group = 'bare_arm'
    ee_touched_link_names = ['link_6']

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
            attached_child_link_name = 'toolchanger_base' if 'TC' in ee_name else 'gripper_base'
            attach_options.update({'attached_child_link_name' : attached_child_link_name})
            client.add_attached_collision_mesh(AttachedCollisionMesh(CollisionMesh(None, ee_name),
                flange_link_name, touch_links=ee_touched_link_names), options=attach_options)

        # * gripper (prismatic joint) sublink polyline check when updating tool configuration
        # lower 0.0008 upper 0.01
        # vals = [-0.36302848441482055, -0.24434609527920614, -0.12217304763960307, 0.0, 0.0, 1.5707963267948966]
        # conf = Configuration(vals, ik_joint_types, ik_joint_names)
        # # client.set_robot_configuration(robot, conf)

        # tool_bodies = client._get_bodies('^{}'.format('g1'))
        # tool_conf = Configuration(joint_values=[0.0008, 0.0008], joint_types=[Joint.PRISMATIC, Joint.PRISMATIC],
        #     joint_names=['joint_gripper_jaw_l', 'joint_gripper_jaw_r'])
        # for b in tool_bodies:
        #     client._set_body_configuration(b, tool_conf)
        # # wait_if_gui('Close Gripper')
        # LOGGER.debug('no-collision when gripper closed')
        # assert not client.check_collisions(robot, conf, options={'diagnosis':diagnosis})

        # tool_conf = Configuration(joint_values=[0.01, 0.01], joint_types=[Joint.PRISMATIC, Joint.PRISMATIC],
        #     joint_names=['joint_gripper_jaw_l', 'joint_gripper_jaw_r'])
        # for b in tool_bodies:
        #     client._set_body_configuration(b, tool_conf)
        # # wait_if_gui('Open Gripper')
        # LOGGER.debug('in-collision when gripper opened')
        # assert client.check_collisions(robot, conf, options={'diagnosis':diagnosis})

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

        LOGGER.debug('Sweeping collision')
        vals = [-0.12217304763960307, -0.73303828583761843, 0.83775804095727824, -2.4609142453120048, 1.2391837689159739, -0.85521133347722145]
        import numpy as np
        # vals = list(np.zeros(6))
        conf1 = Configuration(vals, ik_joint_types, ik_joint_names)
        assert not client.check_collisions(robot, conf1, options={'diagnosis':diagnosis})
        wait_if_gui('first conf')

        vals = [-0.12217304763960307, -0.73303828583761843, 0.83775804095727824, -2.4958208303518914, -1.5533430342749532, -0.85521133347722145]
        conf2 = Configuration(vals, ik_joint_types, ik_joint_names)
        assert not client.check_collisions(robot, conf2, options={'diagnosis':diagnosis})
        wait_if_gui('second conf')

        assert client.check_sweeping_collisions(robot, conf1, conf2, options={'diagnosis':diagnosis})

        wait_if_gui("Finished.")


# TODO revolute_joint_tool_polyline_check
# * gripper (prismatic joint) sublink polyline check when updating tool configuration
