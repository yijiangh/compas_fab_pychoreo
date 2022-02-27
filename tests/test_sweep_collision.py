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
