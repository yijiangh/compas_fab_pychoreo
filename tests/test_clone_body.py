import os, time
from tabnanny import verbose
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

@pytest.mark.clone_body
@pytest.mark.parametrize("body_type", [
    ('robot'),
    ('tool'),
    ('obj'),
    ('stl'),
    ])
def test_clone_body(abb_irb4600_40_255_setup, itj_g1_urdf_path, itj_beam_cm, itj_beam_cm_from_stl,
        viewer, body_type):
    with PyChoreoClient(viewer=viewer, verbose=False) as client:
        pp.draw_pose(pp.unit_pose())

        if body_type == 'tool':
            tool_id = 'g1'
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
        elif body_type == 'stl':
            client.add_collision_mesh(itj_beam_cm_from_stl)
            body = client._get_collision_object_bodies('^itj_beam_b2$')[0]

        cloned_body = pp.clone_body(body, collision=True, visual=False)
        # LOGGER.info(f'collision body cloned {cloned_body}')
        pp.set_pose(cloned_body, pp.Pose(point=(2,0,0)))
        pp.set_color(cloned_body, pp.RED)

        cloned_body = pp.clone_body(body, collision=False, visual=True)
        # LOGGER.info(f'visual body cloned {cloned_body}')
        pp.set_pose(cloned_body, pp.Pose(point=(4,0,0)))
        pp.set_color(cloned_body, pp.BLUE)

        # client._print_object_summary()
        wait_if_gui()
