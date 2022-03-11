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
    ('gripper'),
    ('screwdriver'),
    ('robot'),
    ('obj'),
    ('stl'),
    ])
def test_clone_body(abb_irb4600_40_255_setup, itj_g1_urdf_path, itj_beam_cm, itj_beam_cm_from_stl, itj_s1_urdf_path,
        viewer, body_type):
    with PyChoreoClient(viewer=viewer, verbose=False) as client:
        pp.draw_pose(pp.unit_pose())

        copy_links = None # means all links will be copied
        if body_type == 'gripper':
            tool_id = 'g1'
            client.add_tool_from_urdf(tool_id, itj_g1_urdf_path)
            body = client._get_bodies('^{}$'.format(tool_id))[0]
            copy_links = [pp.link_from_name(body, name) for name in ["gripper_jaw_l"]]
        elif body_type == 'screwdriver':
            tool_id = 's1'
            client.add_tool_from_urdf(tool_id, itj_s1_urdf_path)
            body = client._get_bodies('^{}$'.format(tool_id))[0]
            copy_links = [pp.link_from_name(body, 'gripper_base')]
            # copy_links = [pp.link_from_name(body, name) for name in ['screwdriver_screw']]
        elif body_type == 'robot':
            urdf_filename, _ = abb_irb4600_40_255_setup
            robot = client.load_robot(urdf_filename)
            body = client.get_robot_pybullet_uid(robot)
            copy_links = [pp.link_from_name(body, 'link_4')]
        elif body_type == 'obj':
            client.add_collision_mesh(itj_beam_cm)
            body = client._get_collision_object_bodies('^itj_beam_b2$')[0]
        elif body_type == 'stl':
            client.add_collision_mesh(itj_beam_cm_from_stl)
            body = client._get_collision_object_bodies('^itj_beam_b2$')[0]

        # * full body clone
        cloned_body = pp.clone_body(body, collision=True, visual=False)
        # LOGGER.info(f'full collision body cloned {cloned_body}')
        pp.set_pose(cloned_body, pp.Pose(point=(2,0,0)))
        pp.set_color(cloned_body, pp.RED)

        # * partial body clone
        if copy_links:
            cloned_body = pp.clone_body(body, links=copy_links, collision=True, visual=False)
            # LOGGER.info(f'partial collision body cloned {cloned_body}')
            pp.set_pose(cloned_body, pp.Pose(point=(4,0,0)))
            pp.set_color(cloned_body, pp.BLUE)

        # cloned_body = pp.clone_body(body, collision=False, visual=True)
        # # LOGGER.info(f'visual body cloned {cloned_body}')
        # pp.set_pose(cloned_body, pp.Pose(point=(4,0,0)))
        # pp.set_color(cloned_body, pp.BLUE)

        # client._print_object_summary()
        wait_if_gui()
