import numpy as np
from collections import defaultdict
from itertools import product, combinations
from pybullet_planning import get_all_links
from compas_fab_pychoreo.backend_features.sweeping_collision_checker import SweepingCollisionChecker
from compas_fab_pychoreo.utils import is_valid_option, values_as_list, wildcard_keys

import pybullet_planning as pp
from pybullet_planning import get_custom_limits, joints_from_names, link_from_name, get_collision_fn, joint_from_name, \
    expand_links, set_joint_positions, get_custom_limits, get_self_link_pairs, get_moving_links, \
    draw_collision_diagnosis, BASE_LINK, get_num_links, LockRenderer, WorldSaver, draw_point
from pybullet_planning import vertices_from_rigid, Ray, batch_ray_collision, draw_ray, set_camera_pose, draw_ray_result_diagnosis

from pybullet_planning import wait_if_gui, get_body_name, RED, BLUE, set_color, add_line, apply_affine, get_pose, has_gui

def get_attachment_sweeping_collision_fn(robot_body, joints, obstacles=[],
                    attachments=[],
                    extra_disabled_collisions={},
                    body_name_from_id=None, **kwargs):
    attached_bodies = [attachment.child for attachment in attachments]
    vertices_from_body = defaultdict(dict)
    for attached_body in attached_bodies:
        attached_body, body_links = expand_links(attached_body)
        for body_link in body_links:
            local_from_vertices = vertices_from_rigid(attached_body, body_link)
            vertices_from_body[attached_body][body_link] = (local_from_vertices)

    def sweeping_collision_fn(q1, q2, diagnosis=False):
        # * set robot & attachment positions
        line_from_body = defaultdict(dict)
        set_joint_positions(robot_body, joints, q1)
        for attachment in attachments:
            attachment.assign()
            line_from_body[attachment.child] = defaultdict(list)
            for body_link, vertices in vertices_from_body[attachment.child].items():
                updated_vertices = apply_affine(get_pose(attachment.child), vertices)
                line_from_body[attachment.child][body_link].append(updated_vertices)
        set_joint_positions(robot_body, joints, q2)
        for attachment in attachments:
            attachment.assign()
            for body_link, vertices in vertices_from_body[attachment.child].items():
                updated_vertices = apply_affine(get_pose(attachment.child), vertices)
                line_from_body[attachment.child][body_link].append(updated_vertices)

        with LockRenderer(False):
            rays_from_body = defaultdict(dict)
            for body, lines_from_link in line_from_body.items():
                for body_link, lines in lines_from_link.items():
                    rays_from_body[body][body_link] = []
                    for start, end in zip(lines[0], lines[1]):
                        rays_from_body[body][body_link].append(Ray(start, end))
                        # add_line(start, end)

        # * ray - body check
        # ! TODO check if AABB is updated: https://github.com/bulletphysics/bullet3/pull/2900
        # https://github.com/bulletphysics/bullet3/pull/3331
        # ! pybullet.performCollisionDetection ()
        check_bodies = obstacles
        # print('obstacles: ', obstacles)
        with WorldSaver():
            for body, rays_from_link in rays_from_body.items():
                for body_link, rays in rays_from_link.items():
                    for ray, ray_result in zip(rays, batch_ray_collision(rays)):
                        # TODO ignored collisions
                        if ray_result.objectUniqueId in check_bodies:
                            if diagnosis:
                                all_ray_lines = []
                                with LockRenderer():
                                    for r in rays:
                                        all_ray_lines.append(add_line(r.start, r.end))
                                draw_ray_result_diagnosis(ray, ray_result, b1=body, l1=body_link,
                                    point_color=pp.RED, line_color=pp.YELLOW, \
                                    focus_camera=True, body_name_from_id=body_name_from_id)
                                pp.remove_handles(all_ray_lines)
                            return True
        return False

    return sweeping_collision_fn


class PyChoreoSweepingCollisionChecker(SweepingCollisionChecker):
    def __init__(self, client):
        self.client = client

    def step_in_collision(self, robot, configuration_1=None, configuration_2=None, options=None):
        """Check collisions between the sweeping polylines of attached objects' vertices and the obstacles in the scene.

        Parameters
        ----------
        configuration: :class:`compas_fab.robots.Configuration`
        group: str, optional
        options : dict, optional
            Dictionary containing the following key-value pairs:
            - "self_collisions": bool, set to True if checking self collisions of the robot, defaults to True
            - "collision_object_wildcards": list of str, to check against a subset of collision objects, not everything
                in the scene. Each entry of the list is a wildcard rule, learn more at:
                    https://docs.python.org/3/library/re.html
                Example: exact name match: `"^{}$".format(object_name)`
                         partial name match: `"^{}_".format(object_name)` for all objects with the name "object_name_*"

        Returns
        -------
        is_collision : bool
            True if in collision, False otherwise
        """
        options = options or {}
        assert len(configuration_1.joint_names) == len(configuration_2.joint_names), '{} - {}'.format(
                configuration_1.joint_names, configuration_2.joint_names)
        assert len(configuration_1.joint_names) == len(configuration_1.joint_values), '{} - {}'.format(
                configuration_1.joint_names, configuration_1.joint_values)
        diagnosis = options.get('diagnosis', False)
        sweeping_collision_fn = self._get_sweeping_collision_fn(robot, configuration_1.joint_names, options)
        return sweeping_collision_fn(configuration_1.joint_values, configuration_2.joint_values, diagnosis=diagnosis)

    def _get_sweeping_collision_fn(self, robot, joint_names, options=None):
        robot_uid = self.client.get_robot_pybullet_uid(robot)
        # avoid_collisions = options.get('avoid_collisions', True)
        # self_collisions = options.get('self_collisions', True)
        # collision_distance_threshold = options.get('collision_distance_threshold', 0.0)
        # max_distance = options.get('max_distance', 0.0)
        # debug = options.get('debug', False)

        # * custom joint limits
        ik_joints = joints_from_names(robot_uid, joint_names)
        obstacles, attachments, extra_disabled_collisions = self.client._get_collision_checking_setup(options)
        sweeping_collision_fn = get_attachment_sweeping_collision_fn(robot_uid, ik_joints, obstacles=obstacles,
                                        attachments=attachments, body_name_from_id=self.client._name_from_body_id)
        return sweeping_collision_fn
