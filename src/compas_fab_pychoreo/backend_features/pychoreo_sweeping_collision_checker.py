from compas.robots.model import robot
import numpy as np
from collections import defaultdict
from itertools import product, combinations

from pybullet_planning.interfaces.env_manager.pose_transformation import invert
from pybullet_planning.interfaces.env_manager.user_io import wait_if_gui
from .sweeping_collision_checker import SweepingCollisionChecker
from ..utils import LOGGER
from ..client import PyChoreoClient

import pybullet_planning as pp
from pybullet_planning import joints_from_names, expand_links, set_joint_positions, LockRenderer, WorldSaver
from pybullet_planning import vertices_from_rigid, Ray, batch_ray_collision, draw_ray_result_diagnosis
from pybullet_planning import add_line, apply_affine, get_pose

def get_attachment_sweeping_collision_fn(robot_body, joints, obstacles=[],
                    attachments=[],
                    extra_disabled_collisions={},
                    body_name_from_id=None, **kwargs):
    # ! the attached multi-link bodies are assumed to be at the same configuration between q1 and q2
    # TODO get conf if attachment itself is a multi-link body with moving joints
    cached_vertices_from_body = defaultdict(dict)

    for attachment in attachments:
        attached_body = attachment.child
        attach_conf = pp.get_joint_positions(attached_body, pp.get_movable_joints(attached_body))
        try:
            # ! clone body is for handling joint-based URDF imports, should investigate why this is needed
            with pp.HideOutput():
                attached_body_clone = pp.clone_body(attached_body, visual=False, collision=True)
        except:
            attached_body_clone = attached_body
        _, body_links = expand_links(attached_body_clone)
        set_joint_positions(attached_body_clone, pp.get_movable_joints(attached_body_clone), attach_conf)
        for body_link in body_links:
            # ! for some reasons, pybullet cloned body only has collision shapes, and saves them as visual shapes
            local_from_vertices = vertices_from_rigid(attached_body_clone, body_link, collision=attached_body == attached_body_clone)
            cached_vertices_from_body[attached_body][body_link] = local_from_vertices
        if attached_body_clone != attached_body:
            pp.remove_body(attached_body_clone)

    def sweeping_collision_fn(q1, q2, diagnosis=False):
        # * set robot & attachment positions
        line_from_body = {attachment.child : defaultdict(list) for attachment in attachments}

        def append_vertices_from_conf(line_from_body, q):
            set_joint_positions(robot_body, joints, q)
            for attachment in attachments:
                attachment.assign()
                for body_link, vertices in cached_vertices_from_body[attachment.child].items():
                    world_from_current_link_pose = pp.get_link_pose(attachment.child, body_link)
                    updated_vertices = apply_affine(world_from_current_link_pose, vertices)
                    line_from_body[attachment.child][body_link].append(updated_vertices)
            return line_from_body

        line_from_body = append_vertices_from_conf(line_from_body, q1)
        line_from_body = append_vertices_from_conf(line_from_body, q2)
        rays_from_body = defaultdict(dict)
        for body, lines_from_link in line_from_body.items():
            for body_link, lines in lines_from_link.items():
                rays_from_body[body][body_link] = []
                for start, end in zip(lines[0], lines[1]):
                    rays_from_body[body][body_link].append(Ray(start, end))

        # * ray - body check
        # ! TODO check if AABB is updated: https://github.com/bulletphysics/bullet3/pull/2900
        # https://github.com/bulletphysics/bullet3/pull/3331
        # ! pybullet.performCollisionDetection ()
        check_bodies = obstacles
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
                            with LockRenderer():
                                pp.remove_handles(all_ray_lines)
                        return True
        return False

    return sweeping_collision_fn


class PyChoreoSweepingCollisionChecker(SweepingCollisionChecker):
    def __init__(self, client: PyChoreoClient):
        self.client = client

    def check_sweeping_collisions(self, robot, configuration_1=None, configuration_2=None, options=None):
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
        diagnosis = options.get('diagnosis', False)
        if configuration_1.joint_names != configuration_2.joint_names:
            _conf1 = configuration_1.copy()
            _conf2 = configuration_2.copy()
            if configuration_1.joint_names < configuration_2.joint_names:
                _conf1 = _conf2.merged(_conf1)
            else:
                _conf2 = _conf1.merged(_conf2)
        else:
            _conf1 = configuration_1
            _conf2 = configuration_2
        sweeping_collision_fn = options.get('sweeping_collision_fn',
            self._get_sweeping_collision_fn(robot, _conf1.joint_names, options)) # for reusing sweeping function
        return sweeping_collision_fn(_conf1.joint_values, _conf2.joint_values, diagnosis=diagnosis)

    def _get_sweeping_collision_fn(self, robot, joint_names, options=None):
        robot_uid = self.client.get_robot_pybullet_uid(robot)
        # avoid_collisions = options.get('avoid_collisions', True)
        # self_collisions = options.get('self_collisions', True)
        # collision_distance_threshold = options.get('collision_distance_threshold', 0.0)
        # max_distance = options.get('collision_buffer_distance_threshold', 0.0)
        # debug = options.get('debug', False)

        # * custom joint limits
        ik_joints = joints_from_names(robot_uid, joint_names)
        obstacles, attachments, extra_disabled_collisions = self.client._get_collision_checking_setup(options)
        sweeping_collision_fn = get_attachment_sweeping_collision_fn(robot_uid, ik_joints, obstacles=obstacles,
                                        attachments=attachments, body_name_from_id=self.client._name_from_body_id)
        return sweeping_collision_fn
