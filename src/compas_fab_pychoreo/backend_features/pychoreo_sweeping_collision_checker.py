from collections import defaultdict
from itertools import product, combinations
from pybullet_planning import get_all_links
from compas_fab_pychoreo.backend_features.sweeping_collision_checker import SweepingCollisionChecker
from compas_fab_pychoreo.utils import is_valid_option, values_as_list, wildcard_keys

from pybullet_planning import get_custom_limits, joints_from_names, link_from_name, get_collision_fn, joint_from_name, \
    expand_links, set_joint_positions, get_custom_limits, get_self_link_pairs, get_moving_links, \
    draw_collision_diagnosis, BASE_LINK, get_num_links
from pybullet_planning import vertices_from_rigid, Ray, batch_ray_collision, draw_ray

# from pybullet_planning import wait_if_gui, get_body_name, RED, BLUE, set_color,

def get_attachment_sweeping_collision_fn(body, joints, obstacles=[],
                    attachments=[],
                    extra_disabled_collisions={},
                    body_name_from_id=None, **kwargs):
    attached_bodies = [attachment.child for attachment in attachments]

    def sweeping_collision_fn(q1, q2, diagnosis=False):
        lines_from_body = defaultdict(list)
        # * set body & attachment positions
        set_joint_positions(body, joints, q1)
        for attachment in attachments:
            attachment.assign()
        for attached_body in attached_bodies:
            attached_body, body_links = expand_links(attached_body)
            for body_link in body_links:
                if get_num_links(attached_body) != 0 and body_link == BASE_LINK:
                    continue
                lines_from_body[attached_body].append(vertices_from_rigid(attached_body, body_link))

        set_joint_positions(body, joints, q2)
        for attachment in attachments:
            attachment.assign()
        for attached_body in attached_bodies:
            attached_body, body_links = expand_links(attached_body)
            for body_link in body_links:
                if get_num_links(attached_body) != 0 and body_link == BASE_LINK:
                    continue
                lines_from_body[attached_body].append(vertices_from_rigid(attached_body, body_link))

        rays = []
        for lines in lines_from_body.values():
            for start, end in zip(lines[0], lines[1]):
                rays.append(Ray(start, end))

        # * body - body check
        for ray, ray_result in zip(rays, batch_ray_collision(rays)):
            if ray_result.objectUniqueId in obstacles:
                if diagnosis:
                    draw_ray(ray, ray_result)
                return True
        return False

    return sweeping_collision_fn


class PyChoreoSweepingCollisionChecker(SweepingCollisionChecker):
    def __init__(self, client):
        self.client = client

    def step_in_collision(self, robot, configuration_1=None, configuration_2=None, options=None):
        """[summary]

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
        assert len(configuration_1.joint_names) == len(configuration_1.values), '{} - {}'.format(
                configuration_1.joint_names, configuration_1.values)
        diagnosis = options.get('diagnosis', False)
        sweeping_collision_fn = self._get_sweeping_collision_fn(robot, configuration_1.joint_names, options)
        return sweeping_collision_fn(configuration_1.values, configuration_2.values, diagnosis=diagnosis)

    def _get_sweeping_collision_fn(self, robot, joint_names, options=None):
        robot_uid = robot.attributes['pybullet_uid']
        avoid_collisions = options.get('avoid_collisions', True)
        self_collisions = options.get('self_collisions', True)
        distance_threshold = options.get('distance_threshold', 0.0)
        max_distance = options.get('max_distance', 0.0)
        debug = options.get('debug', False)

        # * custom joint limits
        ik_joints = joints_from_names(robot_uid, joint_names)
        obstacles, attachments, extra_disabled_collisions = self.client._get_collision_checking_setup(options)
        sweeping_collision_fn = get_attachment_sweeping_collision_fn(robot_uid, ik_joints, obstacles=obstacles,
                                        attachments=attachments)
        return sweeping_collision_fn
