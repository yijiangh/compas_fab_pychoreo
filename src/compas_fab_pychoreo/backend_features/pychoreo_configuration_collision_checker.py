from itertools import product, combinations
from pybullet_planning import get_all_links
from compas_fab_pychoreo.backend_features.configuration_collision_checker import ConfigurationCollisionChecker
from compas_fab_pychoreo.utils import is_valid_option, values_as_list, wildcard_keys

from pybullet_planning import get_custom_limits, joints_from_names, link_from_name, get_collision_fn, joint_from_name
from pybullet_planning import wait_if_gui, get_body_name, RED, BLUE, set_color

class PyChoreoConfigurationCollisionChecker(ConfigurationCollisionChecker):
    # TODO: overwrite PyBulletClient.check_collisions
    def __init__(self, client):
        self.client = client

    # def configuration_in_collision(self, configuration, group=None, options=None):
    def check_collisions(self, robot, configuration=None, options=None):
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
        assert len(configuration.joint_names) == len(configuration.values)
        diagnosis = options.get('diagnosis', False)
        collision_fn = self._get_collision_fn(robot, configuration.joint_names, options)
        return collision_fn(configuration.values, diagnosis=diagnosis)

    def _get_collision_fn(self, robot, joint_names, options=None):
        """Returns a `pybullet_planning` collision_fn
        """
        robot_uid = robot.attributes['pybullet_uid']
        custom_limits = options.get('custom_limits', {})
        avoid_collisions = options.get('avoid_collisions', True)
        self_collisions = options.get('self_collisions', True)
        distance_threshold = options.get('distance_threshold', 0.0)
        max_distance = options.get('max_distance', 0.0)
        debug = options.get('debug', False)

        # * custom joint limits
        ik_joints = joints_from_names(robot_uid, joint_names)
        pb_custom_limits = get_custom_limits(robot_uid, ik_joints,
            custom_limits={joint_from_name(robot_uid, jn) : lims for jn, lims in custom_limits.items()})

        obstacles, attachments, extra_disabled_collisions = self.client._get_collision_checking_setup(options)

        collision_fn = get_collision_fn(robot_uid, ik_joints, obstacles=obstacles,
                                        attachments=attachments, self_collisions=avoid_collisions and self_collisions,
                                        disabled_collisions=self.client.get_self_collision_link_ids(robot), # get disabled self-collision links (srdf)
                                        extra_disabled_collisions=extra_disabled_collisions,
                                        custom_limits=pb_custom_limits,
                                        body_name_from_id=self.client._name_from_body_id,
                                        distance_threshold=distance_threshold, max_distance=max_distance)
        return collision_fn
