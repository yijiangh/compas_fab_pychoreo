from compas_fab_pychoreo.backend_features.feature_base import ConfigurationCollisionChecker
from pybullet_planning import joints_from_names, get_collision_fn, joint_from_name

class PyChoreoConfigurationCollisionChecker(ConfigurationCollisionChecker):
    # TODO: overwrite PyBulletClient.check_collisions
    def __init__(self, client):
        self.client = client

    def check_collisions(self, robot, configuration=None, options=None):
        """check collisions between the robot at the given configuration and all the existing obstacles in the scene.

        the collision is checked among:
            1. robot self-collision (if `self_collisions=true`), ignored robot link pairs can be specified in `disabled_collisions`
            2. between (robot links) and (attached objects)
            3. between (robot links, attached objects) and obstacles
        ignored collisions for (2) and (3) can be specified in `extra_disabled_collisions`.

        ! note that:
            - collisions among attached objects are not checked

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
            - "collision_fn": function handle for the collision_fn, this is only for reusing the same
            collision_fn to avoid some computation overhead.
            Default to None, meaning that a new collision_fn is constructed every time the function is called.

        Returns
        -------
        is_collision : bool
            True if in collision, False otherwise
        """
        options = options or {}
        assert len(configuration.joint_names) == len(configuration.joint_values), '{} - {}'.format(
            configuration.joint_names, configuration.joint_values)
        diagnosis = options.get('diagnosis', False)
        collision_fn = options.get('collision_fn', self._get_collision_fn(robot, configuration.joint_names, options))
        return collision_fn(configuration.joint_values, diagnosis=diagnosis)

    def _get_collision_fn(self, robot, joint_names, options=None):
        """Returns a `pybullet_planning` collision_fn
        """
        robot_uid = self.client.get_robot_pybullet_uid(robot)
        joint_custom_limits = options.get('joint_custom_limits', {})
        avoid_collisions = options.get('avoid_collisions', True)
        self_collisions = options.get('self_collisions', True)
        collision_distance_threshold = options.get('collision_distance_threshold', 0.0)
        max_distance = options.get('collision_buffer_distance_threshold', 0.0)
        debug = options.get('debug', False)

        # * custom joint limits
        ik_joints = joints_from_names(robot_uid, joint_names)
        pb_custom_limits = {joint_from_name(robot_uid, jn) : lims \
            for jn, lims in joint_custom_limits.items()}

        obstacles, attachments, extra_disabled_collisions = self.client._get_collision_checking_setup(options)

        collision_fn = get_collision_fn(robot_uid, ik_joints, obstacles=obstacles,
                                        attachments=attachments, self_collisions=avoid_collisions and self_collisions,
                                        disabled_collisions=self.client.get_self_collision_link_ids(robot), # get disabled self-collision links (srdf)
                                        extra_disabled_collisions=extra_disabled_collisions,
                                        custom_limits=pb_custom_limits,
                                        body_name_from_id=self.client._name_from_body_id,
                                        distance_threshold=collision_distance_threshold, max_distance=max_distance)
        return collision_fn
