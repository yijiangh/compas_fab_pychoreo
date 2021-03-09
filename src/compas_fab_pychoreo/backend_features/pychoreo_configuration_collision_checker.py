from pybullet_planning import BASE_LINK
from compas_fab_pychoreo.backend_features.configuration_collision_checker import ConfigurationCollisionChecker
from compas_fab_pychoreo.utils import is_valid_option, values_as_list, wildcard_keys

from pybullet_planning import set_joint_positions, get_link_pose, get_custom_limits, joints_from_names, link_from_name, \
    get_collision_fn, get_disabled_collisions, WorldSaver, joint_from_name
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
        diagnosis = options.get('diagnosis') or False
        collision_fn = self._get_collision_fn(robot, configuration.joint_names, options)
        return collision_fn(configuration.values, diagnosis=diagnosis)

    def _get_collision_fn(self, robot, joint_names, options=None):
        """Returns a `pybullet_planning` collision_fn
        """
        robot_uid = robot.attributes['pybullet_uid']
        self_collisions = options.get('self_collisions') or True
        custom_limits = options.get('custom_limits') or {}
        avoid_collisions = options.get('avoid_collisions', True)
        distance_threshold = options.get('distance_threshold', 0.0)

        if avoid_collisions:
            wildcards = options.get('collision_object_wildcards') or None
            if wildcards is None:
                # consider all of them
                obstacles = values_as_list(self.client.collision_objects)
            else:
                obstacles = []
                for wc in wildcards:
                    names = wildcard_keys(self.client.collision_objects, wc)
                    for n in names:
                        obstacles.extend(self.client.collision_objects[n])
            # ! doesn't make sense to have a wildcard selection for attached objects
            attachments = values_as_list(self.client.pychoreo_attachments)
            # print('attachment: ', self.client.pychoreo_attachments)
            # print('extra_disabled_collision_links: ', self.client.extra_disabled_collision_links)

            # TODO additional disabled collisions in options
            extra_disabled_collision_names = values_as_list(self.client.extra_disabled_collision_links)
            option_disabled_link_names = options.get('extra_disabled_collisions') or set()
            extra_disabled_collisions = set()
            for bpair in list(extra_disabled_collision_names) + list(option_disabled_link_names):
                b1, b1link_name = bpair[0]
                b2, b2link_name = bpair[1]
                b1_link = BASE_LINK if b1link_name is None else link_from_name(b1, b1link_name)
                b2_link = BASE_LINK if b2link_name is None else link_from_name(b2, b2link_name)
                extra_disabled_collisions.add(
                    ((b1, b1_link), (b2, b2_link))
                    )
        else:
            # only check joint limits, no collision considered
            obstacles = []
            attachments = []
            self_collisions = False

        # * custom joint limits
        ik_joints = joints_from_names(robot_uid, joint_names)
        pb_custom_limits = get_custom_limits(robot_uid, ik_joints,
            custom_limits={joint_from_name(robot_uid, jn) : lims for jn, lims in custom_limits.items()})

        collision_fn = get_collision_fn(robot_uid, ik_joints, obstacles=obstacles,
                                        attachments=attachments, self_collisions=self_collisions,
                                        disabled_collisions=self.client.get_self_collision_link_ids(robot), # get disabled self-collision links (srdf)
                                        extra_disabled_collisions=extra_disabled_collisions,
                                        custom_limits=pb_custom_limits,
                                        body_name_from_id=self.client._name_from_body_id,
                                        distance_threshold=distance_threshold)
        return collision_fn
