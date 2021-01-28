from compas_fab_pychoreo.backend_features.configuration_collision_checker import ConfigurationCollisionChecker
from compas_fab_pychoreo.utils import is_valid_option, values_as_list

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

        Returns
        -------
        is_collision : bool
            True if in collision, False otherwise
        """
        assert len(configuration.joint_names) == len(configuration.values)
        diagnosis = is_valid_option(options, 'diagnosis', False)
        collision_fn = self._get_collision_fn(robot, configuration.joint_names, options)
        return collision_fn(configuration.values, diagnosis=diagnosis)

    def _get_collision_fn(self, robot, joint_names, options=None):
        """Returns a `pybullet_planning` collision_fn
        """
        robot_uid = robot.attributes['pybullet_uid']
        self_collisions = is_valid_option(options, 'self_collisions', True)
        custom_limits = is_valid_option(options, 'custom_limits', {})

        # joint_names = robot.get_configurable_joint_names(group=group)
        ik_joints = joints_from_names(robot_uid, joint_names)
        obstacles = values_as_list(self.client.collision_objects)
        # // ConstraintInfo = namedtuple('ConstraintInfo', ['constraint_id', 'body_id', 'robot_uid'])
        attachments = values_as_list(self.client.pychoreo_attachments)

        pb_custom_limits = get_custom_limits(robot_uid, ik_joints,
            custom_limits={joint_from_name(robot_uid, jn) : lims for jn, lims in custom_limits.items()})

        # TODO additional disabled collisions in options
        # option_disabled_linke_names = is_valid_option(options, 'extra_disabled_collisions', [])
        # option_extra_disabled_collisions = get_body_body_disabled_collisions(robot_uid, workspace, extra_disabled_link_names)
        option_extra_disabled_collisions = set()

        collision_fn = get_collision_fn(robot_uid, ik_joints, obstacles=obstacles,
                                        attachments=attachments, self_collisions=self_collisions,
                                        disabled_collisions=self.client.get_self_collision_link_ids(robot), # get disabled self-collision links (srdf)
                                        extra_disabled_collisions=self.client.extra_disabled_collision_link_ids, # | option_extra_disabled_collisions
                                        custom_limits=pb_custom_limits)
        return collision_fn
