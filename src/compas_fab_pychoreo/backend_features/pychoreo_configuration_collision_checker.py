from compas_fab_pychoreo.backend_features.configuration_collision_checker import ConfigurationCollisionChecker
from compas_fab_pychoreo.utils import is_valid_option, values_as_list

from pybullet_planning import set_joint_positions, get_link_pose, get_custom_limits, joints_from_names, link_from_name, \
    get_collision_fn, get_disabled_collisions, WorldSaver
from pybullet_planning import wait_if_gui, get_body_name, RED, BLUE, set_color

class PychoreoConfigurationCollisionChecker(ConfigurationCollisionChecker):
    def __init__(self, client):
        self.client = client

    def configuration_in_collision(self, configuration, group=None, options=None):
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
        assert 'robot' in options, 'a robot model must be specified!'
        diagnosis = is_valid_option(options, 'diagnosis', False)
        collision_fn = self._get_collision_fn(group, options)
        return collision_fn(configuration.values, diagnosis=diagnosis)

    def _get_collision_fn(self, group=None, options=None):
        """Returns a `pybullet_planning` collision_fn
        """
        robot = options['robot']
        robot_uid = robot.attributes['pybullet_uid']

        ik_joint_names = robot.get_configurable_joint_names(group=group)
        ik_joints = joints_from_names(robot_uid, ik_joint_names)

        # get disabled self-collision links (srdf)
        self_collisions = is_valid_option(options, 'self_collisions', True)
        obstacles = values_as_list(self.client.collision_objects)
        # ConstraintInfo = namedtuple('ConstraintInfo', ['constraint_id', 'body_id', 'robot_uid'])
        attachments = values_as_list(self.client.pychoreo_attachments)

        print('disabled_self_collision_links: ', self.client.get_self_collision_link_ids(robot))

        # TODO additional disabled collisions in options
        # option_disabled_linke_names = is_valid_option(options, 'extra_disabled_collisions', [])
        # option_extra_disabled_collisions = get_body_body_disabled_collisions(robot_uid, workspace, extra_disabled_link_names)
        option_extra_disabled_collisions = set()

        collision_fn = get_collision_fn(robot_uid, ik_joints, obstacles=obstacles,
                                        attachments=attachments, self_collisions=self_collisions,
                                        disabled_collisions=self.client.get_self_collision_link_ids(robot),
                                        extra_disabled_collisions=self.client.extra_disabled_collision_link_ids, # | option_extra_disabled_collisions
                                        custom_limits={})
        return collision_fn
