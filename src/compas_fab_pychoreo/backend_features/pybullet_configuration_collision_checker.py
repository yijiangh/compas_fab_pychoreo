from compas_fab_pychoreo.backend_features.configuration_collision_checker import ConfigurationCollisionChecker
from compas_fab_pychoreo.utils import is_valid_option

from pybullet_planning import set_joint_positions, get_link_pose, get_custom_limits, joints_from_names, link_from_name, \
    get_collision_fn, get_disabled_collisions

class PybulletConfigurationCollisionChecker(ConfigurationCollisionChecker):
    def __init__(self, client):
        self.client = client

    def configuration_in_collision(self, configuration, group=None, options=None, diagnosis=False):
        """[summary]

        Parameters
        ----------
        configuration: :class:`compas_fab.robots.Configuration`
        group: str, optional
        options : dict, optional
            Dictionary containing the following key-value pairs:
            - "self_collisions": bool, set to True if checking self collisions of the robot, defaults to True
            - ``"attached_collision_meshes"``: (:obj:`list` of :class:`compas_fab.robots.AttachedCollisionMesh`)
              Defaults to ``None``.

        Returns
        -------
        is_collision : bool
            True if in collision, False otherwise
        """
        robot_uid = self.client.robot_uid
        robot = self.client.compas_fab_robot

        ik_joint_names = robot.get_configurable_joint_names(group=group)
        ik_joints = joints_from_names(robot_uid, ik_joint_names)
        # tool_link_name = robot.get_end_effector_link_name(group=group)
        # tool_link = link_from_name(robot_uid, tool_link_name)

        # get disabled self-collision links
        self_collisions = is_valid_option(options, 'self_collisions', True)
        # https://github.com/yijiangh/pybullet_planning/blob/dev/tests/test_collisions.py#L76
        self_collision_disabled_link_names = robot.disabled_collisions
        self_collision_links = get_disabled_collisions(robot, self_collision_disabled_link_names)

        # attachment
        attached_collision_meshes = is_valid_option(options, 'attached_collision_meshes', [])
        attachments = []
        for acm in attached_collision_meshes:
            attachments.append(self.client.add_attached_collision_mesh(acm))

        collision_fn = get_collision_fn(self.client.robot_uid, ik_joints, obstacles=[],
                                               attachments=attachments, self_collisions=self_collisions,
                                            #    disabled_collisions=disabled_collisions,
                                            #    extra_disabled_collisions=extra_disabled_collisions,
                                               custom_limits={})

        return collision_fn(configuration.values, diagnosis=diagnosis)
