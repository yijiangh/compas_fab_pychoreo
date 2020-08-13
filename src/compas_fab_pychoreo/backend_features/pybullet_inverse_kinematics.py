from compas_fab.backends.interfaces import InverseKinematics
from compas_fab.robots import Configuration

from pybullet_planning import all_between, is_pose_close
from pybullet_planning import inverse_kinematics_helper
from pybullet_planning import get_movable_joints, set_joint_positions, get_link_pose, get_custom_limits, joints_from_names, link_from_name, \
    get_sample_fn
from pybullet_planning import wait_if_gui

from compas_fab_pychoreo.conversions import pose_from_frame

class PybulletInverseKinematics(InverseKinematics):
    def __init__(self, client):
        self.client = client

    def inverse_kinematics(self, frame_WCF, start_configuration=None, group=None, options=None):
        """Calculate the robot's inverse kinematic for a given frame.
        Parameters
        ----------
        frame_WCF: :class:`compas.geometry.Frame`
            The frame to calculate the inverse for.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
            If passed, the inverse will be calculated such that the calculated
            joint positions differ the least from the start_configuration.
            Defaults to the init configuration.
        group: str, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        options: dict, optional
            Dictionary containing the following key-value pairs:
            - ``"base_link"``: (:obj:`str`) Name of the base link.
            - ``"avoid_collisions"``: (:obj:`bool`, optional) Whether or not to avoid collisions.
              Defaults to `True`.
            - ``"constraints"``: (:obj:`list` of :class:`compas_fab.robots.Constraint`, optional)
              A set of constraints that the request must obey. Defaults to `None`.
            - ``"attempts"``: (:obj:`int`, optional) The maximum number of inverse kinematic attempts.
              Defaults to `8`.
            - ``"attached_collision_meshes"``: (:obj:`list` of :class:`compas_fab.robots.AttachedCollisionMesh`, optional)
              Defaults to `None`.
        Raises
        ------
        compas_fab.backends.exceptions.BackendError
            If no configuration can be found.
        Returns
        -------
        :class:`compas_fab.robots.Configuration`
            The planning group's configuration.
        """
        max_iterations = 8 if options is None or 'attempts' not in options else options['attempts']
        robot_uid = self.client.robot_uid
        robot = self.client.compas_fab_robot

        # movable_joints = get_movable_joints(robot_uid)
        ik_joint_names = robot.get_configurable_joint_names(group=group)
        ik_joints = joints_from_names(robot_uid, ik_joint_names)
        tool_link_name = robot.get_end_effector_link_name(group=group)
        tool_link = link_from_name(robot_uid, tool_link_name)

        sample_fn = get_sample_fn(robot_uid, ik_joints)

        target_pose = pose_from_frame(frame_WCF)
        start_conf_vals = start_configuration.values if start_configuration is not None else sample_fn()
        set_joint_positions(robot_uid, ik_joints, start_conf_vals)

        for _ in range(max_iterations):
            # TODO: stop is no progress
            # TODO: stop if collision or invalid joint limits
            kinematic_conf = inverse_kinematics_helper(robot_uid, tool_link, target_pose)
            if kinematic_conf is None:
                return None
            set_joint_positions(robot_uid, ik_joints, kinematic_conf)
            if is_pose_close(get_link_pose(robot_uid, tool_link), target_pose): #, **kwargs):
                break
        else:
            return None
        # TODO custom_limits
        # lower_limits, upper_limits = get_custom_limits(robot_uid, ik_joints, custom_limits)
        # if not all_between(lower_limits, kinematic_conf, upper_limits):
        #     return None

        joint_types = robot.get_joint_types_by_names(ik_joint_names)
        return Configuration(values=kinematic_conf, types=joint_types, joint_names=ik_joint_names)
