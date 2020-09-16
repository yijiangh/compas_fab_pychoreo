import time
from termcolor import cprint
from compas_fab.robots import Configuration, JointTrajectory, JointTrajectoryPoint, Duration
from compas_fab.backends.interfaces import PlanCartesianMotion
from compas_fab.backends.interfaces import InverseKinematics

from compas_fab_pychoreo.conversions import pose_from_frame
from compas_fab_pychoreo.utils import is_valid_option, values_as_list
from compas_fab_pychoreo.backend_features.pybullet_configuration_collision_checker import PybulletConfigurationCollisionChecker
from pybullet_planning import is_connected, get_bodies, WorldSaver, get_collision_fn, joints_from_names, plan_cartesian_motion, \
    link_from_name, interpolate_poses, get_link_pose, get_sample_fn, set_joint_positions, elapsed_time

class PybulletPlanCartesianMotion(PlanCartesianMotion):
    def __init__(self, client):
        self.client = client

    def plan_cartesian_motion(self, frames_WCF, start_configuration=None, group=None, options=None):
        """Calculates a cartesian motion path (linear in tool space).

        Parameters
        ----------
        frames_WCF: list of :class:`compas.geometry.Frame`
            The frames through which the path is defined.
        start_configuration: :class:`Configuration`, optional
            The robot's full configuration, i.e. values for all configurable
            joints of the entire robot, at the starting position.
        group: str, optional
            The planning group used for calculation.
        options: dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

            - ``"diagnosis"``: (:obj:`bool`, optional)
              . Defaults to ``False``.

            - ``"avoid_collisions"``: (:obj:`bool`, optional)
              Whether or not to avoid collisions. Defaults to ``True``.

            - ``"max_step"``: (:obj:`float`, optional) The approximate distance between the
              calculated points. (Defined in the robot's units.) Defaults to ``0.01``.
            - ``"jump_threshold"``: (:obj:`float`, optional)
              The maximum allowed distance of joint positions between consecutive
              points. If the distance is found to be above this threshold, the
              path computation fails. It must be specified in relation to max_step.
              If this threshold is ``0``, 'jumps' might occur, resulting in an invalid
              cartesian path. Defaults to :math:`\\pi / 2`.

              # TODO: jump_threshold
              # TODO: JointConstraint

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.
        """
        robot_uid = self.client.robot_uid
        robot = self.client.compas_fab_robot

        # * parse options
        diagnosis = is_valid_option(options, 'diagnosis', False)
        avoid_collisions = is_valid_option(options, 'avoid_collisions', True)
        pos_step_size = is_valid_option(options, 'max_step', 0.01)

        # * convert link/joint names to pybullet indices
        base_link_name = robot.get_base_link_name(group=group)
        tool_link_name = robot.get_end_effector_link_name(group=group)
        tool_link = link_from_name(robot_uid, tool_link_name)
        base_link = link_from_name(robot_uid, base_link_name)

        joint_names = robot.get_configurable_joint_names(group=group)
        ik_joints = joints_from_names(robot_uid, joint_names)
        joint_types = robot.get_joint_types_by_names(joint_names)

        # * workspace linear interpolation
        given_poses = [pose_from_frame(frame_WCF) for frame_WCF in frames_WCF]
        ee_poses = []
        for p1, p2 in zip(given_poses[:-1], given_poses[1:]):
            c_interp_poses = list(interpolate_poses(p1, p2, pos_step_size=pos_step_size))
            ee_poses.extend(c_interp_poses)

        # * build collision fn here
        attachments = values_as_list(self.client.attachments)
        collision_fn = PybulletConfigurationCollisionChecker(self.client)._get_collision_fn(group=group, options=options)

        with WorldSaver():
            # set to start conf
            if start_configuration is not None:
                start_conf_vals = start_configuration.values
                set_joint_positions(robot_uid, ik_joints, start_conf_vals)

            path = plan_cartesian_motion(self.client.robot_uid, base_link, tool_link, ee_poses)
            # path, cost = plan_cartesian_motion_lg(robot, ik_joints, ee_poses, sample_ik_fn, collision_fn, \
            #     custom_vel_limits=vel_limits, ee_vel=ee_vel)

            if path is not None and avoid_collisions:
                for conf_val in path:
                    for attachment in attachments:
                        attachment.assign()
                    if collision_fn(conf_val, diagnosis=diagnosis):
                        path = None
                        break

        if path is None:
            cprint('No Cartesian motion found!', 'red')
            return None
        else:
            jt_traj_pts = []
            for i, conf in enumerate(path):
                c_conf = Configuration(values=conf, types=joint_types, joint_names=joint_names)
                jt_traj_pt = JointTrajectoryPoint(values=c_conf.values, types=c_conf.types, time_from_start=Duration(i*1,0))
                jt_traj_pts.append(jt_traj_pt)
            trajectory = JointTrajectory(trajectory_points=jt_traj_pts,
                joint_names=joint_names, start_configuration=start_configuration, fraction=1.0)
            return trajectory

