import time
from compas_fab.backends.interfaces import PlanMotion
from compas_fab.backends.interfaces import InverseKinematics
from compas_fab.robots import JointTrajectory, Duration, JointTrajectoryPoint

from pybullet_planning import is_connected, get_bodies, WorldSaver, joints_from_names, set_joint_positions, plan_joint_motion, check_initial_end
from pybullet_planning import get_custom_limits, get_joint_positions, get_sample_fn, get_extend_fn, get_distance_fn, MAX_DISTANCE
from pybullet_planning.motion_planners import birrt, lazy_prm
from pybullet_planning import wait_if_gui
from compas_fab_pychoreo.backend_features.pybullet_configuration_collision_checker import PybulletConfigurationCollisionChecker

# from compas_fab_pychoreo.conversions import
from compas_fab_pychoreo.utils import is_valid_option, values_as_list

class PybulletPlanMotion(PlanMotion):
    def __init__(self, client):
        self.client = client

    def plan_motion(self, end_configuration, start_configuration=None, group=None, options=None):
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

            - ``"base_link"``: (:obj:`str`) Name of the base link.
            - ``"ee_link"``: (:obj:`str`) Name of the end effector link.
            - ``"joint_names"``: (:obj:`list` of :obj:`str`) List containing joint names.
            - ``"joint_types"``: (:obj:`list` of :obj:`str`) List containing joint types.

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

              # TODO: JointConstraint

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.
        """
        robot_uid = self.client.robot_uid
        robot = self.client.compas_fab_robot

        ik_joint_names = robot.get_configurable_joint_names(group=group)
        ik_joint_types = robot.get_joint_types_by_names(ik_joint_names)
        ik_joints = joints_from_names(robot_uid, ik_joint_names)

        self_collisions = is_valid_option(options, 'self_collisions', True)
        diagnosis = is_valid_option(options, 'diagnosis', False)
        custom_limits = is_valid_option(options, 'custom_limits', {})
        resolutions = is_valid_option(options, 'resolutions', 0.1)
        weights = is_valid_option(options, 'weights', None)
        max_distance = is_valid_option(options, 'max_distance', MAX_DISTANCE)

        attachments = values_as_list(self.client.attachments)
        obstacles = values_as_list(self.client.collision_objects)

        disabled_collisions = self.client.self_collision_links
        # TODO additional disabled collisions in options
        # option_disabled_linke_names = is_valid_option(options, 'extra_disabled_collisions', [])
        # option_extra_disabled_collisions = get_body_body_disabled_collisions(robot_uid, workspace, extra_disabled_link_names)
        option_extra_disabled_collisions = set()
        extra_disabled_collisions = self.client.extra_disabled_collisions
        #| option_extra_disabled_collisions, TODO this cause incorrect result

        # TODO: worldsaver?
        # # TODO: compute joint weight as np.reciprocal(joint velocity bound) from URDF
        # JOINT_WEIGHTS = np.reciprocal([6.28318530718, 5.23598775598, 6.28318530718,
        #                        6.6497044501, 6.77187749774, 10.7337748998]) # sec / radian
        if start_configuration is not None:
            set_joint_positions(robot_uid, ik_joints, start_configuration.values)

        # TODO: if isinstance(goal_, JointConstraint):

        # conf_vals = plan_joint_motion(robot_uid, ik_joints, end_configuration.values,
        #                          obstacles=obstacles, attachments=attachments,
        #                          self_collisions=self_collisions, disabled_collisions=disabled_collisions,
        #                          extra_disabled_collisions=extra_disabled_collisions,
        #                          resolutions=resolutions, custom_limits=custom_limits,
        #                          diagnosis=diagnosis) #weights=weights,

        end_conf = end_configuration.values

        assert len(ik_joints) == len(end_conf)
        sample_fn = get_sample_fn(robot_uid, ik_joints, custom_limits=custom_limits)
        distance_fn = get_distance_fn(robot_uid, ik_joints, weights=weights)
        extend_fn = get_extend_fn(robot_uid, ik_joints, resolutions=resolutions)
        # collision_fn = get_collision_fn(robot_uid, ik_joints, obstacles=obstacles, attachments=attachments, self_collisions=self_collisions,
        #                                 disabled_collisions=disabled_collisions, extra_disabled_collisions=extra_disabled_collisions,
        #                                 custom_limits=custom_limits, max_distance=max_distance)
        collision_fn = PybulletConfigurationCollisionChecker(self.client)._get_collision_fn(group=group, options=options)

        start_conf = get_joint_positions(robot_uid, ik_joints)

        # if collision_fn(start_conf, diagnosis=True):
        #     print('initial pose colliding')
        #     wait_if_gui()

        if not check_initial_end(start_conf, end_conf, collision_fn, diagnosis=diagnosis):
            return None
        conf_vals = birrt(start_conf, end_conf, distance_fn, sample_fn, extend_fn, collision_fn)
        #return plan_lazy_prm(start_conf, end_conf, sample_fn, extend_fn, collision_fn)

        if conf_vals is not None and len(conf_vals) > 0:
            traj_pts = [JointTrajectoryPoint(values=conf_val, types=ik_joint_types, time_from_start=Duration(i*1,0)) \
                for i, conf_val in enumerate(conf_vals)]
            trajectory = JointTrajectory(trajectory_points=traj_pts, \
                joint_names=ik_joint_names, start_configuration=start_configuration, fraction=1.0)
            return trajectory
        else:
            return None
