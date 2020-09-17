import time
from compas_fab.backends.interfaces import PlanMotion
from compas_fab.backends.interfaces import InverseKinematics
from compas_fab.robots import JointTrajectory, Duration, JointTrajectoryPoint, Configuration

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

        # * parse options
        diagnosis = is_valid_option(options, 'diagnosis', False)
        custom_limits = is_valid_option(options, 'custom_limits', {})
        resolutions = is_valid_option(options, 'resolutions', 0.1)
        weights = is_valid_option(options, 'weights', None)
        # # TODO: compute default joint weight as np.reciprocal(joint velocity bound) from URDF
        # JOINT_WEIGHTS = np.reciprocal([6.28318530718, 5.23598775598, 6.28318530718,
        #                        6.6497044501, 6.77187749774, 10.7337748998]) # sec / radian

        # * convert link/joint names to pybullet indices
        joint_names = robot.get_configurable_joint_names(group=group)
        ik_joints = joints_from_names(robot_uid, joint_names)
        joint_types = robot.get_joint_types_by_names(joint_names)

        with WorldSaver():
            # set to start conf
            if start_configuration is not None:
                start_conf_vals = start_configuration.values
                set_joint_positions(robot_uid, ik_joints, start_conf_vals)

            sample_fn = get_sample_fn(robot_uid, ik_joints, custom_limits=custom_limits)
            distance_fn = get_distance_fn(robot_uid, ik_joints, weights=weights)
            extend_fn = get_extend_fn(robot_uid, ik_joints, resolutions=resolutions)
            collision_fn = PybulletConfigurationCollisionChecker(self.client)._get_collision_fn(group=group, options=options)

            start_conf = get_joint_positions(robot_uid, ik_joints)
            end_conf = end_configuration.values
            assert len(ik_joints) == len(end_conf)

            if not check_initial_end(start_conf, end_conf, collision_fn, diagnosis=diagnosis):
                return None
            path = birrt(start_conf, end_conf, distance_fn, sample_fn, extend_fn, collision_fn)
            #return plan_lazy_prm(start_conf, end_conf, sample_fn, extend_fn, collision_fn)

        if path is None:
            print('No free motion found!')
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
