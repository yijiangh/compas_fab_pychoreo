import time
from compas_fab.backends.interfaces import PlanMotion
from compas_fab.backends.interfaces import InverseKinematics
from compas_fab.robots import JointTrajectory, Duration, JointTrajectoryPoint, Configuration

from pybullet_planning import is_connected, get_bodies, WorldSaver, joints_from_names, set_joint_positions, plan_joint_motion, check_initial_end
from pybullet_planning import get_custom_limits, get_joint_positions, get_sample_fn, get_extend_fn, get_distance_fn, MAX_DISTANCE, joint_from_name
from pybullet_planning.motion_planners import birrt, lazy_prm
from pybullet_planning import wait_if_gui
from compas_fab_pychoreo.backend_features.pychoreo_configuration_collision_checker import PyChoreoConfigurationCollisionChecker

# from compas_fab_pychoreo.conversions import
from compas_fab_pychoreo.utils import is_valid_option, values_as_list

class PyChoreoPlanMotion(PlanMotion):
    def __init__(self, client):
        self.client = client

    def plan_motion(self, robot, goal_constraints, start_configuration=None, group=None, options=None):
        """Calculates a motion path.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which the motion path is being calculated.
        goal_constraints: list of :class:`compas_fab.robots.Constraint`
            The goal to be achieved, defined in a set of constraints.
            Constraints can be very specific, for example defining value domains
            for each joint, such that the goal configuration is included,
            or defining a volume in space, to which a specific robot link (e.g.
            the end-effector) is required to move to.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
            The robot's full configuration, i.e. values for all configurable
            joints of the entire robot, at the starting position.
        group: str, optional
            The name of the group to plan for.
        options : dict, optional
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

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.
        """
        robot_uid = robot.attributes['pybullet_uid']

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
        pb_custom_limits = get_custom_limits(robot_uid, ik_joints,
            custom_limits={joint_from_name(robot_uid, jn) : lims for jn, lims in custom_limits.items()})

        with WorldSaver():
            # set to start conf
            if start_configuration is not None:
                start_conf_vals = start_configuration.values
                set_joint_positions(robot_uid, ik_joints, start_conf_vals)
            sample_fn = get_sample_fn(robot_uid, ik_joints, custom_limits=pb_custom_limits)
            distance_fn = get_distance_fn(robot_uid, ik_joints, weights=weights)
            extend_fn = get_extend_fn(robot_uid, ik_joints, resolutions=resolutions)
            options['robot'] = robot
            collision_fn = PyChoreoConfigurationCollisionChecker(self.client)._get_collision_fn(robot, joint_names, options=options)

            start_conf = get_joint_positions(robot_uid, ik_joints)
            end_conf = self._joint_values_from_joint_constraints(joint_names, goal_constraints)
            assert len(ik_joints) == len(end_conf)

            if not check_initial_end(start_conf, end_conf, collision_fn, diagnosis=diagnosis):
                return None
            path = birrt(start_conf, end_conf, distance_fn, sample_fn, extend_fn, collision_fn)
            #return plan_lazy_prm(start_conf, end_conf, sample_fn, extend_fn, collision_fn)

        if path is None:
            # TODO use LOG
            print('No free motion found!')
            return None
        else:
            jt_traj_pts = []
            for i, conf in enumerate(path):
                # c_conf = Configuration(values=conf, types=joint_types, joint_names=joint_names)
                jt_traj_pt = JointTrajectoryPoint(values=conf, types=joint_types, time_from_start=Duration(i*1,0))
                # TODO why don't we have a `joint_names` input for JointTrajectoryPoint?
                # https://github.com/compas-dev/compas_fab/blob/master/src/compas_fab/robots/trajectory.py#L64
                jt_traj_pt.joint_names = joint_names
                jt_traj_pts.append(jt_traj_pt)
            trajectory = JointTrajectory(trajectory_points=jt_traj_pts,
                joint_names=joint_names, start_configuration=jt_traj_pts[0], fraction=1.0)
            return trajectory

    def _joint_values_from_joint_constraints(self, joint_names, constraints):
        # extract joint values from constraints using joint_names
        joint_vals = []
        for name in joint_names:
            for constr in constraints:
                if constr.joint_name == name and constr.type == constr.JOINT:
                    joint_vals.append(constr.value)
                    break
            else:
                assert False, 'Joint name {} not found in constraints {}'.format(name, constraints)
        return joint_vals
