import time
from termcolor import cprint
from compas_fab.backends.interfaces import PlanMotion
from compas_fab.robots import JointTrajectory, Duration, JointTrajectoryPoint

from compas_fab_pychoreo.backend_features.pychoreo_configuration_collision_checker import PyChoreoConfigurationCollisionChecker
from compas_fab_pychoreo.utils import is_valid_option

from pybullet_planning import WorldSaver, joints_from_names, check_initial_end
from pybullet_planning import get_custom_limits, get_joint_positions, get_sample_fn, get_extend_fn, get_distance_fn, MAX_DISTANCE, joint_from_name
from pybullet_planning import wait_if_gui

from pybullet_planning.motion_planners import solve_motion_plan
from pybullet_planning.motion_planners import INF
MOTION_PLANNING_ALGORITHMS = [
    "prm",
    "lazy_prm",
    "rrt_connect",
    "birrt",
    "rrt",
    "rrt_star",
    "lattice",
    # TODO: RRT in position/velocity space using spline interpolation
    # TODO: https://ompl.kavrakilab.org/planners.html
]

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
        robot_uid = self.client.get_robot_pybullet_uid(robot)
        verbose = is_valid_option(options, 'verbose', False)
        diagnosis = options.get('diagnosis', False)

        # * robot-related paramters
        custom_limits = options.get('custom_limits', {})
        resolutions = options.get('joint_resolutions', 0.1)
        weights = options.get('joint_weights', None)
        # TODO: auto compute joint weight
        algorithm = options.get('algorithm', 'birrt') # number of random restarts
        if algorithm not in MOTION_PLANNING_ALGORITHMS:
            cprint('{} algorithm not implemented, using birrt instead.'.format(algorithm), 'yellow')

        plan_options = {}
        max_time = options.get('max_time', INF) # total allowable planning time
        max_iterations = options.get('rrt_iterations', 20) # iterations of rrt explorations
        smooth_iterations = options.get('smooth_iterations', 20) # apply smoothing after finding a solution by default

        # * prm family paramaters
        plan_options['num_samples'] = options.get('num_samples', 100)
        # num_samples = options.get('num_samples', 100)
        # * rrt family paramaters
        if algorithm in ['birrt', 'rrt_connect']:
            plan_options['tree_frequency'] = options.get('tree_frequency', 1)
            # The frequency of adding tree nodes when extending a path from the current tree to the newly sampled config.
            # For example, if tree_freq=2, then a tree node is added every three nodes in the newly extended path, larger value means coarser extension, less nodes are added, by default 1. Only enlarge this if you think the planning is too slow,
            # and a path is easy to find.
        if algorithm == 'birrt':
            plan_options['restarts'] = options.get('rrt_restarts', 2) # number of random restarts
        if algorithm in ['rrt', 'rrt_star']:
            plan_options['goal_probability'] = options.get('goal_probability', 0.2) # goal biase probability for single-direction rrt

        # * convert link/joint names to pybullet indices
        joint_names = robot.get_configurable_joint_names(group=group)
        ik_joints = joints_from_names(robot_uid, joint_names)
        joint_types = robot.get_joint_types_by_names(joint_names)
        pb_custom_limits = get_custom_limits(robot_uid, ik_joints,
            custom_limits={joint_from_name(robot_uid, jn) : lims for jn, lims in custom_limits.items()})
        # print('pb custom limits: ', list(pb_custom_limits))

        with WorldSaver():
            if start_configuration is not None:
                # * set to start conf
                self.client.set_robot_configuration(robot, start_configuration)
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

            # path = birrt(start_conf, end_conf, distance_fn, sample_fn, extend_fn, collision_fn,
            #     restarts=rrt_restarts, iterations=rrt_iterations, tree_frequency=tree_frequency,
            #     smooth=smooth_iterations, )
            path = solve_motion_plan(start_conf, end_conf, distance_fn, sample_fn, extend_fn, collision_fn,
                algorithm=algorithm, max_time=max_time, max_iterations=max_iterations, smooth=smooth_iterations,
                **plan_options) #num_samples=num_samples,

        if path is None:
            # TODO use LOG
            if verbose:
                cprint('No free motion found!', 'red')
            return None
        else:
            jt_traj_pts = []
            for i, conf in enumerate(path):
                jt_traj_pt = JointTrajectoryPoint(joint_values=conf, joint_types=joint_types, time_from_start=Duration(i*1,0))
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
