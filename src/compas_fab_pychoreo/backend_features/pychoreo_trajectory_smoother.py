from termcolor import cprint
import pybullet_planning as pp

from compas_fab.robots import JointTrajectory, Duration, JointTrajectoryPoint, Configuration
from .trajectory_smoother import TrajectorySmoother
from .pychoreo_configuration_collision_checker import PyChoreoConfigurationCollisionChecker
from .pychoreo_sweeping_collision_checker import PyChoreoSweepingCollisionChecker

class PyChoreoTrajectorySmoother(TrajectorySmoother):
    def __init__(self, client):
        self.client = client

    def smooth_trajectory(self, robot, trajectory, options=None):
        """Apply smoothing to a given trajectory.

        Parameters
        ----------
        robot : [type]
            [description]
        trajectory : [type]
            [description]
        options : [type], optional
            [description], by default None

        Returns
        -------
        triplet
            (success_bool, smoothed trajectory, message)
        """
        # verbose = options.get('verbose', False)
        diagnosis = options.get('diagnosis', False)
        joint_weights = options.get('joint_weights', {})
        joint_resolutions = options.get('joint_resolutions', {})
        # ! max_time is prioritized over iterations
        smooth_iterations = options.get('smooth_iterations', 200)
        max_smooth_time = options.get('max_smooth_time', 60) # seconds
        check_sweeping_collision = options.get('check_sweeping_collision', False)
        coarse_waypoints = options.get('coarse_waypoints', False) # smoothing uses refined waypoints by default

        if trajectory is None or len(trajectory.points) == 0:
            return (False, None, 'Empty trajectory input.')

        robot_uid = self.client.get_robot_pybullet_uid(robot)
        # * convert link/joint names to pybullet indices
        # ! we assume all joint names are the same across the entire trajectory
        joint_names = trajectory.joint_names or trajectory.points[0].joint_names
        joint_types = robot.get_joint_types_by_names(joint_names)
        pb_joints = pp.joints_from_names(robot_uid, joint_names)
        # ! currently pp's resolutions and weights only support array not dict
        pb_joint_resolutions = None if len(joint_resolutions) == 0 else \
            [joint_resolutions[joint_name] for joint_name in joint_names]
        pb_joint_weights = None if len(joint_weights) == 0 else \
            [joint_weights[joint_name] for joint_name in joint_names]

        sweep_collision_fn = PyChoreoSweepingCollisionChecker(self.client)._get_sweeping_collision_fn(robot, joint_names, options=options) if check_sweeping_collision else None

        path = [conf.joint_values for conf in trajectory.points]
        with pp.WorldSaver():
            distance_fn = pp.get_distance_fn(robot_uid, pb_joints, weights=pb_joint_weights)
            extend_fn = pp.get_extend_fn(robot_uid, pb_joints, resolutions=pb_joint_resolutions, norm=pp.INF)
            collision_fn = PyChoreoConfigurationCollisionChecker(self.client)._get_collision_fn(robot, joint_names, options=options)

            smoothed_path = pp.smooth_path(path, extend_fn, collision_fn, distance_fn=distance_fn, \
                max_smooth_iterations=smooth_iterations, max_time=max_smooth_time, verbose=False,
                sweep_collision_fn=sweep_collision_fn, coarse_waypoints=coarse_waypoints)

        if smoothed_path:
            old_cost = pp.compute_path_cost(path, cost_fn=distance_fn)
            new_cost = pp.compute_path_cost(smoothed_path, cost_fn=distance_fn)
            jt_traj_pts = []
            for i, conf in enumerate(smoothed_path):
                jt_traj_pt = JointTrajectoryPoint(joint_values=conf, joint_types=joint_types, time_from_start=Duration(i*1,0))
                jt_traj_pt.joint_names = joint_names
                jt_traj_pts.append(jt_traj_pt)
            smoothed_trajectory = JointTrajectory(trajectory_points=jt_traj_pts,
                joint_names=joint_names, start_configuration=jt_traj_pts[0], fraction=1.0)
            return (True, smoothed_trajectory, 'Smoothing succeeded | before smoothing total path distance {:.3f}, after {:.3f}'.format(
                old_cost, new_cost))
        else:
            return (False, None, 'TBD smoothing fails.')
