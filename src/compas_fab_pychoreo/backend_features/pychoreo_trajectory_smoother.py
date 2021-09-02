from termcolor import cprint
import pybullet_planning as pp

from compas_fab.robots import JointTrajectory, Duration, JointTrajectoryPoint, Configuration
from compas_fab_pychoreo.backend_features.trajectory_smoother import TrajectorySmoother
from compas_fab_pychoreo.backend_features.pychoreo_configuration_collision_checker import PyChoreoConfigurationCollisionChecker

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
        verbose = options.get('verbose', False)
        diagnosis = options.get('diagnosis', False)
        custom_limits = options.get('custom_limits', {})
        weights = options.get('joint_weights', None)
        resolutions = options.get('joint_resolutions', 0.1)
        # ! max_time is prioritized over iterations
        smooth_iterations = options.get('smooth_iterations', 100)
        max_smooth_time = options.get('max_smooth_time', 120) # seconds

        if trajectory is None or len(trajectory.points) == 0:
            return (False, None, 'Empty trajectory input.')

        robot_uid = self.client.get_robot_pybullet_uid(robot)
        # * convert link/joint names to pybullet indices
        joint_names = trajectory.joint_names or trajectory.points[0].joint_names
        joint_types = robot.get_joint_types_by_names(joint_names)
        pb_joints = pp.joints_from_names(robot_uid, joint_names)
        # pb_custom_limits = pp.get_custom_limits(robot_uid, pb_joints,
        #     custom_limits={pp.joint_from_name(robot_uid, jn) : lims for jn, lims in custom_limits.items()})

        path = [conf.joint_values for conf in trajectory.points]
        with pp.WorldSaver():
            distance_fn = pp.get_distance_fn(robot_uid, pb_joints, weights=weights)
            extend_fn = pp.get_extend_fn(robot_uid, pb_joints, resolutions=resolutions)
            options['robot'] = robot
            collision_fn = PyChoreoConfigurationCollisionChecker(self.client)._get_collision_fn(robot, joint_names, options=options)

            smoothed_path = pp.smooth_path(path, extend_fn, collision_fn, distance_fn=distance_fn, \
                iterations=smooth_iterations, max_time=max_smooth_time, verbose=verbose)

        if smoothed_path:
            jt_traj_pts = []
            for i, conf in enumerate(path):
                jt_traj_pt = JointTrajectoryPoint(joint_values=conf, joint_types=joint_types, time_from_start=Duration(i*1,0))
                jt_traj_pt.joint_names = joint_names
                jt_traj_pts.append(jt_traj_pt)
            smoothed_trajectory = JointTrajectory(trajectory_points=jt_traj_pts,
                joint_names=joint_names, start_configuration=jt_traj_pts[0], fraction=1.0)
            return (True, smoothed_trajectory, 'Smoothing succeeded!')
        else:
            return (False, None, 'TBD smoothing fails.')
