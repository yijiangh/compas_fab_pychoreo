from copy import deepcopy

from compas_fab.robots import JointTrajectory, Robot
from .trajectory_smoother import TrajectorySmoother
from ..client import PyChoreoClient
from .pychoreo_configuration_collision_checker import PyChoreoConfigurationCollisionChecker
from .pychoreo_sweeping_collision_checker import PyChoreoSweepingCollisionChecker

class CustomTrajectorySmoother(TrajectorySmoother):
    def __init__(self, client: PyChoreoClient):
        self.client = client

    def smooth_trajectory(self, robot: Robot, trajectory: JointTrajectory, options=None):
        """Apply smoothing to a given trajectory.

        Parameters
        ----------
        robot : [type]
            [description]
        trajectory : [type]
            [description]
        options : dict, optional
            Dictionary containing the following key-value pairs:
            - "joint_resolutions": dict, {joint_name : joint_resolution}

        Returns
        -------
        triplet
            (success_bool, smoothed trajectory, message)
        """
        if trajectory is None or len(trajectory.points) == 0:
            return (False, None, 'Empty trajectory input.')

        diagnosis = options.get('diagnosis', False)
        joint_resolutions = options.get('joint_resolutions', {})
        smooth_iterations = options.get('smooth_iterations', 200)
        max_smooth_time = options.get('max_smooth_time', 60) # seconds
        check_sweeping_collision = options.get('check_sweeping_collision', False)

        # ? usage: check_collisions(self, robot, configuration, options)
        collision_checker = PyChoreoConfigurationCollisionChecker(self.client)
        # ? usage: check_collisions(self, robot, configuration_1, configuration_2, options)
        sweep_collision_checker = PyChoreoSweepingCollisionChecker(self.client)

        smoothed_trajectory = deepcopy(trajectory)

        # Victor's Laplacian smoothing as an example
        def smooth(trajectory, percentage):
            for i in range (1, len(trajectory.points)- 1):
                for j in range(9):
                    value = trajectory.points[i].values[j]
                    average = ( trajectory.points[i - 1].values[j] + trajectory.points[i + 1].values[j] ) / 2
                    trajectory.points[i].values[j] = value * (1-percentage) + average * percentage
            return trajectory

        for i in range(smooth_iterations):
            smoothed_trajectory = smooth(trajectory, smoothed_trajectory)

        if smoothed_trajectory:
            return (True, smoothed_trajectory, 'Smoothing succeeded')
        else:
            return (False, None, 'TBD smoothing fails.')

