from compas.robots.model import robot
import numpy as np
from collections import defaultdict
from itertools import product, combinations

from ..utils import LOGGER, align_configurations
from ..client import PyChoreoClient
from .feature_base import SweepingCollisionChecker
from .pychoreo_configuration_collision_checker import PyChoreoConfigurationCollisionChecker

import pybullet_planning as pp
from pybullet_planning import joints_from_names

class PyChoreoDenseSampleSweepingCollisionChecker(SweepingCollisionChecker):
    def __init__(self, client: PyChoreoClient):
        self.client = client

    def check_sweeping_collisions(self, robot, configuration_1=None, configuration_2=None, options=None):
        """Approximate continuous collision checking between two robot configurations by densely interpolate between the
        configurations by a given division number.

        Parameters
        ----------
        configuration: :class:`compas_fab.robots.Configuration`
        group: str, optional
        options : dict, optional
            Dictionary containing the following key-value pairs:
            - "dense_sample_sweeping_check_num_steps": int, number of points used for interpolating between the two configurations.
            - "collision_fn": function handle for the collision_fn, this is only for reusing the same
            collision_fn to avoid some computation overhead. 
            Default to None, meaning that a new collision_fn is constructed every time the function is called.

        Returns
        -------
        is_collision : bool
            True if in collision, False otherwise
        """
        options = options or {}
        _conf1, _conf2 = align_configurations(configuration_1, configuration_2)
        diagnosis = options.get('diagnosis', False)
        num_steps = options.get('dense_sample_sweeping_check_num_steps', 5)

        robot_uid = self.client.get_robot_pybullet_uid(robot)
        joint_names = _conf1.joint_names
        joints = joints_from_names(robot_uid, joint_names)
        refine_fn = pp.get_refine_fn(robot_uid, joints, num_steps=num_steps)

        collision_fn = options.get('collision_fn', PyChoreoConfigurationCollisionChecker(self.client)._get_collision_fn(robot, joint_names, options=options))
        path = pp.direct_path(_conf1.joint_values, _conf2.joint_values, refine_fn, collision_fn, \
            diagnosis=diagnosis, sweep_collision_fn=None)
        return path is None