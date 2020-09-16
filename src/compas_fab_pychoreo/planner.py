"""
Internal implementation of the planner backend interface for pybullet_planning
"""
from compas_fab.backends.interfaces.client import PlannerInterface

from compas_fab_pychoreo.backend_features.pybullet_inverse_kinematics import PybulletInverseKinematics
from compas_fab_pychoreo.backend_features.pybullet_configuration_collision_checker import PybulletConfigurationCollisionChecker
from compas_fab_pychoreo.backend_features.pybullet_plan_cartesian_motion import PybulletPlanCartesianMotion
from compas_fab_pychoreo.backend_features.pybullet_plan_motion import PybulletPlanMotion

class PybulletPlanner(PlannerInterface):
    """Implement the planner backend interface based on pybullet_planning
    """

    def __init__(self, client):
        super(PybulletPlanner, self).__init__(client)
        # move_group_name : inverse_kinenamtics function
        self.ik_fn_from_group = {}

    def forward_kinematics(self, *args, **kwargs):
        raise NotImplementedError()
    #     return MoveItForwardKinematics(self.client)(*args, **kwargs)

    def inverse_kinematics(self, *args, **kwargs):
        return PybulletInverseKinematics(self.client)(*args, **kwargs)

    def plan_cartesian_motion(self, *args, **kwargs):
        return PybulletPlanCartesianMotion(self.client)(*args, **kwargs)

    def plan_motion(self, *args, **kwargs):
        return PybulletPlanMotion(self.client)(*args, **kwargs)

    ###################################################

    def configuration_in_collision(self, *args, **kwargs):
        return PybulletConfigurationCollisionChecker(self.client)(*args, **kwargs)

    ###################################################

    def add_collision_mesh(self, *args, **kwargs):
        return self.client.add_collision_mesh(*args, **kwargs)

    def remove_collision_mesh(self, *args, **kwargs):
        return self.client.remove_collision_mesh(*args, **kwargs)

    def append_collision_mesh(self, *args, **kwargs):
        return self.client.append_collision_mesh(*args, **kwargs)

    def add_attached_collision_mesh(self, *args, **kwargs):
        return self.client.add_attached_collision_mesh(*args, **kwargs)

    def remove_attached_collision_mesh(self, *args, **kwargs):
        return self.client.remove_attached_collision_mesh(*args, **kwargs)
