"""
Internal implementation of the planner backend interface for pybullet_planning
"""
from compas_fab.backends.interfaces.client import PlannerInterface

from compas_fab_pychoreo.backend_features.pybullet_inverse_kinematics import PybulletInverseKinematics
from compas_fab_pychoreo.backend_features.pybullet_plan_cartesian_motion import PybulletPlanCartesianMotion
# from compas_fab_pychoreo.backend_features.pybullet_plan_motion import PybulletPlanMotion

class PybulletPlanner(PlannerInterface):
    """Implement the planner backend interface based on pybullet_planning
    """

    def __init__(self, client):
        super(PybulletPlanner, self).__init__(client)

    def forward_kinematics(self, *args, **kwargs):
        raise NotImplementedError()
    #     return MoveItForwardKinematics(self.client)(*args, **kwargs)

    def inverse_kinematics(self, *args, **kwargs):
        return PybulletInverseKinematics(self.client)(*args, **kwargs)

    def plan_cartesian_motion(self, *args, **kwargs):
        raise NotImplementedError()
    #     return PybulletPlanCartesianMotion(self.client)(*args, **kwargs)

    def plan_motion(self, *args, **kwargs):
        raise NotImplementedError()
    #     return PybulletPlanMotion(self.client)(*args, **kwargs)

    ###################################################

    # @forward_docstring(MoveItAddCollisionMesh)
    # def add_collision_mesh(self, *args, **kwargs):
    #     return MoveItAddCollisionMesh(self.client)(*args, **kwargs)

    # @forward_docstring(MoveItRemoveCollisionMesh)
    # def remove_collision_mesh(self, *args, **kwargs):
    #     return MoveItRemoveCollisionMesh(self.client)(*args, **kwargs)

    # @forward_docstring(MoveItAppendCollisionMesh)
    # def append_collision_mesh(self, *args, **kwargs):
    #     return MoveItAppendCollisionMesh(self.client)(*args, **kwargs)

    # @forward_docstring(MoveItAddAttachedCollisionMesh)
    # def add_attached_collision_mesh(self, *args, **kwargs):
    #     return MoveItAddAttachedCollisionMesh(self.client)(*args, **kwargs)

    # @forward_docstring(MoveItRemoveAttachedCollisionMesh)
    # def remove_attached_collision_mesh(self, *args, **kwargs):
    #     return MoveItRemoveAttachedCollisionMesh(self.client)(*args, **kwargs)
