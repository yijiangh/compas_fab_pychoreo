"""
Internal implementation of the planner backend interface for pybullet_planning
"""
from compas_fab.backends.interfaces.client import PlannerInterface

from .backend_features.pychoreo_inverse_kinematics import PyChoreoInverseKinematics
from .backend_features.pychoreo_plan_cartesian_motion import PyChoreoPlanCartesianMotion
from .backend_features.pychoreo_plan_motion import PyChoreoPlanMotion
from .backend_features.pychoreo_add_attached_collision_mesh import PyChoreoAddAttachedCollisionMesh
from .backend_features.pychoreo_remove_attached_collision_mesh import PyChoreoRemoveAttachedCollisionMesh
from .backend_features.pychoreo_add_collision_mesh import PyChoreoAddCollisionMesh
from .backend_features.pychoreo_remove_collision_mesh import PyBulletRemoveCollisionMesh
from .backend_features.pychoreo_detach_attached_collision_mesh import PyChoreoDetachAttachedCollisionMesh

from .backend_features.pychoreo_configuration_collision_checker import PyChoreoConfigurationCollisionChecker
from .backend_features.pychoreo_sweeping_collision_checker import PyChoreoSweepingCollisionChecker

from compas_fab.backends.pybullet.backend_features.pybullet_forward_kinematics import PyBulletForwardKinematics
from compas_fab.backends.pybullet.backend_features.pybullet_append_collision_mesh import PyBulletAppendCollisionMesh

class PyChoreoPlanner(PlannerInterface):
    """Implement the planner backend interface based on pybullet_planning
    """

    def __init__(self, client):
        super(PyChoreoPlanner, self).__init__(client)

    def inverse_kinematics(self, *args, **kwargs):
        return PyChoreoInverseKinematics(self.client)(*args, **kwargs)

    def plan_cartesian_motion(self, *args, **kwargs):
        return PyChoreoPlanCartesianMotion(self.client)(*args, **kwargs)

    def plan_motion(self, *args, **kwargs):
        return PyChoreoPlanMotion(self.client)(*args, **kwargs)

    ###################################################

    def check_collisions(self, *args, **kwargs):
        return PyChoreoConfigurationCollisionChecker(self.client)(*args, **kwargs)

    def check_sweeping_collisions(self, *args, **kwargs):
        return PyChoreoSweepingCollisionChecker(self.client)(*args, **kwargs)

    ###################################################
    # ! using main-branch

    def forward_kinematics(self, *args, **kwargs):
        return PyBulletForwardKinematics(self.client)(*args, **kwargs)

    def append_collision_mesh(self, *args, **kwargs):
        return PyBulletAppendCollisionMesh(self.client)(*args, **kwargs)

    #####

    def add_collision_mesh(self, *args, **kwargs):
        return PyChoreoAddCollisionMesh(self.client)(*args, **kwargs)

    def remove_collision_mesh(self, *args, **kwargs):
        return PyBulletRemoveCollisionMesh(self.client)(*args, **kwargs)

    def add_attached_collision_mesh(self, *args, **kwargs):
        return PyChoreoAddAttachedCollisionMesh(self.client)(*args, **kwargs)

    def remove_attached_collision_mesh(self, *args, **kwargs):
        return PyChoreoRemoveAttachedCollisionMesh(self.client)(*args, **kwargs)

    def detach_attached_collision_mesh(self, *args, **kwargs):
        return PyChoreoDetachAttachedCollisionMesh(self.client)(*args, **kwargs)
