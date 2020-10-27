"""
Internal implementation of the planner backend interface for pybullet_planning
"""
from compas_fab.backends.interfaces.client import PlannerInterface

from compas_fab_pychoreo.backend_features.pychoreo_inverse_kinematics import PychoreoInverseKinematics
from compas_fab_pychoreo.backend_features.pychoreo_configuration_collision_checker import PychoreoConfigurationCollisionChecker
from compas_fab_pychoreo.backend_features.pychoreo_plan_cartesian_motion import PychoreoPlanCartesianMotion
from compas_fab_pychoreo.backend_features.pychoreo_plan_motion import PychoreoPlanMotion

from compas_fab.backends.pybullet.backend_features.pybullet_forward_kinematics import PyBulletForwardKinematics
from compas_fab.backends.pybullet.backend_features.pybullet_add_attached_collision_mesh import PyBulletAddAttachedCollisionMesh
from compas_fab.backends.pybullet.backend_features.pybullet_add_collision_mesh import PyBulletAddCollisionMesh
from compas_fab.backends.pybullet.backend_features.pybullet_append_collision_mesh import PyBulletAppendCollisionMesh
from compas_fab.backends.pybullet.backend_features.pybullet_remove_attached_collision_mesh import PyBulletRemoveAttachedCollisionMesh
from compas_fab.backends.pybullet.backend_features.pybullet_remove_collision_mesh import PyBulletRemoveCollisionMesh

class PychoreoPlanner(PlannerInterface):
    """Implement the planner backend interface based on pybullet_planning
    """

    def __init__(self, client):
        super(PychoreoPlanner, self).__init__(client)

    def forward_kinematics(self, *args, **kwargs):
        return PyBulletForwardKinematics(self.client)(*args, **kwargs)

    def inverse_kinematics(self, *args, **kwargs):
        return PychoreoInverseKinematics(self.client)(*args, **kwargs)

    def plan_cartesian_motion(self, *args, **kwargs):
        return PychoreoPlanCartesianMotion(self.client)(*args, **kwargs)

    def plan_motion(self, *args, **kwargs):
        return PychoreoPlanMotion(self.client)(*args, **kwargs)

    ###################################################

    def configuration_in_collision(self, *args, **kwargs):
        return PychoreoConfigurationCollisionChecker(self.client)(*args, **kwargs)

    ###################################################

    def add_collision_mesh(self, *args, **kwargs):
        return PyBulletAddCollisionMesh(self.client)(*args, **kwargs)

    def append_collision_mesh(self, *args, **kwargs):
        return PyBulletAppendCollisionMesh(self.client)(*args, **kwargs)

    def remove_collision_mesh(self, *args, **kwargs):
        return PyBulletRemoveCollisionMesh(self.client)(*args, **kwargs)

    #####

    def add_attached_collision_mesh(self, *args, **kwargs):
        return PyBulletAddAttachedCollisionMesh(self.client)(*args, **kwargs)

    def remove_attached_collision_mesh(self, *args, **kwargs):
        return PyBulletRemoveAttachedCollisionMesh(self.client)(*args, **kwargs)


