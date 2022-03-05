"""
Internal implementation of the planner backend interface for pybullet_planning
"""
from compas_fab.backends.interfaces.client import PlannerInterface

from compas_fab_pychoreo.backend_features.pychoreo_inverse_kinematics import PyChoreoInverseKinematics
from compas_fab_pychoreo.backend_features.pychoreo_configuration_collision_checker import PyChoreoConfigurationCollisionChecker
from compas_fab_pychoreo.backend_features.pychoreo_sweeping_collision_checker import PyChoreoSweepingCollisionChecker
from compas_fab_pychoreo.backend_features.pychoreo_plan_cartesian_motion import PyChoreoPlanCartesianMotion
from compas_fab_pychoreo.backend_features.pychoreo_plan_motion import PyChoreoPlanMotion

from compas_fab.backends.pybullet.backend_features.pybullet_forward_kinematics import PyBulletForwardKinematics
from compas_fab.backends.pybullet.backend_features.pybullet_add_attached_collision_mesh import PyBulletAddAttachedCollisionMesh
from compas_fab.backends.pybullet.backend_features.pybullet_add_collision_mesh import PyBulletAddCollisionMesh
from compas_fab.backends.pybullet.backend_features.pybullet_append_collision_mesh import PyBulletAppendCollisionMesh
from compas_fab.backends.pybullet.backend_features.pybullet_remove_attached_collision_mesh import PyBulletRemoveAttachedCollisionMesh
from compas_fab.backends.pybullet.backend_features.pybullet_remove_collision_mesh import PyBulletRemoveCollisionMesh

class PyChoreoPlanner(PlannerInterface):
    """Implement the planner backend interface based on pybullet_planning
    """

    def __init__(self, client):
        super(PyChoreoPlanner, self).__init__(client)

    def forward_kinematics(self, *args, **kwargs):
        # ! using main-branch
        return PyBulletForwardKinematics(self.client)(*args, **kwargs)

    def inverse_kinematics(self, *args, **kwargs):
        return PyChoreoInverseKinematics(self.client)(*args, **kwargs)

    def plan_cartesian_motion(self, *args, **kwargs):
        return PyChoreoPlanCartesianMotion(self.client)(*args, **kwargs)

    def plan_motion(self, *args, **kwargs):
        return PyChoreoPlanMotion(self.client)(*args, **kwargs)

    ###################################################

    # def configuration_in_collision(self, *args, **kwargs):
    def check_collisions(self, *args, **kwargs):
        return PyChoreoConfigurationCollisionChecker(self.client)(*args, **kwargs)

    def check_sweeping_collisions(self, *args, **kwargs):
        return PyChoreoSweepingCollisionChecker(self.client)(*args, **kwargs)

    ###################################################
    # ! using main-branch
    def add_collision_mesh(self, *args, **kwargs):
        return PyBulletAddCollisionMesh(self.client)(*args, **kwargs)

    def append_collision_mesh(self, *args, **kwargs):
        return PyBulletAppendCollisionMesh(self.client)(*args, **kwargs)

    def remove_collision_mesh(self, *args, **kwargs):
        return PyBulletRemoveCollisionMesh(self.client)(*args, **kwargs)

    #####
    # ! the following functions we use our implementations in the client

    # def add_attached_collision_mesh(self, *args, **kwargs):
    #     return PyBulletAddAttachedCollisionMesh(self.client)(*args, **kwargs)

    # def remove_attached_collision_mesh(self, *args, **kwargs):
    #     return PyBulletRemoveAttachedCollisionMesh(self.client)(*args, **kwargs)


