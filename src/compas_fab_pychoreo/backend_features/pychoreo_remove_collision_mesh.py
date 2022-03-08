import pybullet_planning as pp
from compas_fab.backends.interfaces import RemoveCollisionMesh
from ..utils import LOGGER

class PyBulletRemoveCollisionMesh(RemoveCollisionMesh):
    """Callable to remove a collision mesh from the planning scene."""
    def __init__(self, client):
        self.client = client

    def remove_collision_mesh(self, id, options=None):
        """Remove a collision mesh from the planning scene.

        Parameters
        ----------
        id : str
            Name of collision mesh to be removed.
        options : dict, optional
            Unused parameter.

        Returns
        -------
        ``None``
        """
        if id not in self.client.collision_objects:
            LOGGER.warning("Collision object with name '{}' does not exist in scene.".format(id))
            return

        for body_id in self.client.collision_objects[id]:
            pp.remove_body(body_id)

