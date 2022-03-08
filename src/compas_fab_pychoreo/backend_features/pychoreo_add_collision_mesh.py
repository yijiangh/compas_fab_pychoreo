import pybullet_planning as pp
from compas_fab.backends.interfaces import AddCollisionMesh
from compas_fab.backends.pybullet.backend_features import PyBulletAddCollisionMesh

class PyChoreoAddCollisionMesh(AddCollisionMesh):
    """Callable to add a collision mesh to the planning scene."""
    def __init__(self, client):
        self.client = client

    def add_collision_mesh(self, collision_mesh, options=None):
        """Add a collision mesh to the planning scene.

        Parameters
        ----------
        collision_mesh : :class:`compas_fab.robots.CollisionMesh`
            Object containing the collision mesh to be added.
        options : dict
            Dictionary containing the following key-value pairs:

            - ``"mass"``: (:obj:`float`) The mass of the object, in kg.
              If `0` is given, (the default), the object added is static.

        Returns
        -------
        ``None``
        """
        options = options or {}
        PyBulletAddCollisionMesh(self.client)(collision_mesh, options=options)
        color = options.get('color', pp.GREY)
        name = collision_mesh.id
        for body in self.client.collision_objects[name]:
            pp.set_color(body, color)

