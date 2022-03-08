import pybullet_planning as pp
from compas_fab.backends.interfaces import RemoveAttachedCollisionMesh
from ..utils import wildcard_keys

class PyChoreoRemoveAttachedCollisionMesh(RemoveAttachedCollisionMesh):
    """Callable to remove an attached collision mesh from the robot."""
    def __init__(self, client):
        self.client = client

    def remove_attached_collision_mesh(self, name, options=None):
        wildcard = options.get('wildcard') or '^{}$'.format(name)
        names = wildcard_keys(self.client.pychoreo_attachments, wildcard)
        for name in names:
            del self.client.attached_collision_objects[name]
            del self.client.pychoreo_attachments[name]
            del self.client.extra_disabled_collision_links[name]
