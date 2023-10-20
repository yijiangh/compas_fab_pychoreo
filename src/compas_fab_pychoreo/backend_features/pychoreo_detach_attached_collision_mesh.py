import pybullet_planning as pp
from .feature_base import DetachAttachedCollisionMesh
from ..utils import wildcard_keys, LOGGER

class PyChoreoDetachAttachedCollisionMesh(DetachAttachedCollisionMesh):
    """Callable to detach an attached collision mesh from the robot."""
    def __init__(self, client):
        self.client = client

    def detach_attached_collision_mesh(self, name, options=None):
        """Detach an attached collision mesh from the robot.

        Parameters
        ----------
        name : str
            Name of collision mesh to be removed.
        options : dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

        Returns
        -------
        xxx
        """
        # detach attached collision mesh, and leave them in the world as collision objects
        options = options or {}
        wildcard = options.get('wildcard') or '^{}$'.format(name)
        names = wildcard_keys(self.client.pychoreo_attachments, wildcard)
        if len(names) == 0:
            LOGGER.warning('No attachment with name {} found.'.format(name), 'yellow')
            return None
        detached_attachments = []
        for name in names:
            attachments = self.client.pychoreo_attachments[name]
            detached_attachments.extend(attachments)
            # * add detached attachments to collision_objects
            self.client.collision_objects[name] = [at.child for at in attachments]
            for constraint_info in self.client.attached_collision_objects[name]:
                if constraint_info.constraint_id in pp.get_constraints():
                    pp.remove_constraint(constraint_info.constraint_id)
            del self.client.attached_collision_objects[name]
            del self.client.pychoreo_attachments[name]
            del self.client.extra_disabled_collision_links[name]
        return detached_attachments
