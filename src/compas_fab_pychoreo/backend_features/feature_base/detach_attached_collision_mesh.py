class DetachAttachedCollisionMesh(object):
    """Interface for a Planner's **detach** attached collision mesh feature.  Any implementation of
    ``DetachAttachedCollisionMesh`` must define the method ``detach_attached_collision_mesh``.  The
    ``__call__`` magic method allows an instance of an implementation of
    ``DetachAttachedCollisionMesh`` to be treated as its ``detach_attached_collision_mesh`` method.  See
    <https://docs.python.org/3/reference/datamodel.html#object.__call__> and
    <https://en.wikipedia.org/wiki/Function_object#In_Python>.
    """

    def __call__(self, name, options=None):
        return self.detach_attached_collision_mesh(name, options)

    def detach_attached_collision_mesh(self, name, options=None):
        """Detach an attached collision mesh from the robot.

        Parameters
        ----------
        id : str
            Name of collision mesh to be removed.
        options : dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

        Returns
        -------
        xxx
        """
        pass
