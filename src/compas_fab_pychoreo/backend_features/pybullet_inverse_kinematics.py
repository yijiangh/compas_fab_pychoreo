from compas_fab.backends.interfaces import InverseKinematics

class PybulletInverseKinematics(InverseKinematics):
    def __init__(self, pb_client, robot_uid, custom_ik_fn=None):
        self.pb_client = pb_client
        self.robot_uid = robot_uid
        self.custom_ik_fn = custom_ik_fn # if None use default pybullet ik

    def inverse_kinematics(self, frame_WCF, start_configuration=None, group=None, options=None):
        """Calculate the robot's inverse kinematic for a given frame.
        Parameters
        ----------
        frame_WCF: :class:`compas.geometry.Frame`
            The frame to calculate the inverse for.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
            If passed, the inverse will be calculated such that the calculated
            joint positions differ the least from the start_configuration.
            Defaults to the init configuration.
        group: str, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        options: dict, optional
            Dictionary containing the following key-value pairs:
            - ``"base_link"``: (:obj:`str`) Name of the base link.
            - ``"avoid_collisions"``: (:obj:`bool`, optional) Whether or not to avoid collisions.
              Defaults to `True`.
            - ``"constraints"``: (:obj:`list` of :class:`compas_fab.robots.Constraint`, optional)
              A set of constraints that the request must obey. Defaults to `None`.
            - ``"attempts"``: (:obj:`int`, optional) The maximum number of inverse kinematic attempts.
              Defaults to `8`.
            - ``"attached_collision_meshes"``: (:obj:`list` of :class:`compas_fab.robots.AttachedCollisionMesh`, optional)
              Defaults to `None`.
        Raises
        ------
        compas_fab.backends.exceptions.BackendError
            If no configuration can be found.
        Returns
        -------
        :class:`compas_fab.robots.Configuration`
            The planning group's configuration.
        """
        pass

    def inverse_kinematics_pb(self, pose):
        pass
