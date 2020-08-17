from itertools import combinations
from pybullet_planning import is_connected, disconnect, connect, load_pybullet, CLIENT, HideOutput
from pybullet_planning import BASE_LINK
from pybullet_planning import get_link_pose, link_from_name, get_disabled_collisions
from pybullet_planning import set_joint_positions
from pybullet_planning import joints_from_names
from pybullet_planning import inverse_kinematics
from pybullet_planning import get_movable_joints  # from pybullet_planning.interfaces.robots.joint
from pybullet_planning import get_joint_names  # from pybullet_planning.interfaces.robots.joint
from pybullet_planning import set_pose, get_bodies, remove_body, create_attachment
from pybullet_planning import draw_pose, get_body_body_disabled_collisions

from compas_fab.backends.interfaces.client import ClientInterface
from compas_fab_pychoreo.planner import PybulletPlanner

from .utils import LOG
from .exceptions import CollisionError
from .exceptions import InverseKinematicsError
from .conversions import frame_from_pose
from .conversions import pose_from_frame
from .conversions import convert_mesh_to_body

class PyBulletClient(ClientInterface):
    """Interface to use pybullet as backend via the **pybullet_plannning**.

    :class:`.PybulletClient` is a context manager type, so it's best
    used in combination with the ``with`` statement to ensure
    resource deallocation.

    In compas_fab, multiple robots need to be assembled in a single URDF. And agent coordination is
    done by working with manipulation groups.

    https://github.com/compas-dev/compas_fab/blob/master/src/compas_fab/backends/interfaces/client.py

    Parameters
    ----------
    viewer : :obj:`bool`
        Enable pybullet GUI. Defaults to True.

    Examples
    --------
    >>> from compas_fab.backends import PybulletClient
    >>> with PybulletClient() as client:
    ...     print('Connected: %s' % client.is_connected)
    Connected: 1

    """

    def __init__(self, viewer=True, verbose=False):
        super(PyBulletClient, self).__init__()
        self.viewer = viewer
        self.verbose = verbose
        # TODO: only one robot for now
        # self.urdf_filename = None
        self.compas_fab_robot = None
        self.robot_uid = None
        #
        self.collision_objects = {}
        self.attachments = {}
        self.planner = PybulletPlanner(self)
        #
        self.extra_disabled_collisions = set()

    # @classmethod
    # def from_robot(cls, robot, urdf_filename=None, viewer=True, verbose=False):
    #     if urdf_filename is None:
    #         raise NotImplementedError()
    #     client = cls(viewer, verbose)
    #     client.compas_fab_robot = robot
    #     return client

    ###########################################################

    def __enter__(self):
        connect(use_gui=self.viewer)
        # if self.compas_fab_robot:
        #     robot = self.compas_fab_robot
        #     self.load_robot_from_urdf(robot.model.urdf_filename)
        return self

    def __exit__(self, *args):
        disconnect()

    @property
    def is_connected(self):
        """Indicates whether the client has an active connection.

        Returns
        -------
        bool
            `True` if connected, `False` otherwise.
        """
        return is_connected()

    ###########################################################

    def load_robot_from_urdf(self, urdf_filename):
        """Create a pybullet robot using the input urdf file.

        Parameters
        ----------
        urdf_filename : str
            absolute file path to the urdf file. The mesh file can be linked by either
            `package::` or relative path.

        Returns
        -------
        int
            a pybullet body unique index.
        """
        with HideOutput(not self.verbose):
            self.robot_uid = load_pybullet(urdf_filename, fixed_base=True)
            # TODO create compas_fab robot here
        return self.robot_uid

    def ensure_robot(self):
        """Checks if the robot is loaded."""
        if self.robot_uid is None and self.robot_uid in get_bodies():
            raise Exception('This method is only callable once a robot is loaded')

    ###########################################################

    def forward_kinematics(self, robot, configuration, group, ee_link_name, check_collision=True):
        """we do not need a backend feature for the FK case.
        """
        self.ensure_robot()
        joints = joints_from_names(self.robot_uid, configuration.joint_names)
        ee_link = link_from_name(self.robot_uid, ee_link_name)
        set_joint_positions(self.robot_uid, joints, configuration.values)
        pose = get_link_pose(self.robot_uid, ee_link)
        if check_collision:
            # collision, names = self.collision_check()
            # if collision:
            #     raise CollisionError(*names)
            raise NotImplementedError()
        return frame_from_pose(pose)

    ###########################################################

    def add_collision_mesh(self, collision_mesh):
        """
        """
        mesh = collision_mesh.mesh
        name = collision_mesh.id
        frame = collision_mesh.frame
        body = convert_mesh_to_body(mesh, frame, name)
        if name in self.collision_objects:
            self.remove_collision_mesh(name)  # mimic ROS' behaviour: collision object with same name is replaced
        self.collision_objects[name] = [body]

    def remove_collision_mesh(self, name):
        if name in self.collision_objects:
            for body in self.collision_objects[name]:
                remove_body(body)
        else:
            LOG.warning("Collison object with name '{}' does not exist in scene.".format(name))

    def append_collision_mesh(self, collision_mesh):
        """
        """
        mesh = collision_mesh.mesh
        name = collision_mesh.id
        frame = collision_mesh.frame
        if name in self.collision_objects:
            body = convert_mesh_to_body(mesh, frame, name)
            self.collision_objects[name].append(body)
        else:
            self.add_collision_mesh(collision_mesh)

    def transform_collision_objects_to_frame(self, names, frame):
        bodies = [body for name in names for body in self.collision_objects[name]]
        for body in bodies:
            set_pose(body, pose_from_frame(frame))

    def add_attached_collision_mesh(self, attached_collision_mesh):
        """Adds an attached collision object to the planning scene.

        Parameters
        ----------
        attached_collision_mesh : :class:`compas_fab.robots.AttachedCollisionMesh`

        Returns
        -------
        attachment : pybullet Attachment
        """
        robot_uid = self.robot_uid
        tool_attach_link = link_from_name(robot_uid, attached_collision_mesh.link_name)
        ee_link_pose = get_link_pose(robot_uid, tool_attach_link)

        mesh = attached_collision_mesh.collision_mesh.mesh
        name = attached_collision_mesh.collision_mesh.id
        frame = attached_collision_mesh.collision_mesh.frame
        body = convert_mesh_to_body(mesh, frame, name)
        # TODO
        # if name in self.attachments:
        #     # mimic ROS' behaviour: collision object with same name is replaced
        #     self.remove_collision_mesh(name)
        # * update attachment collision links
        for touched_link_name in attached_collision_mesh.touch_links:
            self.extra_disabled_collisions.add(((robot_uid, link_from_name(robot_uid, touched_link_name)), (body, BASE_LINK)))
        set_pose(body, ee_link_pose)
        attachment = create_attachment(robot_uid, tool_attach_link, body)
        attachment.assign()
        self.attachments[name] = attachment
        return attachment

    def remove_attached_collision_mesh(self, id):
        raise NotImplementedError

    ########################################

    def configuration_in_collision(self, *args, **kwargs):
        return self.planner.configuration_in_collision(*args, **kwargs)

    ########################################
    # ignored collision info (similar to ROS's Allowed Collision Matrix)
    @property
    def self_collision_links(self):
        # return set of 2-int pair
        self_collision_disabled_link_names = self.compas_fab_robot.semantics.disabled_collisions
        return get_disabled_collisions(self.robot_uid, self_collision_disabled_link_names)
