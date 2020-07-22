from itertools import combinations
import pybullet
from pybullet_planning import is_connected, disconnect, connect, load_pybullet, CLIENT
from pybullet_planning import HideOutput
from pybullet_planning import get_link_pose
from pybullet_planning import link_from_name
from pybullet_planning import set_joint_positions
from pybullet_planning import joints_from_names
from pybullet_planning import inverse_kinematics
from pybullet_planning import get_movable_joints  # from pybullet_planning.interfaces.robots.joint
from pybullet_planning import get_joint_names  # from pybullet_planning.interfaces.robots.joint
from pybullet_planning import set_pose

from .utils import LOG
from .exceptions import CollisionError
from .exceptions import InverseKinematicsError
from .conversions import frame_from_pose
from .conversions import pose_from_frame
from .conversions import convert_mesh_to_body


def get_disabled_collisions(semantics):
    # TODO: move to semantics
    disabled_collisions = {}
    for dc in semantics.root.iter('disable_collisions'):
        link1, link2 = dc.attrib['link1'], dc.attrib['link2']
        if link1 not in disabled_collisions:
            disabled_collisions.update({link1: []})
        disabled_collisions[link1].append(link2)
    return disabled_collisions


def get_link_names_with_collision_geometry(robot):
    # todo move to robot model and robot
    names = []
    for link in robot.model.iter_links():
        if len(link.collision):
            names.append(link.name)
    return names


class PyBulletClient(object):
    """Interface to use pybullet as backend via the **pybullet_plannning**.

    :class:`.PybulletClient` is a context manager type, so it's best
    used in combination with the ``with`` statement to ensure
    resource deallocation.

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

    Note
    ----
    For more examples, check out the :ref:`ROS examples page <ros_examples>`.
    """

    def __init__(self, viewer=True, verbose=False):
        super(PyBulletClient, self).__init__()
        self.viewer = viewer
        self.verbose = verbose
        # the following need to be created, ideally through classmethod creators
        self.compas_fab_robot = None
        self.robot_uid = None
        self.collision_objects = {}
        self.attached_collision_objects = {}
        # self.collision_map = {}

    @classmethod
    def from_robot(cls, robot, viewer=True, verbose=False):
        # https://github.com/bulletphysics/bullet3/blob/aec9968e281faca7bc56bc05ccaf0ef29d82d062/examples/pybullet/gym/pybullet_utils/urdfEditor.py
        # https://github.com/bulletphysics/bullet3/blob/9d1a2c4c571ff2376949c99690fca1023933d7d7/examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp
        client = cls(viewer, verbose)
        client.compas_fab_robot = robot
        return client

    def __enter__(self):
        connect(use_gui=self.viewer)
        if self.compas_fab_robot:
            robot = self.compas_fab_robot
            self.load_robot_from_urdf(robot.model.urdf_filename)
            # self.create_collision_map(robot)
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

    def load_robot_from_urdf(self, urdf_filename):
        """Create a pybullet robot using the input urdf file.

        Parameters
        ----------
        urdf_filename : str
            absolute file path to the urdf file. The mesh file can be linked by either
            `package::` or relative path.
        fixed_base : bool, optional
            True if the robot is fixed-base, by default True

        Returns
        -------
        int
            a pybullet body unique index.
        """
        # TODO: fuse end effector and robot link tree into one
        with HideOutput(not self.verbose):
            self.robot_uid = load_pybullet(urdf_filename, fixed_base=True)

    # def create_collision_map(self, robot, disabled_collisions=None):
    #     """
    #     """
    #     link_names = get_link_names_with_collision_geometry(robot)
    #     disabled_collisions = disabled_collisions or {}
    #     if robot.semantics:
    #         disabled_collisions.update(get_disabled_collisions(robot.semantics))
    #     collision_map = {}
    #     for link1, link2 in combinations(link_names, 2):
    #         if link1 in disabled_collisions and link2 in disabled_collisions[link1]:
    #             continue
    #         if link2 in disabled_collisions and link1 in disabled_collisions[link2]:
    #             continue
    #         if link1 not in collision_map:
    #             collision_map[link1] = []
    #         collision_map[link1].append(link2)
    #     self.collision_map = collision_map

    def forward_kinematics(self, robot, configuration, group, ee_link_name, check_collision=True):
        """
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

    def configuration_in_collision(self, configuration):
        raise NotImplementedError()
        # joints = joints_from_names(self.robot_uid, configuration.joint_names)
        # set_joint_positions(self.robot_uid, joints, configuration.values)
        # collision, names = self.collision_check()
        # return collision

    def check_configurations_for_collision(self, configurations):
        """Used for a custom inverse kinematics function.
        """
        for i, configuration in enumerate(configurations):
            if not configuration:  # if an ik solution was already removed
                continue
            in_collision = self.configuration_in_collision(configuration)
            if in_collision:
                configurations[i] = None

    def check_collision_objects_for_collision(self):
        names = self.collision_objects.keys()
        for name1, name2 in combinations(names, 2):
            for body1 in self.collision_objects[name1]:
                for body2 in self.collision_objects[name2]:
                    pts = pybullet.getClosestPoints(bodyA=body1, bodyB=body2, distance=0, physicsClientId=CLIENT)
                    if len(pts):
                        LOG.warning("Collision between '{}' and '{}'".format(name1, name2))
                        return True, (name1, name2)
        return False, (None, None)

    def transform_collision_objects_to_frame(self, names, frame):
        bodies = [body for name in names for body in self.collision_objects[name]]
        for body in bodies:
            set_pose(body, pose_from_frame(frame))

    # def collision_check(self):
    #     """
    #     depending on if the number of collision objects is higher than the number of robot links then we should change order...
    #     """
    #     # collision objects
    #     for name, bodies in self.collision_objects.items():
    #         for body in bodies:
    #             pts = pybullet.getClosestPoints(bodyA=self.robot_uid, bodyB=body, distance=0, physicsClientId=CLIENT)
    #             if len(pts):
    #                 LOG.warning("Collision between 'robot' and '{}'".format(name))
    #                 return True, ("robot", name)

    #     # robot links
    #     for link1_name, names in self.collision_map.items():
    #         for link2_name in names:
    #             link1 = link_from_name(self.robot_uid, link1_name)
    #             link2 = link_from_name(self.robot_uid, link2_name)
    #             pts = pybullet.getClosestPoints(bodyA=self.robot_uid, bodyB=self.robot_uid, distance=0, linkIndexA=link1, linkIndexB=link2, physicsClientId=CLIENT)
    #             if len(pts):
    #                 LOG.warning("Collision between '{}' and '{}'".format(link1_name, link2_name))
    #                 return True, (link1_name, link2_name)

    #     # attached collision objects
    #     # TODO

    #     return False, ()

    def ensure_robot(self):
        """Checks if the robot is loaded."""
        if self.robot_uid is None:
            raise Exception('This method is only callable once a robot is loaded')

    def inverse_kinematics(self, robot, frame, group,
                           start_configuration, avoid_collisions=True,
                           constraints=None, attempts=8,
                           attached_collision_meshes=None):
        """
        # TODO: use full advantage of pybullets IK, witth constraints....
        # https://github.com/bulletphysics/bullet3/blob/2d1594cb29eb6ab477f8ee23db9c1f0e35a4697a/examples/pybullet/examples/inverse_kinematics.py
        """
        self.ensure_robot()
        ee_link_name = robot.get_end_effector_link_name(group)
        pb_ee_link = link_from_name(self.robot_uid, ee_link_name)
        joint_names = robot.get_configurable_joint_names()
        joint_names = [name.decode('UTF-8') for name in get_joint_names(self.robot_uid, get_movable_joints(self.robot_uid))]
        joint_positions = inverse_kinematics(self.robot_uid, pb_ee_link, pose_from_frame(frame), max_iterations=attempts)
        if not joint_positions:
            raise InverseKinematicsError()
        return joint_positions, joint_names

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
                pybullet.removeBody(body)
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

    def add_attached_collision_mesh(self, attached_collision_mesh):
        raise NotImplementedError

    def remove_attached_collision_mesh(self, id):
        raise NotImplementedError
