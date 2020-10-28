from itertools import combinations
from pybullet_planning import HideOutput
from pybullet_planning import BASE_LINK, RED, GREEN
from pybullet_planning import get_link_pose, link_from_name, get_disabled_collisions
from pybullet_planning import set_joint_positions
from pybullet_planning import joints_from_names
from pybullet_planning import inverse_kinematics
from pybullet_planning import get_movable_joints  # from pybullet_planning.interfaces.robots.joint
from pybullet_planning import get_joint_names  # from pybullet_planning.interfaces.robots.joint
from pybullet_planning import set_pose, get_bodies, remove_body, create_attachment, set_color
from pybullet_planning import draw_pose, get_body_body_disabled_collisions

# // from compas_fab.backends.interfaces.client import ClientInterface
from compas_fab.backends import PyBulletClient
from compas_fab_pychoreo.planner import PychoreoPlanner
from compas_fab_pychoreo.utils import is_valid_option

from .exceptions import CollisionError
from .exceptions import InverseKinematicsError
from .conversions import frame_from_pose
from .conversions import pose_from_frame
from .conversions import convert_mesh_to_body

class PychoreoClient(PyBulletClient):
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
        super(PychoreoClient, self).__init__(connection_type='gui' if viewer else 'direct', verbose=verbose)
        self.planner = PychoreoPlanner(self)
        # attached object name => pybullet_planning `Attachment` object
        # notice that parent body (robot) info is stored inside Attachment
        self.pychoreo_attachments = {}
        self.extra_disabled_collision_link_ids = set()

    ###########################################################

    # def add_collision_mesh(self, collision_mesh, color=RED):
    #     """
    #     """
    #     mesh = collision_mesh.mesh
    #     name = collision_mesh.id
    #     frame = collision_mesh.frame
    #     body = convert_mesh_to_body(mesh, frame, name, color)
    #     if name in self.collision_objects:
    #         self.remove_collision_mesh(name)  # mimic ROS' behaviour: collision object with same name is replaced
    #     self.collision_objects[name] = [body]
    #     return self.collision_objects[name]

    # def remove_collision_mesh(self, name):
    #     if name in self.collision_objects:
    #         for body in self.collision_objects[name]:
    #             remove_body(body)
    #         del self.collision_objects[name]
    #     else:
    #         LOG.warning("Collison object with name '{}' does not exist in scene.".format(name))

    # def append_collision_mesh(self, collision_mesh, color=RED):
    #     """
    #     """
    #     mesh = collision_mesh.mesh
    #     name = collision_mesh.id
    #     frame = collision_mesh.frame
    #     if name in self.collision_objects:
    #         body = convert_mesh_to_body(mesh, frame, name, color)
    #         self.collision_objects[name].append(body)
    #     else:
    #         self.add_collision_mesh(collision_mesh, color)
    #     return self.collision_objects[name]

    def add_attached_collision_mesh(self, attached_collision_mesh, options=None):
        """Adds an attached collision object to the planning scene.

        Parameters
        ----------
        robot : robot model
        attached_collision_mesh : :class:`compas_fab.robots.AttachedCollisionMesh`

        Returns
        -------
        attachment : pybullet_planning Attachment
        """
        name = attached_collision_mesh.collision_mesh.id
        self.planner.add_attached_collision_mesh(attached_collision_mesh, options=options)

        robot = options['robot']
        robot_uid = robot.attributes['pybullet_uid']
        attached_constr_info = self.attached_collision_objects[name]
        tool_attach_link = link_from_name(robot_uid, attached_collision_mesh.link_name)
        ee_link_pose = get_link_pose(robot_uid, tool_attach_link)

        # mesh = attached_collision_mesh.collision_mesh.mesh
        # frame = attached_collision_mesh.collision_mesh.frame
        body = attached_constr_info[0].body_id
        color = is_valid_option(options, 'color', GREEN)

        # * update attachment collision links
        for touched_link_name in attached_collision_mesh.touch_links:
            self.extra_disabled_collision_link_ids.add(((robot_uid, link_from_name(robot_uid, touched_link_name)), (body, BASE_LINK)))
        set_pose(body, ee_link_pose)
        set_color(body, color)
        attachment = create_attachment(robot_uid, tool_attach_link, body)
        attachment.assign()
        self.pychoreo_attachments[name] = attachment
        return attachment

    def remove_attached_collision_mesh(self, name, options=None):
        self.planner.remove_attached_collision_mesh(self, name, options=options)
        if name in self.pychoreo_attachments:
            del self.pychoreo_attachments[name]

    ########################################

    def set_robot_configuration(self, robot, configuration, group=None):
        super(PychoreoClient, self).set_robot_configuration(robot, configuration, group=group)
        for attachment in self.pychoreo_attachments.values():
            attachment.assign()

    def configuration_in_collision(self, *args, **kwargs):
        return self.planner.configuration_in_collision(*args, **kwargs)

    ########################################
    # ignored collision info (similar to ROS's Allowed Collision Matrix)
    def get_self_collision_link_ids(self, robot):
        # return set of 2-int pair
        if robot.semantics is not None:
            robot_uid = robot.attributes['pybullet_uid']
            self_collision_disabled_link_names = robot.semantics.disabled_collisions
            return get_disabled_collisions(robot_uid, self_collision_disabled_link_names)
        else:
            return {}
