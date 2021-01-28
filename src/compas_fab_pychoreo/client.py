from collections import defaultdict
from itertools import combinations
from termcolor import cprint

from pybullet_planning import HideOutput, CLIENTS
from pybullet_planning import BASE_LINK, RED, GREEN, GREY
from pybullet_planning import get_link_pose, link_from_name, get_disabled_collisions
from pybullet_planning import set_joint_positions
from pybullet_planning import joints_from_names
from pybullet_planning import inverse_kinematics
from pybullet_planning import get_movable_joints  # from pybullet_planning.interfaces.robots.joint
from pybullet_planning import get_joint_names  # from pybullet_planning.interfaces.robots.joint
from pybullet_planning import set_pose, get_bodies, remove_body, create_attachment, set_color
from pybullet_planning import draw_pose, get_body_body_disabled_collisions

from compas_fab.backends import PyBulletClient
from compas_fab_pychoreo.planner import PyChoreoPlanner
from compas_fab_pychoreo.utils import is_valid_option
from compas_fab.backends.pybullet.const import STATIC_MASS

from .exceptions import CollisionError
from .exceptions import InverseKinematicsError
from .conversions import frame_from_pose
from .conversions import pose_from_frame
from .conversions import convert_mesh_to_body

class PyChoreoClient(PyBulletClient):
    """Interface to use pybullet as backend via the **pybullet_plannning**.

    Parameters
    ----------
    viewer : :obj:`bool`
        Enable pybullet GUI. Defaults to True.

    """

    def __init__(self, viewer=True, verbose=False):
        # with HideOutput(not verbose):
        # this does not seem to work
        super(PyChoreoClient, self).__init__(connection_type='gui' if viewer else 'direct', verbose=verbose)
        self.planner = PyChoreoPlanner(self)
        # attached object name => a list of pybullet_planning `Attachment` object
        # notice that parent body (robot) info is stored inside Attachment
        self.pychoreo_attachments = defaultdict(list)
        self.extra_disabled_collision_link_ids = set()

    def connect(self):
        with HideOutput(not self.verbose):
            super(PyChoreoClient, self).connect()
        # TODO, CLIENTS dict required by LockRender
        CLIENTS[self.client_id] = True if self.connection_type == 'gui' else None

    def get_robot_pybullet_uid(self, robot):
        return robot.attributes['pybullet_uid']

    def load_robot(self, urdf_file):
        # PyBulletClient's redirect_stdout does not seem to work...
        with HideOutput(not self.verbose):
            robot = super(PyChoreoClient, self).load_robot(urdf_file)
        return robot

    ###########################################################

    def add_collision_mesh(self, collision_mesh, options=None):
        # add color
        self.planner.add_collision_mesh(collision_mesh, options=options)
        color = is_valid_option(options, 'color', GREY)
        name = collision_mesh.id
        for body in self.collision_objects[name]:
            set_color(body, color)

    def add_attached_collision_mesh(self, attached_collision_mesh, options=None):
        """Adds an attached collision object to the planning scene.

        Note: the pybullet fixed constraint only affects physics simulation by adding an artificial force,
        Thus, by `set_joint_configuration` and `step_simulation`, the attached object will not move together.
        Thus, we use `pybullet_planning`'s `Attachment` class to simplify kinematics.

        Parameters
        ----------
        attached_collision_mesh : :class:`compas_fab.robots.AttachedCollisionMesh`

        Returns
        -------
        attachment : a pybullet_planning `Attachment` object
        """
        name = attached_collision_mesh.collision_mesh.id
        self.planner.add_attached_collision_mesh(attached_collision_mesh, options=options)

        robot = options['robot']
        robot_uid = robot.attributes['pybullet_uid']
        tool_attach_link = link_from_name(robot_uid, attached_collision_mesh.link_name)
        ee_link_pose = get_link_pose(robot_uid, tool_attach_link)
        mass = is_valid_option(options, 'mass', STATIC_MASS)
        options['mass'] = mass
        color = is_valid_option(options, 'color', GREEN)

        for constr_info in self.attached_collision_objects[name]:
            # * update attachment collision links
            body = constr_info.body_id
            for touched_link_name in attached_collision_mesh.touch_links:
                self.extra_disabled_collision_link_ids.add(((robot_uid, link_from_name(robot_uid, touched_link_name)), (body, BASE_LINK)))
            set_pose(body, ee_link_pose)
            set_color(body, color)
            attachment = create_attachment(robot_uid, tool_attach_link, body)
            attachment.assign()
            self.pychoreo_attachments[name].append(attachment)
        return self.pychoreo_attachments[name]

    def remove_attached_collision_mesh(self, name, options=None):
        # ! Remove and detach, ambiguity?
        self.planner.remove_attached_collision_mesh(name, options=options)
        if name in self.pychoreo_attachments:
            del self.pychoreo_attachments[name]

    def detach_attached_collision_mesh(self, name, options=None):
        if name in self.pychoreo_attachments:
            attachments = self.pychoreo_attachments[name]
            del self.pychoreo_attachments[name]
            return attachments
        else:
            cprint('No attachment with name {} found.'.format(name), 'yellow')
            return None

    ########################################

    def set_robot_configuration(self, robot, configuration): #, group=None
        # We enforce that `joint_names` attribute must be specified in `configuration`
        # ! YJ: I don't like the assumption that the "unspecified" joints are assumed to be zero
        # I think it's better to leave them "as-is"
        # https://github.com/compas-dev/compas_fab/blob/master/src/compas_fab/backends/pybullet/client.py#L402
        # super(PyChoreoClient, self).set_robot_configuration(robot, configuration, group=group)
        robot_uid = robot.attributes['pybullet_uid']
        # TODO if joint_names are specified within configuration, take intersection
        # group_joint_names = robot.get_configurable_joint_names(group=group)
        joints = joints_from_names(robot_uid, configuration.joint_names)
        set_joint_positions(robot_uid, joints, configuration.values)
        for attachments in self.pychoreo_attachments.values():
            for attachment in attachments:
                attachment.assign()

    # def configuration_in_collision(self, *args, **kwargs):
    def check_collisions(self, *args, **kwargs):
        return self.planner.check_collisions(*args, **kwargs)

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
