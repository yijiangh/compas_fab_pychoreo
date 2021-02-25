import pybullet
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
from pybullet_planning import add_fixed_constraint, remove_constraint
from pybullet_planning import draw_pose, get_body_body_disabled_collisions

from compas_fab.backends import PyBulletClient
from compas_fab.backends.pybullet.const import ConstraintInfo

from compas_fab_pychoreo.planner import PyChoreoPlanner
from compas_fab_pychoreo.utils import wildcard_keys
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
        # name -> ((robot_uid, touched_link_name), (body, None))
        # None means BASE_LINK
        self.extra_disabled_collision_links = defaultdict(set)

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
        color = options.get('color') or GREY
        name = collision_mesh.id
        for body in self.collision_objects[name]:
            set_color(body, color)

    def add_attached_collision_mesh(self, attached_collision_mesh, options=None):
        """Adds an attached collision object to the planning scene, the grasp pose is set according to the
        **current** relative pose in the scene. Thus, the robot conf needs to be set to the right values to
        make the grasp pose right.

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
        options = options or {}
        assert 'robot' in options, 'The robot option must be specified!'
        robot = options['robot']
        mass = options.get('mass') or STATIC_MASS
        options['mass'] = mass
        color = options.get('color') or GREEN

        robot_uid = robot.attributes['pybullet_uid']
        name = attached_collision_mesh.collision_mesh.id
        attached_bodies = []
        if name not in self.collision_objects:
            # ! I don't want to add another copy of the objects
            # self.planner.add_attached_collision_mesh(attached_collision_mesh, options=options)
            # attached_bodies = [constr.body_id for constr in self.attached_collision_objects[name]]
            self.planner.add_collision_mesh(attached_collision_mesh.collision_mesh, options=options)
        attached_bodies = self.collision_objects[name]
        del self.collision_objects[name]
        self.attached_collision_objects[name] = []

        tool_attach_link = link_from_name(robot_uid, attached_collision_mesh.link_name)
        for body in attached_bodies:
            # * update attachment collision links
            for touched_link_name in attached_collision_mesh.touch_links:
                self.extra_disabled_collision_links[name].add(
                    ((robot_uid, touched_link_name), (body, None))
                    )
            set_color(body, color)
            # create attachment based on their *current* pose
            attachment = create_attachment(robot_uid, tool_attach_link, body)
            attachment.assign()
            self.pychoreo_attachments[name].append(attachment)

            # create fixed constraint to conform to PybulletClient
            constraint_id = add_fixed_constraint(attachment.child, attachment.parent, attachment.parent_link)
            constraint_info = ConstraintInfo(constraint_id, attachment.child, attachment.parent)
            self.attached_collision_objects[name].append(constraint_info)

        return self.pychoreo_attachments[name]

    def remove_attached_collision_mesh(self, name, options=None):
        # ! Remove and detach, ambiguity?
        self.planner.remove_attached_collision_mesh(name, options=options)
        wildcard = options.get('wildcard') or '^{}$'.format(name)
        names = wildcard_keys(self.pychoreo_attachments, wildcard)
        for name in names:
            del self.attached_collision_objects[name]
            del self.pychoreo_attachments[name]

    def detach_attached_collision_mesh(self, name, options=None):
        # detach attached collision mesh, and leave them in the world as collision objects
        wildcard = options.get('wildcard') or '^{}$'.format(name)
        names = wildcard_keys(self.pychoreo_attachments, wildcard)
        if len(names) == 0:
            cprint('No attachment with name {} found.'.format(name), 'yellow')
            return None
        detached_attachments = []
        for name in names:
            attachments = self.pychoreo_attachments[name]
            detached_attachments.extend(attachments)
            # * add detached attachments to collision_objects
            self.collision_objects[name] = [at.child for at in attachments]
            for constraint_info in self.attached_collision_objects[name]:
                remove_constraint(constraint_info.constraint_id)
            del self.attached_collision_objects[name]
            del self.pychoreo_attachments[name]
        return detached_attachments

    ########################################

    def set_object_frame(self, name, frame, options=None):
        wildcard = options.get('wildcard') or '^{}$'.format(name)
        status = self._get_body_statues(wildcard)
        assert status != -1
        contain_dict = self.collision_objects if status == 0 else self.pychoreo_attachments
        names = wildcard_keys(contain_dict, wildcard)
        # if len(names) == 0:
        #     cprint('wildcard {} not found in {}'.format(wildcard, self.collision_objects.keys()), 'yellow')
        for name in names:
            for bdata in contain_dict[name]:
                set_pose(bdata if status == 0 else bdata.child, pose_from_frame(frame))

    def _get_body_statues(self, wildcard):
        co_names = wildcard_keys(self.collision_objects, wildcard)
        at_names = wildcard_keys(self.pychoreo_attachments, wildcard)
        if len(co_names) == 0 and len(at_names) == 0:
            return -1
        elif len(co_names) > 0 and len(at_names) == 0:
            return 0
        elif len(co_names) == 0 and len(at_names) > 0:
            return 1
        else:
            raise ValueError('names {} should not appear at both collision objects ({}) and attached objects ({}) at the same time!'.format(
                wildcard, co_names, at_names))

    def _get_collision_object_names(self, wildcard):
        return wildcard_keys(self.collision_objects, wildcard)

    # TODO separation between attached and collision objects might cause problems in mode changes
    def _get_collision_object_bodies(self, wildcard):
        # wildcard = wildcard or '^{}$'.format(name)
        bodies = []
        for n in self._get_collision_object_names(wildcard):
            bodies.extend(self.collision_objects[n])
        return bodies

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
