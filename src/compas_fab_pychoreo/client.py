from compas_fab.robots.configuration import Configuration
import pybullet
from collections import defaultdict
from itertools import combinations
from termcolor import cprint

from pybullet_planning import HideOutput, CLIENTS, load_pybullet
from pybullet_planning import BASE_LINK, RED, GREY, YELLOW, BLUE
from pybullet_planning import get_link_pose, link_from_name, get_disabled_collisions, get_all_links
from pybullet_planning import set_joint_positions
from pybullet_planning import joints_from_names, get_joint_positions
from pybullet_planning import inverse_kinematics
from pybullet_planning import get_movable_joints  # from pybullet_planning.interfaces.robots.joint
from pybullet_planning import get_joint_names  # from pybullet_planning.interfaces.robots.joint
from pybullet_planning import set_pose, get_bodies, remove_body, create_attachment, set_color, get_link_name, get_name
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
        # PybulletClient keeps
        #   self.collision_objects
        #   self.attached_collision_objects

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
        options = options or {}
        self.planner.add_collision_mesh(collision_mesh, options=options)
        color = options.get('color', GREY)
        name = collision_mesh.id
        for body in self.collision_objects[name]:
            set_color(body, color)

    def add_tool_from_urdf(self, tool_name, urdf_file):
        # TODO return compas_fab Tool class
        # TODO we might need to keep track of tools as robots too
        with HideOutput(not self.verbose):
            tool_robot = load_pybullet(urdf_file, fixed_base=False)
        self.collision_objects[tool_name] = [tool_robot]

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
        mass = options.get('mass', STATIC_MASS)
        options['mass'] = mass
        color = options.get('color', BLUE)
        attached_child_link_name = options.get('attached_child_link_name', None)

        robot_uid = robot.attributes['pybullet_uid']
        name = attached_collision_mesh.collision_mesh.id
        attached_bodies = []
        # ! mimic ROS' behavior: collision object with same name is replaced
        if name in self.attached_collision_objects:
            cprint('Replacing existing attached collision mesh {}'.format(name), 'yellow')
            self.remove_attached_collision_mesh(name)
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
            # TODO let user choose to include the direct touch link collision or not
            # * update attachment collision links
            for touched_link_name in attached_collision_mesh.touch_links:
                self.extra_disabled_collision_links[name].add(
                    ((robot_uid, touched_link_name), (body, None))
                    )
            links = get_all_links(body)
            for link in links:
                set_color(body, color, link=link)
            # create attachment based on their *current* pose
            attach_child_link = BASE_LINK if not attached_child_link_name else link_from_name(body, attached_child_link_name)
            attachment = create_attachment(robot_uid, tool_attach_link, body, attach_child_link)
            attachment.assign()
            self.pychoreo_attachments[name].append(attachment)
            # create fixed constraint to conform to PybulletClient (we don't use it though)
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
            del self.extra_disabled_collision_links[name]

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
            del self.extra_disabled_collision_links[name]
        return detached_attachments

    ########################################
    # Collision / Attached pybullet body managment

    def _get_collision_object_names(self, wildcard):
        return wildcard_keys(self.collision_objects, wildcard)

    # TODO separation between attached and collision objects might cause problems in mode changes
    def _get_collision_object_bodies(self, wildcard):
        bodies = []
        for n in self._get_collision_object_names(wildcard):
            bodies.extend(self.collision_objects[n])
        return bodies

    def _get_attachment_names(self, wildcard):
        return wildcard_keys(self.pychoreo_attachments, wildcard)

    # TODO separation between attached and collision objects might cause problems in mode changes
    def _get_attached_bodies(self, wildcard):
        bodies = []
        for n in self._get_attachment_names(wildcard):
            bodies.extend([at.child for at in self.pychoreo_attachments[n]])
        return bodies

    def _get_body_status(self, wildcard):
        # -1 if not attached and not collision object
        # 0 if collision object, 1 if attached
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

    def _get_bodies(self, wildcard):
        co_bodies = self._get_collision_object_bodies(wildcard)
        attached_bodies = self._get_attached_bodies(wildcard)
        return co_bodies + attached_bodies

    @property
    def _name_from_body_id(self):
        name_from_body_id = {}
        for name, bodies in self.collision_objects.items():
            for body in bodies:
                name_from_body_id[body] = name
        for name, attachments in self.pychoreo_attachments.items():
            for attach in attachments:
                name_from_body_id[attach.child] = name
        return name_from_body_id

    def _get_name_from_body_id(self, body_id):
        name_from_body_id = self.name_from_body_id
        return None if body_id not in name_from_body_id else name_from_body_id[body_id]

    def set_object_frame(self, wildcard, frame, options=None):
        bodies = self._get_bodies(wildcard)
        for body in bodies:
            set_pose(body, pose_from_frame(frame))

    # TODO
    # def set_object_frame(self, wildcard):

    def _print_object_summary(self):
        print('^'*10)
        print('PychoreoClient scene summary:')
        body_name_from_id = self._name_from_body_id
        print('Collision Objects:')
        for name, bodies in self.collision_objects.items():
            print('\t{}: {}'.format(name, bodies))
        print('Attachments:')
        for name, attachments in self.pychoreo_attachments.items():
            print('\t{}: {}'.format(name, [at.child for at in attachments]))
        print('Extra disabled collision links:')
        for name, blink_pairs in self.extra_disabled_collision_links.items():
            print('\t{}:'.format(name))
            for (b1,l1_name), (b2,l2_name) in blink_pairs:
                b1_name = body_name_from_id[b1] if b1 in body_name_from_id else get_name(b1)
                b2_name = body_name_from_id[b2] if b2 in body_name_from_id else get_name(b2)
                print('\t\t({}-{}), ({}-{})'.format(b1_name,l1_name,b2_name,l2_name))

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
        self._set_body_configuration(robot_uid, configuration)
        for attachments in self.pychoreo_attachments.values():
            for attachment in attachments:
                attachment.assign()

    def _set_body_configuration(self, body_id, configuration):
        joints = joints_from_names(body_id, configuration.joint_names)
        set_joint_positions(body_id, joints, configuration.values)

    def get_robot_configuration(self, robot, group):
        robot_uid = robot.attributes['pybullet_uid']
        joint_names = robot.get_configurable_joint_names(group=group)
        joints = joints_from_names(robot_uid, joint_names)
        joint_types = robot.get_joint_types_by_names(joint_names)
        joint_values = get_joint_positions(robot_uid, joints)
        return Configuration(values=joint_values, types=joint_types, joint_names=joint_names)

    # def configuration_in_collision(self, *args, **kwargs):
    def check_collisions(self, *args, **kwargs):
        return self.planner.check_collisions(*args, **kwargs)

    def get_link_frame_from_name(self, robot, link_name):
        robot_uid = robot.attributes['pybullet_uid']
        return frame_from_pose(get_link_pose(robot_uid, link_from_name(robot_uid, link_name)))

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
