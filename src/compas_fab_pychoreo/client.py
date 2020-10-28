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

from compas_fab.backends import PyBulletClient
from compas_fab_pychoreo.planner import PyChoreoPlanner
from compas_fab_pychoreo.utils import is_valid_option

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
        super(PyChoreoClient, self).__init__(connection_type='gui' if viewer else 'direct', verbose=verbose)
        self.planner = PyChoreoPlanner(self)
        # attached object name => pybullet_planning `Attachment` object
        # notice that parent body (robot) info is stored inside Attachment
        self.pychoreo_attachments = {}
        self.extra_disabled_collision_link_ids = set()

    ###########################################################

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
        attached_constr_info = self.attached_collision_objects[name]
        tool_attach_link = link_from_name(robot_uid, attached_collision_mesh.link_name)
        ee_link_pose = get_link_pose(robot_uid, tool_attach_link)

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
        # wrapper for PyBulletClient's `set_robot_configuration` so that all the attachments are updated as well
        super(PyChoreoClient, self).set_robot_configuration(robot, configuration, group=group)
        for attachment in self.pychoreo_attachments.values():
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
