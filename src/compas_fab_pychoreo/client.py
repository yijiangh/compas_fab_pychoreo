import os
import shutil
import tempfile

from itertools import product, combinations
from collections import defaultdict
from itertools import combinations

import numpy as np
import pybullet_planning as pp
from pybullet_planning import HideOutput, CLIENTS, load_pybullet, INFO_FROM_BODY, ModelInfo
from pybullet_planning import BASE_LINK, RED, GREY, YELLOW, BLUE
from pybullet_planning import get_link_pose, link_from_name, get_disabled_collisions, get_all_links
from pybullet_planning import set_joint_positions
from pybullet_planning import joints_from_names, get_joint_positions
from pybullet_planning import set_pose, set_color, get_name
from pybullet_planning import get_pose
from pybullet_planning import draw_collision_diagnosis, expand_links, pairwise_link_collision, pairwise_link_collision_info

from compas_fab.backends import PyBulletClient
from compas_fab.backends.pybullet.const import ConstraintInfo, STATIC_MASS
from compas_fab.robots.configuration import Configuration

from .conversions import frame_from_pose, pose_from_transformation, pose_from_frame
from .utils import values_as_list, LOGGER, wildcard_keys, is_poses_close

class PyChoreoClient(PyBulletClient):
    """Interface to use pybullet as backend via the **pybullet_plannning**.

    Parameters
    ----------
    viewer : :obj:`bool`
        Enable pybullet GUI. Defaults to True.

    """

    def __init__(self, viewer=True, verbose=False):
        from compas_fab_pychoreo.planner import PyChoreoPlanner
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
        # TODO remove temp dir even if key interrupted
        self._cache_dir = tempfile.TemporaryDirectory()

    def connect(self):
        with HideOutput(not self.verbose):
            super(PyChoreoClient, self).connect()
        # TODO, CLIENTS dict required by LockRender
        CLIENTS[self.client_id] = True if self.connection_type == 'gui' else None

    def get_robot_pybullet_uid(self, robot):
        pb_client = super(PyChoreoClient, self)
        return pb_client.get_uid(pb_client.get_cached_robot(robot))

    def load_robot(self, urdf_file, resource_loaders=None):
        # PyBulletClient's redirect_stdout does not seem to work...
        with HideOutput(not self.verbose):
            robot = super(PyChoreoClient, self).load_robot(urdf_file, resource_loaders)
        return robot

    ###########################################################

    def convert_mesh_to_body(self, mesh, frame, _name=None, concavity=False, mass=STATIC_MASS):
        """Convert compas mesh and its frame to a pybullet body.

        Parameters
        ----------
        mesh : :class:`compas.datastructures.Mesh`
        frame : :class:`compas.geometry.Frame`
        _name : :obj:`str`, optional
            Name of the mesh for tagging in PyBullet's GUI
        concavity : :obj:`bool`, optional
            When ``False`` (the default), the mesh will be loaded as its convex hull for collision checking purposes.
            When ``True``, a non-static mesh will be decomposed into convex parts using v-HACD.
        mass : :obj:`float`, optional
            Mass of the body to be created, in kg.  If ``0`` mass is given (the default),
            the object is static.

        Returns
        -------
        :obj:`int`

        Notes
        -----
        If this method is called several times with the same ``mesh`` instance, but the ``mesh`` has been modified
        in between calls, PyBullet's default caching behavior will prevent it from recognizing these changes.  It
        is best practice to create a new mesh instance or to make use of the `frame` argument, if applicable.  If
        this is not possible, PyBullet's caching behavior can be changed with
        ``pybullet.setPhysicsEngineParameter(enableFileCaching=0)``.
        """
        tmp_obj_path = os.path.join(self._cache_dir.name, '{}.obj'.format(mesh.guid))
        mesh.to_obj(tmp_obj_path)
        tmp_obj_path = self._handle_concavity(tmp_obj_path, self._cache_dir.name, concavity, mass)
        pyb_body_id = self.body_from_obj(tmp_obj_path, concavity=concavity, mass=mass)
        self._set_base_frame(frame, pyb_body_id)
        # need to keep this information since pybullet does not store file_path when parsing from obj file
        # but it does store filepath when parsing from a URDF
        INFO_FROM_BODY[self.client_id, pyb_body_id] = ModelInfo(name=None, path=tmp_obj_path, fixed_base=False, scale=1.0)
        return pyb_body_id

    def add_tool_from_urdf(self, tool_name, urdf_file):
        # TODO return compas_fab Tool class
        # TODO we might need to keep track of tools as robots too
        with HideOutput(not self.verbose):
            tool_robot = load_pybullet(urdf_file, fixed_base=False)
        self.collision_objects[tool_name] = [tool_robot]
        # INFO_FROM_BODY[self.client.client_id, tool_robot] = ModelInfo(tool_name, urdf_file, False, 1.0)

    ########################################
    # Collision / Attached pybullet body managment
    def detach_attached_collision_mesh(self, name, options=None):
        self.planner.detach_attached_collision_mesh(name, options=options)

    def get_object_names_and_status(self, wildcard):
        status = self._get_body_status(wildcard)
        if status == 'collision_object':
            names = self._get_collision_object_names(wildcard)
        elif status == 'attached_object':
            names = self._get_attachment_names(wildcard)
        else:
            names = []
        return names, status

    def _get_collision_object_names(self, wildcard):
        return wildcard_keys(self.collision_objects, wildcard)

    def _get_collision_object_bodies(self, wildcard):
        bodies = []
        for n in self._get_collision_object_names(wildcard):
            bodies.extend(self.collision_objects[n])
        return bodies

    def _get_attachment_names(self, wildcard):
        return wildcard_keys(self.pychoreo_attachments, wildcard)

    def _get_attached_bodies(self, wildcard):
        bodies = []
        for n in self._get_attachment_names(wildcard):
            bodies.extend([at.child for at in self.pychoreo_attachments[n]])
        return bodies

    def _get_body_status(self, wildcard):
        co_names = wildcard_keys(self.collision_objects, wildcard)
        at_names = wildcard_keys(self.pychoreo_attachments, wildcard)
        if len(co_names) == 0 and len(at_names) == 0:
            return 'not_exist'
        elif len(co_names) > 0 and len(at_names) == 0:
            return 'collision_object'
        elif len(co_names) == 0 and len(at_names) > 0:
            return 'attached_object'
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
        options = options or {}
        color = options.get('color', None)
        bodies = self._get_bodies(wildcard)
        for body in bodies:
            set_pose(body, pose_from_frame(frame))
            if color:
                links = get_all_links(body)
                for link in links:
                    set_color(body, color, link=link)

    def get_object_frame(self, wildcard, scale=1.0):
        bodies = self._get_bodies(wildcard)
        body_frames = {}
        for body in bodies:
            body_frames[body] = frame_from_pose(get_pose(body), scale)
        return body_frames

    def _print_object_summary(self):
        LOGGER.info('^'*10)
        LOGGER.info('PychoreoClient scene summary:')
        body_name_from_id = self._name_from_body_id
        LOGGER.info('Collision Objects:')
        LOGGER.info('\t{}'.format(['{}: {}'.format(name, bodies) for name, bodies in self.collision_objects.items()]))
        LOGGER.info('Attachments:')
        for name, attachments in self.pychoreo_attachments.items():
            LOGGER.info('\t{}: {}'.format(name, [at.child for at in attachments]))
        LOGGER.info('Extra disabled collision links:')
        for name, blink_pairs in self.extra_disabled_collision_links.items():
            LOGGER.info('\t{}:'.format(name))
            for (b1,l1_name), (b2,l2_name) in blink_pairs:
                b1_name = body_name_from_id[b1] if b1 in body_name_from_id else get_name(b1)
                b2_name = body_name_from_id[b2] if b2 in body_name_from_id else get_name(b2)
                LOGGER.info('\t\t({}-{}), ({}-{})'.format(b1_name,l1_name,b2_name,l2_name))

    ########################################

    def set_robot_configuration(self, robot, configuration, group=None): #, group=None
        # We enforce that `joint_names` attribute must be specified in `configuration`
        # ! YJ: I don't like the assumption that the "unspecified" joints are assumed to be zero
        # I think it's better to leave them "as-is"
        # https://github.com/compas-dev/compas_fab/blob/master/src/compas_fab/backends/pybullet/client.py#L402
        # super(PyChoreoClient, self).set_robot_configuration(robot, configuration, group=group)
        robot_uid = self.get_robot_pybullet_uid(robot)
        # TODO if joint_names are specified within configuration, take intersection
        # group_joint_names = robot.get_configurable_joint_names(group=group)
        self._set_body_configuration(robot_uid, configuration)
        for attachments in self.pychoreo_attachments.values():
            for attachment in attachments:
                attachment.assign()

    def _set_body_configuration(self, body_id, configuration):
        joints = joints_from_names(body_id, configuration.joint_names)
        set_joint_positions(body_id, joints, configuration.joint_values)

    def get_robot_configuration(self, robot, group):
        robot_uid = self.get_robot_pybullet_uid(robot)
        joint_names = robot.get_configurable_joint_names(group=group)
        joints = joints_from_names(robot_uid, joint_names)
        joint_types = robot.get_joint_types_by_names(joint_names)
        joint_values = get_joint_positions(robot_uid, joints)
        return Configuration(joint_values=joint_values, joint_types=joint_types, joint_names=joint_names)

    def get_link_frame_from_name(self, robot, link_name):
        robot_uid = self.get_robot_pybullet_uid(robot)
        return frame_from_pose(get_link_pose(robot_uid, link_from_name(robot_uid, link_name)))

    ###########################################################

    def check_collisions(self, *args, **kwargs):
        """check collisions between the robot at the given configuration and all the existing obstacles in the scene.

        the collision is checked among:
            1. robot self-collision (if `self_collisions=true`), ignored robot link pairs can be specified in `disabled_collisions`
            2. between (robot links) and (attached objects)
            3. between (robot links, attached objects) and obstacles
        ignored collisions for (2) and (3) can be specified in `extra_disabled_collisions`.

        ! note that:
            - collisions among attached objects are not checked
        """
        return self.planner.check_collisions(*args, **kwargs)

    def check_sweeping_collisions(self, *args, **kwargs):
        """Check collisions between the sweeping polylines of attached objects' vertices and the obstacles in the scene.
        No Robot links are checked here!
        """
        return self.planner.check_sweeping_collisions(*args, **kwargs)

    def check_attachment_collisions(self, options=None):
        """check collisions among the list of attached objects and obstacles in the scene.

        This includes collisions between:
            - each pair of (attached object, obstacle)
            - each pair of (attached object 1, attached object 2)
        """
        options = options or {}
        diagnosis = options.get('diagnosis', False)
        collision_distance_threshold = options.get('collision_distance_threshold', 0.0)
        max_distance = options.get('collision_buffer_distance_threshold', 0.0)

        # current status of the scene
        obstacles, attachments, extra_disabled_collisions = self._get_collision_checking_setup(options)
        # get all pybullet bodies of the attachment
        attached_bodies = [att.child for att in attachments]

        return _check_bodies_collisions(attached_bodies, obstacles,
            extra_disabled_collisions=extra_disabled_collisions, diagnosis=diagnosis, body_name_from_id=self._name_from_body_id,
            distance_threshold=collision_distance_threshold, max_distance=max_distance)

    def _get_collision_checking_setup(self, options=None):
        avoid_collisions = options.get('avoid_collisions', True)
        obstacles = []
        attachments = []
        extra_disabled_collisions = set()
        if avoid_collisions:
            wildcards = options.get('collision_object_wildcards') or None
            if wildcards is None:
                # consider all of them
                obstacles = values_as_list(self.collision_objects)
            else:
                obstacles = []
                for wc in wildcards:
                    names = wildcard_keys(self.collision_objects, wc)
                    for n in names:
                        obstacles.extend(self.collision_objects[n])
            # ! doesn't make sense to have a wildcard selection for attached objects
            attachments = values_as_list(self.pychoreo_attachments)

            # TODO additional disabled collisions in options
            extra_disabled_collision_names = values_as_list(self.extra_disabled_collision_links)
            option_disabled_link_names = options.get('extra_disabled_collisions', set())
            extra_disabled_collisions = set()
            for bpair in list(extra_disabled_collision_names) + list(option_disabled_link_names):
                b1, b1link_name = bpair[0]
                b2, b2link_name = bpair[1]
                b1_links = get_all_links(b1) if b1link_name is None else [link_from_name(b1, b1link_name)]
                b2_links = get_all_links(b2) if b2link_name is None else [link_from_name(b2, b2link_name)]
                for b1_link, b2_link in product(b1_links, b2_links):
                    extra_disabled_collisions.add(
                        ((b1, b1_link), (b2, b2_link))
                        )
        else:
            # only check joint limits, no collision considered
            pass
        return obstacles, attachments, extra_disabled_collisions

    ########################################
    # ignored collision info (similar to ROS's Allowed Collision Matrix)
    def get_self_collision_link_ids(self, robot):
        # return set of 2-int pair
        if robot.semantics is not None:
            robot_uid = self.get_robot_pybullet_uid(robot)
            self_collision_disabled_link_names = robot.semantics.disabled_collisions
            return get_disabled_collisions(robot_uid, self_collision_disabled_link_names)
        else:
            return {}

######################################

def _check_bodies_collisions(moving_bodies : list, obstacles : list,
        extra_disabled_collisions=None, diagnosis=False, body_name_from_id=None, **kwargs) -> bool:
    """check collisions among a list of bodies (`moving_bodies`) with a list of obstacles.

    This includes collisions between:
        - each pair of (moving_body, obstacle)
        - each pair of (moving_body1, moving_body2)

    where `moving_body` in `moving_bodies`, `obstacle` in `obstacles`
    """
    extra_disabled_collisions = extra_disabled_collisions or None
    # * converting body pairs to (body,link) pairs
    check_body_pairs = list(product(moving_bodies, obstacles)) + list(combinations(moving_bodies, 2))
    check_body_link_pairs = []
    for body1, body2 in check_body_pairs:
        body1, links1 = expand_links(body1)
        body2, links2 = expand_links(body2)
        if body1 == body2:
            continue
        bb_link_pairs = product(links1, links2)
        for bb_links in bb_link_pairs:
            bbll_pair = ((body1, bb_links[0]), (body2, bb_links[1]))
            if bbll_pair not in extra_disabled_collisions and bbll_pair[::-1] not in extra_disabled_collisions:
                check_body_link_pairs.append(bbll_pair)
    # * body - body check
    for (body1, link1), (body2, link2) in check_body_link_pairs:
        if pairwise_link_collision(body1, link1, body2, link2, **kwargs):
            if diagnosis:
                LOGGER.debug('check_attachment_collisions:')
                cr = pairwise_link_collision_info(body1, link1, body2, link2)
                draw_collision_diagnosis(cr, body_name_from_id=body_name_from_id)
            return True
    return False

