import pybullet_planning as pp
from compas_fab.backends.interfaces import AddAttachedCollisionMesh
from compas_fab.backends.pybullet.const import ConstraintInfo
from ..conversions import pose_from_transformation
from ..utils import is_poses_close

class PyChoreoAddAttachedCollisionMesh(AddAttachedCollisionMesh):
    """Callable to add a collision mesh and attach it to the robot."""
    def __init__(self, client):
        self.client = client

    def add_attached_collision_mesh(self, attached_collision_mesh, options=None):
        """Adds an attached collision object to the planning scene.

        If no grasp pose is passed in, by default the grasp is set according to the
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
        mass = options.get('mass', pp.STATIC_MASS)
        options['mass'] = mass
        color = options.get('color', None)
        attached_child_link_name = options.get('attached_child_link_name', None)
        parent_link_from_child_link = options.get('parent_link_from_child_link_transformation', None)

        robot_uid = self.client.get_robot_pybullet_uid(robot)
        name = attached_collision_mesh.collision_mesh.id
        attached_bodies = []
        # ! mimic ROS' behavior: collision object with same name is replaced
        if name in self.client.attached_collision_objects:
            # cprint('Replacing existing attached collision mesh {}'.format(name), 'yellow')
            self.client.detach_attached_collision_mesh(name, options=options)
            # self.remove_attached_collision_mesh(name, options=options)
        if name not in self.client.collision_objects:
            # ! I don't want to add another copy of the objects
            # self.planner.add_attached_collision_mesh(attached_collision_mesh, options=options)
            # attached_bodies = [constr.body_id for constr in self.attached_collision_objects[name]]
            self.client.planner.add_collision_mesh(attached_collision_mesh.collision_mesh, options=options)

        attached_bodies = self.client.collision_objects[name]
        del self.client.collision_objects[name]
        self.client.attached_collision_objects[name] = []

        tool_attach_link = pp.link_from_name(robot_uid, attached_collision_mesh.link_name)
        for body in attached_bodies:
            # TODO let user choose to include the direct touch link collision or not
            # * update attachment collision links
            for touched_link_name in attached_collision_mesh.touch_links:
                self.client.extra_disabled_collision_links[name].add(
                    ((robot_uid, touched_link_name), (body, None))
                    )
            links = pp.get_all_links(body)
            if color:
                for link in links:
                    pp.set_color(body, color, link=link)
            attach_child_link = pp.BASE_LINK if not attached_child_link_name else pp.link_from_name(body, attached_child_link_name)
            if not parent_link_from_child_link:
                # * create attachment based on their *current* pose
                attachment = pp.create_attachment(robot_uid, tool_attach_link, body, attach_child_link)
            else:
                # * create attachment based on a given grasp transformation
                grasp_pose = pose_from_transformation(parent_link_from_child_link)
                attachment = pp.Attachment(robot_uid, tool_attach_link, grasp_pose, body)

            parent_link_pose = pp.get_link_pose(robot_uid, tool_attach_link)
            attached_body_pose = pp.get_link_pose(body, attach_child_link)
            if is_poses_close(parent_link_pose, attached_body_pose, options=options):
                attachment.assign()
            # else:
            #     LOGGER.warning('Attaching {} (link {}) to robot link {}, but they are not in the same pose in pybullet scene.'.format(name, attached_child_link_name if attached_child_link_name else 'BASE_LINK',
            #                attached_collision_mesh.link_name))

            self.client.pychoreo_attachments[name].append(attachment)
            # create fixed constraint to conform to PybulletClient (we don't use it though)
            constraint_id = pp.add_fixed_constraint(attachment.child, attachment.parent, attachment.parent_link)
            constraint_info = ConstraintInfo(constraint_id, attachment.child, attachment.parent)
            self.client.attached_collision_objects[name].append(constraint_info)

        return self.client.pychoreo_attachments[name]
