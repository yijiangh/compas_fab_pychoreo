from compas_fab.backends.interfaces import InverseKinematics
from compas_fab.robots import Configuration

from pybullet_planning import all_between, is_pose_close
from pybullet_planning import inverse_kinematics_helper
from pybullet_planning import get_movable_joints, set_joint_positions, get_link_pose, get_custom_limits, joints_from_names, link_from_name, \
    WorldSaver, get_link_subtree, prune_fixed_joints, clone_body, remove_body, get_configuration, joint_from_name
from pybullet_planning import wait_if_gui

from compas_fab_pychoreo.conversions import pose_from_frame
from compas_fab_pychoreo.utils import is_valid_option

class PyChoreoInverseKinematics(InverseKinematics):
    def __init__(self, client):
        self.client = client

    def inverse_kinematics(self, robot, frame_WCF, start_configuration=None, group=None, options=None):
        """Calculate the robot's inverse kinematic for a given frame.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which inverse kinematics is being calculated.
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
            - ``"return_all"``: (bool, optional) return a list of all computed ik solutions
              Defaults to False
        Raises
        ------
        compas_fab.backends.exceptions.BackendError
            If no configuration can be found.
        Returns
        -------
        :class:`compas_fab.robots.Configuration`
            The planning group's configuration.
        """
        max_iterations = is_valid_option(options, 'attempts', 8)
        avoid_collisions = is_valid_option(options, 'avoid_collisions', True)
        return_all = is_valid_option(options, 'return_all', False)
        custom_limits = options.get('custom_limits') or {}
        # return_closest_to_start = is_valid_option(options, 'return_closest_to_start', False)
        # cull = is_valid_option(options, 'cull', True)

        robot_uid = robot.attributes['pybullet_uid']
        ik_joint_names = robot.get_configurable_joint_names(group=group)
        ik_joints = joints_from_names(robot_uid, ik_joint_names)
        tool_link_name = robot.get_end_effector_link_name(group=group)
        tool_link = link_from_name(robot_uid, tool_link_name)
        pb_custom_limits = get_custom_limits(robot_uid, ik_joints,
            custom_limits={joint_from_name(robot_uid, jn) : lims for jn, lims in custom_limits.items()})

        target_pose = pose_from_frame(frame_WCF)
        with WorldSaver():
            if start_configuration is not None:
                start_conf_vals = start_configuration.values
                set_joint_positions(robot_uid, ik_joints, start_conf_vals)
            # else:
            #     sample_fn = get_sample_fn(robot_uid, ik_joints)
            #     start_conf_vals = sample_fn()

            # if group not in self.client.planner.ik_fn_from_group:
            # use default ik fn
            conf_vals = self._compute_ik(robot_uid, ik_joints, tool_link, target_pose, max_iterations, custom_limits=pb_custom_limits)
            joint_types = robot.get_joint_types_by_names(ik_joint_names)
            configurations = [Configuration(values=conf_val, types=joint_types, joint_names=ik_joint_names) \
                for conf_val in conf_vals if conf_val is not None]
            # else:
            #     # qs = client.inverse_kinematics(frame_WCF, group=move_group)
            #     configurations = self.client.planner.ik_fn_from_group[group](frame_WCF, group=group, options=options)

            if avoid_collisions:
                configurations = [conf for conf in configurations if not self.client.check_collisions(robot, conf, options=options)]

        if return_all:
            return configurations
        else:
            return configurations[0] if len(configurations) > 0 else None

    def _compute_ik(self, robot_uid, ik_joints, tool_link, target_pose, max_iterations=8, custom_limits={}):
        lower_limits, upper_limits = get_custom_limits(robot_uid, get_movable_joints(robot_uid), custom_limits)
        selected_links = get_link_subtree(robot_uid, ik_joints[0]) # TODO: child_link_from_joint?
        selected_movable_joints = prune_fixed_joints(robot_uid, selected_links)
        assert(tool_link in selected_links)
        selected_target_link = selected_links.index(tool_link)
        sub_robot = clone_body(robot_uid, links=selected_links, visual=False, collision=False) # TODO: joint limits
        sub_movable_joints = get_movable_joints(sub_robot)

        for _ in range(max_iterations):
            # TODO: stop is no progress
            # TODO: stop if collision or invalid joint limits
            # kinematic_conf = inverse_kinematics_helper(robot_uid, tool_link, target_pose)
            sub_kinematic_conf = inverse_kinematics_helper(sub_robot, selected_target_link, target_pose)
            if sub_kinematic_conf is None:
                remove_body(sub_robot)
                return []
            set_joint_positions(sub_robot, sub_movable_joints, sub_kinematic_conf)
            if is_pose_close(get_link_pose(sub_robot, selected_target_link), target_pose): #, **kwargs):
                set_joint_positions(robot_uid, selected_movable_joints, sub_kinematic_conf)
                kinematic_conf = get_configuration(robot_uid)
                if not all_between(lower_limits, kinematic_conf, upper_limits):
                    remove_body(sub_robot)
                    return []
                break
        else:
            remove_body(sub_robot)
            return []
        remove_body(sub_robot)
        conf = [kinematic_conf[i] for i, jt in enumerate(selected_movable_joints) if jt in ik_joints]
        # TODO in RFL case, the base_link will give us 17 joint, needs some pruning according to the ik_joints
        return [conf]
