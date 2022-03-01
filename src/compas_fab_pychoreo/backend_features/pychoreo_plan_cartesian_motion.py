from compas_fab.backends.ros.fileserver_loader import LOGGER
import numpy as np
import time
import math
from pybullet_planning.interfaces.env_manager.user_io import wait_for_user
from pybullet_planning.interfaces.robots.body import dump_body
from termcolor import cprint
from compas_fab.robots import Configuration, JointTrajectory, JointTrajectoryPoint, Duration, robot
from compas.robots import Joint
from compas.geometry import distance_point_point
from compas_fab.backends.interfaces import PlanCartesianMotion
from compas_fab.backends.interfaces import InverseKinematics
from compas_fab.backends.pybullet.utils import redirect_stdout

from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo.utils import is_valid_option, values_as_list, is_configurations_close, is_poses_close, is_frames_close
from compas_fab_pychoreo.backend_features.pychoreo_configuration_collision_checker import PyChoreoConfigurationCollisionChecker
from pybullet_planning import WorldSaver, joints_from_names, joint_from_name, \
    link_from_name, interpolate_poses, plan_cartesian_motion_lg, get_labeled_configuration

from pybullet_planning import HideOutput, draw_pose
from pybullet_planning import all_between, get_custom_limits, get_movable_joints, set_joint_positions, get_link_pose, \
    get_link_subtree
from pybullet_planning import prune_fixed_joints, get_configuration
from pybullet_planning import inverse_kinematics_helper, clone_body, remove_body

def create_sub_robot(robot, first_joint, target_link):
    # TODO: create a class or generator for repeated use
    selected_links = get_link_subtree(robot, first_joint) # TODO: child_link_from_joint?
    selected_joints = prune_fixed_joints(robot, selected_links)
    assert(target_link in selected_links)
    sub_target_link = selected_links.index(target_link)
    sub_robot = clone_body(robot, links=selected_links, visual=False, collision=False) # TODO: joint limits
    assert len(selected_joints) == len(get_movable_joints(sub_robot))
    return sub_robot, selected_joints, sub_target_link

def plan_cartesian_motion_from_links(robot, selected_links, target_link, waypoint_poses,
        max_iterations=200, custom_limits={}, get_sub_conf=False, options=None):
    """Plan a cartesian path using iterative pybullet IK"""
    options = options or {}
    lower_limits, upper_limits = get_custom_limits(robot, get_movable_joints(robot), custom_limits)
    selected_movable_joints = prune_fixed_joints(robot, selected_links)
    assert(target_link in selected_links)
    selected_target_link = selected_links.index(target_link)
    sub_robot = clone_body(robot, links=selected_links, visual=False, collision=False) # TODO: joint limits
    sub_movable_joints = get_movable_joints(sub_robot)
    #null_space = get_null_space(robot, selected_movable_joints, custom_limits=custom_limits)
    null_space = None

    solutions = []
    for target_pose in waypoint_poses:
        for iteration in range(max_iterations):
            sub_kinematic_conf = inverse_kinematics_helper(sub_robot, selected_target_link, target_pose, null_space=null_space)
            if sub_kinematic_conf is None:
                remove_body(sub_robot)
                return None
                # continue
            set_joint_positions(sub_robot, sub_movable_joints, sub_kinematic_conf)
            # pos_tolerance=1e-3, ori_tolerance=1e-3*np.pi
            if is_poses_close(get_link_pose(sub_robot, selected_target_link), target_pose, options=options):
                set_joint_positions(robot, selected_movable_joints, sub_kinematic_conf)
                kinematic_conf = get_configuration(robot)
                if not all_between(lower_limits, kinematic_conf, upper_limits):
                    remove_body(sub_robot)
                    return None
                if not get_sub_conf:
                    solutions.append(kinematic_conf)
                else:
                    solutions.append(sub_kinematic_conf)
                break
        else:
            remove_body(sub_robot)
            return None
    remove_body(sub_robot)
    return solutions

def plan_cartesian_motion_with_customized_ik(robot_uid, ik_info, waypoint_poses,
        custom_limits={}, options=None, **kwargs):
    """Plan cartesian motion with a customized ik fn"""
    options = options or {}
    lower_limits, upper_limits = get_custom_limits(robot_uid, get_movable_joints(robot_uid), custom_limits)
    # selected_movable_joints = prune_fixed_joints(robot, selected_links)
    # assert(target_link in selected_links)
    # selected_target_link = selected_links.index(target_link)
    # sub_robot = clone_body(robot, links=selected_links, visual=False, collision=False) # TODO: joint limits
    # sub_movable_joints = get_movable_joints(sub_robot)
    #null_space = get_null_space(robot, selected_movable_joints, custom_limits=custom_limits)
    # null_space = None

    ik_joints = joints_from_names(robot_uid, ik_info.ik_joint_names)
    free_joints = joints_from_names(robot_uid, ik_info.free_joint_names)
    ee_link = link_from_name(robot_uid, ik_info.ee_link_name)

    solutions = []
    for i, target_pose in enumerate(waypoint_poses):
        for conf in ik_info.ik_fn(target_pose, **kwargs):
            if conf is None:
                LOGGER.debug('TracIK: #{}/{} waypoint IK fails due to unfound.'.format(i, len(waypoint_poses)))
                return None
                # continue
            set_joint_positions(robot_uid, ik_joints, conf)
            if is_poses_close(get_link_pose(robot_uid, ee_link), target_pose, options=options):
                kinematic_conf = get_configuration(robot_uid)
                if not all_between(lower_limits, kinematic_conf, upper_limits):
                    LOGGER.debug('Reject due to limit: {} \nL {}\nU {}'.format(kinematic_conf, lower_limits, upper_limits))
                    return None
                solutions.append(kinematic_conf)
                break
            else:
                # draw_pose(get_link_pose(robot_uid, ee_link), length=1.0)
                # draw_pose(target_pose, length=1.0)
                LOGGER.debug('Pose not close:\nIK {}\nGiven {}!'.format(get_link_pose(robot_uid, ee_link), target_pose))
        else:
            LOGGER.debug('TracIK: #{}/{} waypoint IK fails.'.format(i, len(waypoint_poses)))
            return None
    return solutions

#######################################################

class PyChoreoPlanCartesianMotion(PlanCartesianMotion):
    def __init__(self, client):
        self.client = client

    def plan_cartesian_motion(self, robot, frames_WCF, start_configuration=None, group=None, options=None):
        """Calculates a cartesian motion path (linear in tool space).

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which the motion path is being calculated.
        frames_WCF: list of :class:`compas.geometry.Frame`
            The frames through which the path is defined.
        start_configuration: :class:`Configuration`, optional
            The robot's full configuration, i.e. values for all configurable
            joints of the entire robot, at the starting position.
        group: str, optional
            The planning group used for calculation.
        options: dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

            - ``"diagnosis"``: (:obj:`bool`, optional)
              . Defaults to ``False``.

            - ``"avoid_collisions"``: (:obj:`bool`, optional)
              Whether or not to avoid collisions. Defaults to ``True``.

            - ``"planner_id"``: (:obj:`str`)
              The name of the algorithm used for path planning.
              Defaults to ``'IterativeIK'``.
              Available planners: ``'IterativeIK', 'LadderGraph'``

            - ``"max_step"``: (:obj:`float`, optional) The approximate distance between the
              calculated points. (Defined in the robot's units.) Defaults to ``0.01``.
            - ``"jump_threshold"``: (:obj:`float`, optional)
              The maximum allowed distance of joint positions between consecutive
              points. If the distance is found to be above this threshold, the
              path computation fails. It must be specified in relation to max_step.
              If this threshold is ``0``, 'jumps' might occur, resulting in an invalid
              cartesian path.
              Defaults to :math:`\\pi / 6` for revolute or continuous joints, 0.1 for prismatic joints.

              # TODO: JointConstraint

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.
        """
        robot_uid = self.client.get_robot_pybullet_uid(robot)

        # * convert link/joint names to pybullet indices
        base_link_name = robot.get_base_link_name(group=group)
        tool_link_name = robot.get_end_effector_link_name(group=group)
        tool_link = link_from_name(robot_uid, tool_link_name)

        joint_names = robot.get_configurable_joint_names(group=group)
        joint_types = robot.get_joint_types_by_names(joint_names)
        ik_joints = joints_from_names(robot_uid, joint_names)

        # * parse options
        verbose = options.get('verbose', True)
        diagnosis = options.get('diagnosis', False)
        avoid_collisions = options.get('avoid_collisions', True)
        pos_step_size = options.get('max_step', 0.01)
        planner_id = is_valid_option(options, 'planner_id', 'IterativeIK')
        joint_jump_tolerances = options.get('joint_jump_tolerances', {})
        joint_custom_limits = options.get('joint_custom_limits', {}) # {joint_name : (lower_limit, upper_limit)}

        pb_jump_tolerances = {joint_from_name(robot_uid, jt_name) : j_diff \
            for jt_name, j_diff in joint_jump_tolerances.items()}
        pb_custom_limits = {joint_from_name(robot_uid, joint_name) : lims \
            for joint_name, lims in joint_custom_limits.items()}

        # * Iterative IK options
        customized_ikinfo = options.get('customized_ikinfo', None)

        # * ladder graph options
        frame_variant_gen = is_valid_option(options, 'frame_variant_generator', None)
        # ik function used in the ladder graph search
        ik_function = options.get('ik_function', None)

        # * convert to poses and do workspace linear interpolation
        given_poses = [pose_from_frame(frame_WCF) for frame_WCF in frames_WCF]
        ee_poses = []
        for p1, p2 in zip(given_poses[:-1], given_poses[1:]):
            c_interp_poses = list(interpolate_poses(p1, p2, pos_step_size=pos_step_size))
            ee_poses.extend(c_interp_poses)
        if len(ee_poses) == 0:
            LOGGER.error('target interpolated workspace poses are empty.')
            return None

        # TODO try floating attachment only planning before full cartesian planning.keys())?

        # * build collision fn
        attachments = values_as_list(self.client.pychoreo_attachments)
        collision_fn = PyChoreoConfigurationCollisionChecker(self.client)._get_collision_fn(robot, joint_names, options=options)

        failure_reason = ''
        with WorldSaver():
            # * set to start conf
            if start_configuration is not None:
                self.client.set_robot_configuration(robot, start_configuration)
                # print('FK: ', get_link_pose(robot_uid, tool_link))
                # print('Start ee: ', ee_poses[0])
                # print('is_pose_close: ', is_pose_close(get_link_pose(robot_uid, tool_link), ee_poses[0],
                #     pos_tolerance=pos_tolerance, ori_tolerance=ori_tolerance))

            if planner_id == 'IterativeIK':
                selected_links = [link_from_name(robot_uid, l) for l in robot.get_link_names(group=group)]
                if customized_ikinfo is None:
                    path = plan_cartesian_motion_from_links(robot_uid, selected_links, tool_link,
                        ee_poses, custom_limits=pb_custom_limits, get_sub_conf=False, options=options)
                else:
                    assert tool_link_name == customized_ikinfo.ee_link_name
                    path = plan_cartesian_motion_with_customized_ik(robot_uid, customized_ikinfo, ee_poses,
                        custom_limits=pb_custom_limits, options=options)
                        # max_ik_attempts=max_ik_attempts, free_delta=free_delta, max_joint_distance=max_joint_distance)

                if path is None:
                    failure_reason = 'IK plan is not found.'
                # collision checking is not included in the default Cartesian planning
                if path is not None:
                    for i, conf_val in enumerate(path):
                        pruned_conf_val = self._prune_configuration(robot_uid, conf_val, joint_names)
                        if avoid_collisions:
                            for attachment in attachments:
                                attachment.assign()
                            if collision_fn(pruned_conf_val, diagnosis=diagnosis):
                                failure_reason = 'IK plan is found but collision violated'
                                path = None
                                break
                        path[i] = pruned_conf_val

                # TODO check joint threshold
            elif planner_id == 'LadderGraph':
                # get ik fn from client
                # collision checking is turned off because collision checking is handled inside LadderGraph planner
                ik_options = {'avoid_collisions' : False, 'return_all' : True}
                sample_ik_fn = ik_function or self._get_sample_ik_fn(robot, ik_options)

                # convert ee_variant_fn
                if frame_variant_gen is not None:
                    def sample_ee_fn(pose):
                        for v_frame in frame_variant_gen.generate_frame_variant(frame_from_pose(pose)):
                            yield pose_from_frame(v_frame)
                else:
                    sample_ee_fn = None

                path, cost = plan_cartesian_motion_lg(robot_uid, ik_joints, ee_poses, sample_ik_fn, collision_fn, \
                    jump_threshold=pb_jump_tolerances, sample_ee_fn=sample_ee_fn,
                    enforce_start_conf=start_configuration is not None)

                if verbose:
                    LOGGER.debug('Ladder graph cost: {}'.format(cost))
            else:
                LOGGER.error('Cartesian planner {} not implemented!', planner_id)
                return None

        if path is None:
            if verbose:
                LOGGER.debug('No Cartesian motion found, due to {}!'.format(failure_reason))
            return None
        else:
            # TODO start_conf might have different number of joints with the given group?
            start_traj_pt = None
            if start_configuration is not None:
                start_traj_pt = JointTrajectoryPoint(joint_values=start_configuration.joint_values,
                    joint_types=start_configuration.joint_types)
                start_traj_pt.joint_names = start_configuration.joint_names

            jt_traj_pts = []
            for i, (conf, target_pose) in enumerate(zip(path, ee_poses)):
                jt_traj_pt = JointTrajectoryPoint(joint_values=conf, joint_names=joint_names, joint_types=joint_types)
                if start_traj_pt is not None:
                    jtp = start_traj_pt.copy()
                    # ! TrajectoryPoint doesn't copy over joint_names...
                    jtp.joint_names = start_traj_pt.joint_names
                    jtp.merge(jt_traj_pt)
                    jt_traj_pt = jtp
                jt_traj_pt.time_from_start = Duration(i*1,0)

                # * check start_conf joint value agreement
                if i == 0 and start_configuration is not None and \
                    not is_configurations_close(start_configuration, jt_traj_pt, options=options, report_when_close=False):
                    # ! in case there is numerical error that causes the first computed conf differ from the given
                    # start conf, we add the start conf into the trajectory
                    conf_temp = start_traj_pt.copy()
                    conf_temp.merge(jt_traj_pt)
                    jt_traj_pts.append(conf_temp)
                    # LOGGER.error('plan_cartesian_motion: planned traj\'s first conf does not agree with the given start conf.')

                # * check FK and target frame agreement
                fk_frame = self.client.forward_kinematics(robot, jt_traj_pt, group=group, options={'link' : tool_link_name})
                # self.client.set_robot_configuration(robot, jt_traj_pt)
                # tool_link = link_from_name(robot_uid, tool_link_name)
                # fk_frame = frame_from_pose(get_link_pose(robot_uid, tool_link))
                target_frame = frame_from_pose(target_pose)
                if not is_frames_close(fk_frame, target_frame, options=options):
                    LOGGER.error('plan_cartesian_motion: Conf FK disagreement found, FK center point diff (m) (o, x, y): {} | tool link name {}!'.format(
                        list(distance_point_point(val_fk, val_fr) for val_fk, val_fr in zip(fk_frame, target_frame)),
                            tool_link_name))
                    return None

                jt_traj_pts.append(jt_traj_pt)

            # TODO check intermediate joint jump
            trajectory = JointTrajectory(trajectory_points=jt_traj_pts,
                joint_names=jt_traj_pts[0].joint_names, start_configuration=jt_traj_pts[0], fraction=1.0)
            return trajectory

    def _prune_configuration(self, robot_uid, conf_val, joint_names):
        conf_val_from_joint_name = get_labeled_configuration(robot_uid)
        pruned_conf = []
        for joint_name, joint_value in zip(conf_val_from_joint_name, conf_val):
            if joint_name in joint_names or joint_name.decode('UTF-8') in joint_names:
                pruned_conf.append(joint_value)
        return pruned_conf

    def _get_sample_ik_fn(self, robot, ik_options=None):
        LOGGER.debug('Ladder graph using client default IK solver.', 'yellow')
        ik_options = ik_options or {}
        def sample_ik_fn(pose):
            # pb pose -> list(conf values)
            # TODO run random seed here and return a list of confs
            configurations = self.client.inverse_kinematics(robot, frame_from_pose(pose), options=ik_options)
            return [configuration.joint_values for configuration in configurations if configuration is not None]
        return sample_ik_fn
