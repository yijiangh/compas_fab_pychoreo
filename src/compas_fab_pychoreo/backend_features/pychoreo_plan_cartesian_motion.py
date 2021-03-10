import time
import math
from termcolor import cprint
from compas_fab.robots import Configuration, JointTrajectory, JointTrajectoryPoint, Duration
from compas.robots import Joint
from compas_fab.backends.interfaces import PlanCartesianMotion
from compas_fab.backends.interfaces import InverseKinematics

from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo.utils import is_valid_option, values_as_list
from compas_fab_pychoreo.backend_features.pychoreo_configuration_collision_checker import PyChoreoConfigurationCollisionChecker
from pybullet_planning import WorldSaver, joints_from_names, plan_cartesian_motion, \
    link_from_name, interpolate_poses, plan_cartesian_motion_lg, get_labeled_configuration
from pybullet_planning import wait_for_user

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
              cartesian path. Defaults to :math:`\\pi / 2`.

              # TODO: jump_threshold
              # TODO: JointConstraint

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.
        """
        robot_uid = robot.attributes['pybullet_uid']

        # * convert link/joint names to pybullet indices
        base_link_name = robot.get_base_link_name(group=group)
        tool_link_name = robot.get_end_effector_link_name(group=group)
        tool_link = link_from_name(robot_uid, tool_link_name)
        base_link = link_from_name(robot_uid, base_link_name)

        joint_names = robot.get_configurable_joint_names(group=group)
        ik_joints = joints_from_names(robot_uid, joint_names)
        joint_types = robot.get_joint_types_by_names(joint_names)

        # * parse options
        diagnosis = is_valid_option(options, 'diagnosis', False)
        avoid_collisions = options.get('avoid_collisions', True)
        pos_step_size = is_valid_option(options, 'max_step', 0.01)
        jump_threshold = is_valid_option(options, 'jump_threshold', {jt : math.pi/3 if jt_type == Joint.REVOLUTE else 0.1 \
            for jt, jt_type in zip(ik_joints, joint_types)})
        planner_id = is_valid_option(options, 'planner_id', 'IterativeIK')
        frame_variant_gen = is_valid_option(options, 'frame_variant_generator', None)

        # * convert to poses and do workspace linear interpolation
        given_poses = [pose_from_frame(frame_WCF) for frame_WCF in frames_WCF]
        ee_poses = []
        for p1, p2 in zip(given_poses[:-1], given_poses[1:]):
            c_interp_poses = list(interpolate_poses(p1, p2, pos_step_size=pos_step_size))
            ee_poses.extend(c_interp_poses)

        # * build collision fn
        attachments = values_as_list(self.client.pychoreo_attachments)
        collision_fn = PyChoreoConfigurationCollisionChecker(self.client)._get_collision_fn(robot, joint_names, options=options)

        # TODO separate attachment and robot body collision checking
        # First check attachment & env (here we can give users fine-grained control)
        # path planning
        # check attachment and robot

        with WorldSaver():
            # set to start conf
            if start_configuration is not None:
                self.client.set_robot_configuration(robot, start_configuration)

            if planner_id == 'IterativeIK':
                path = plan_cartesian_motion(robot_uid, base_link, tool_link, ee_poses, get_sub_conf=True)
                # collision checking is not included in the default Cartesian planning
                if path is not None and avoid_collisions:
                    for i, conf_val in enumerate(path):
                        pruned_conf_val = self._prune_configuration(robot_uid, conf_val, joint_names)
                        for attachment in attachments:
                            attachment.assign()
                        if collision_fn(pruned_conf_val, diagnosis=diagnosis):
                            path = None
                            break
                        path[i] = pruned_conf_val

                # TODO check joint threshold
            elif planner_id == 'LadderGraph':
                # get ik fn from client
                # collision checking is turned off because collision checking is handled inside LadderGraph planner
                ik_options = {'avoid_collisions' : False, 'return_all' : True}
                def sample_ik_fn(pose):
                    configurations = self.client.inverse_kinematics(robot, frame_from_pose(pose), options=ik_options)
                    return [configuration.values for configuration in configurations if configuration is not None]

                # convert ee_variant_fn
                if frame_variant_gen is not None:
                    def sample_ee_fn(pose):
                        for v_frame in frame_variant_gen.generate_frame_variant(frame_from_pose(pose)):
                            yield pose_from_frame(v_frame)
                else:
                    sample_ee_fn = None

                path, cost = plan_cartesian_motion_lg(robot_uid, ik_joints, ee_poses, sample_ik_fn, collision_fn, \
                    jump_threshold=jump_threshold, sample_ee_fn=sample_ee_fn)

                print('Ladder graph cost: {}'.format(cost))
            else:
                raise ValueError('Cartesian planner {} not implemented!', planner_id)

        if path is None:
            cprint('No Cartesian motion found!', 'red')
            return None
        else:
            jt_traj_pts = []
            for i, conf in enumerate(path):
                # c_conf = Configuration(values=conf, types=joint_types, joint_names=joint_names)
                jt_traj_pt = JointTrajectoryPoint(values=conf, types=joint_types, time_from_start=Duration(i*1,0))
                jt_traj_pt.joint_names = joint_names
                jt_traj_pts.append(jt_traj_pt)
            if start_configuration is not None and not start_configuration.close_to(jt_traj_pts[0]):
                print()
                cprint('No plan found due to joint jump from start conf, max diff {} | start conf {}, traj 0 {}'.format(
                    start_configuration.max_difference(jt_traj_pts[0]), start_configuration.values, jt_traj_pts[0].values), 'red')
                return None
            # TODO check intermediate joint jump
            trajectory = JointTrajectory(trajectory_points=jt_traj_pts,
                joint_names=joint_names, start_configuration=jt_traj_pts[0], fraction=1.0)
            return trajectory

    def _prune_configuration(self, robot_uid, conf_val, joint_names):
        conf_val_from_joint_name = get_labeled_configuration(robot_uid)
        pruned_conf = []
        for joint_name, joint_value in zip(conf_val_from_joint_name, conf_val):
            if joint_name in joint_names or joint_name.decode('UTF-8') in joint_names:
                pruned_conf.append(joint_value)
        return pruned_conf
