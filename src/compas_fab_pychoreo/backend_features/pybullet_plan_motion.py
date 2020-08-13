import time
from compas_fab.backends.interfaces import PlanMotion
from compas_fab.backends.interfaces import InverseKinematics

from pybullet_planning import is_connected, get_bodies, WorldSaver, get_collision_fn
from compas_fab_pychoreo.conversions import joint_values_from_configuration, joints_from_names

class PybulletPlanMotion(PlanMotion):
    def __init__(self, pb_client, inverse_kinematics):
        self.pb_client = pb_client
        self.robot_uid = robot_uid
        self.inverse_kinematics = inverse_kinematics

    def plan_motion(self, frames_WCF, start_configuration=None, group=None, options=None):
        """Calculates a cartesian motion path (linear in tool space).

        Parameters
        ----------
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

            - ``"base_link"``: (:obj:`str`) Name of the base link.
            - ``"ee_link"``: (:obj:`str`) Name of the end effector link.
            - ``"joint_names"``: (:obj:`list` of :obj:`str`) List containing joint names.
            - ``"joint_types"``: (:obj:`list` of :obj:`str`) List containing joint types.

            - ``"avoid_collisions"``: (:obj:`bool`, optional)
              Whether or not to avoid collisions. Defaults to ``True``.

            - ``"max_step"``: (:obj:`float`, optional) The approximate distance between the
              calculated points. (Defined in the robot's units.) Defaults to ``0.01``.
            - ``"jump_threshold"``: (:obj:`float`, optional)
              The maximum allowed distance of joint positions between consecutive
              points. If the distance is found to be above this threshold, the
              path computation fails. It must be specified in relation to max_step.
              If this threshold is ``0``, 'jumps' might occur, resulting in an invalid
              cartesian path. Defaults to :math:`\\pi / 2`.

              # TODO: JointConstraint

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.
        """
        assert is_connected()
        assert 'robot_uid' in options and 'inverse_kinematics' in options
        robot_uid = options['robot_uid']
        assert robot_uid in get_bodies()
        assert issubclass(type(options['inverse_kinematics']), InverseKinematics)

        # ik_joints, conf = joint_values_from_configuration(start_configuration, robot_uid)
        ik_joints = joints_from_names(robot_uid, options['joint_names'])

        # * build collision fn here
        # assume checking with all the collision objects saved in the pb_client
        obstacles = []
        for name in self.pb_client.collision_objects:
            for obstacle_body in self.pb_client.collision_objects[name]:
                obstacles.append(obstacle_body)
        collision_fn = get_collision_fn(robot_uid, ik_joints, obstacles=obstacles,
                                        # TODO : get these arguments
                                        attachments=[], self_collisions=False,
                                        #    disabled_collisions=disabled_collisions,
                                        #    extra_disabled_collisions=extra_disabled_collisions,
                                        custom_limits={})

        # * convert to a pb ik_fn

        with WorldSaver():
            # reset to start conf
            set_joint_positions(robot_uid, ik_joints, conf) # seed

            st_time = time.time()
            path, cost = plan_cartesian_motion_lg(robot, ik_joints, ee_poses, sample_ik_fn, collision_fn, \
                custom_vel_limits=vel_limits, ee_vel=ee_vel)
            print('Solving time: {}'.format(elapsed_time(st_time)))

            if path is None:
                cprint('No free motion found!', 'red')
                return None
            else:
                jt_traj_pts = []
                for i, conf in enumerate(path):
                    c_conf = Configuration(values=conf, types=self.joint_types, joint_names=self.joint_names)
                    c_conf.scale(1e3)
                    jt_traj_pt = JointTrajectoryPoint(values=c_conf.values, types=c_conf.types, time_from_start=Duration(i*1,0))
                    jt_traj_pts.append(jt_traj_pt)
                trajectory = JointTrajectory(trajectory_points=jt_traj_pts,
                    joint_names=self.joint_names, start_configuration=start_configuration, fraction=1.0)
