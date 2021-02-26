import sys
import pybullet
from termcolor import cprint
from copy import copy

from compas.geometry import Frame, distance_point_point
from compas_fab.robots import Configuration, JointTrajectoryPoint, JointTrajectory
from compas_fab.robots import CollisionMesh, Duration, AttachedCollisionMesh

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement
from integral_timber_joints.process.state import get_object_from_flange

import ikfast_abb_irb4600_40_255
from pybullet_planning import GREY
from pybullet_planning import link_from_name, get_link_pose, draw_pose, multiply, Pose, Euler, set_joint_positions, \
    joints_from_names, LockRenderer, WorldSaver, wait_for_user, joint_from_name, wait_if_gui
from pybullet_planning import get_sample_fn, link_from_name, sample_tool_ik, interpolate_poses, get_joint_positions
from pybullet_planning import plan_cartesian_motion, uniform_pose_generator

from compas_fab_pychoreo.backend_features.pychoreo_configuration_collision_checker import PyChoreoConfigurationCollisionChecker
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo_examples.ik_solver import InverseKinematicsSolver, get_ik_fn_from_ikfast
from compas_fab_pychoreo.utils import divide_list_chunks, values_as_list
from compas_fab_pychoreo.utils import wildcard_keys

from .visualization import BEAM_COLOR, GRIPPER_COLOR, CLAMP_COLOR, TOOL_CHANGER_COLOR
from .robot_setup import R11_INTER_CONF_VALS, MAIN_ROBOT_ID, BARE_ARM_GROUP, GANTRY_ARM_GROUP, GANTRY_Z_LIMIT
from .robot_setup import get_gantry_control_joint_names, get_cartesian_control_joint_names, get_gantry_custom_limits

##############################

def set_state(client, robot, process, state_from_object, initialize=False, scale=1e-3, options={}):
    gantry_attempts = options.get('gantry_attempts') or 20
    debug = options.get('debug') or False

    # robot needed for creating attachments
    robot_uid = client.get_robot_pybullet_uid(robot)
    flange_link_name = robot.get_end_effector_link_name(group=GANTRY_ARM_GROUP)
    sorted_gantry_joint_names = get_gantry_control_joint_names(MAIN_ROBOT_ID)
    gantry_arm_joint_types = robot.get_joint_types_by_names(sorted_gantry_joint_names)
    gantry_z_joint = joint_from_name(robot_uid, sorted_gantry_joint_names[2])
    gantry_z_sample_fn = get_sample_fn(robot_uid, [gantry_z_joint], custom_limits={gantry_z_joint : GANTRY_Z_LIMIT})

    with LockRenderer(not debug):
        # * Do robot first
        robot_state = state_from_object['robot']
        if robot_state.kinematic_config is not None:
            client.set_robot_configuration(robot, robot_state.kinematic_config)
            tool_link = link_from_name(robot_uid, flange_link_name)
            # in millimeter
            FK_tool_frame = frame_from_pose(get_link_pose(robot_uid, tool_link), scale=1/scale)
            # perform FK
            if robot_state.current_frame is None:
                robot_state.current_frame = FK_tool_frame
            else:
                # assert frame ~= current_frame
                tol = 1e-6
                if distance_point_point(robot_state.current_frame.point, FK_tool_frame.point) < tol \
                  and distance_point_point(robot_state.current_frame.xaxis, FK_tool_frame.xaxis) < tol \
                  and distance_point_point(robot_state.current_frame.yaxis, FK_tool_frame.yaxis) < tol:
                  raise ValueError('Robot FK tool pose and current frame diverge: {}'.format(distance_point_point(robot_state.current_frame.point, FK_tool_frame.point)))
            if initialize:
                # update tool_changer's current_frame
                # ! change if tool_changer has a non-trivial grasp pose
                state_from_object['tool_changer'].current_frame = FK_tool_frame

        for object_id, object_state in state_from_object.items():
            # print('====')
            # print('{} : {}'.format(object_id, object_state))
            if object_id.startswith('robot'):
                continue
            if initialize:
                # * create each object in the state dictionary
                # create objects in pybullet, convert mm to m
                color = GREY
                if object_id.startswith('b'):
                    beam = process.assembly.beam(object_id)
                    # ! notice that the notch geometry will be convexified in pybullet
                    meshes = [beam.mesh]
                    color = BEAM_COLOR
                elif object_id.startswith('c') or object_id.startswith('g'):
                    tool = process.tool(object_id)
                    tool.current_frame = Frame.worldXY()
                    if object_state.kinematic_config is not None:
                        # this might be in millimeter, but that's not related to pybullet's business (if we use static meshes)
                        tool._set_kinematic_state(object_state.kinematic_config)
                    meshes = tool.draw_visual()
                    if object_id.startswith('c'):
                        color = CLAMP_COLOR
                    elif object_id.startswith('g'):
                        color = GRIPPER_COLOR
                elif object_id.startswith('t'):
                    tool = process.robot_toolchanger
                    tool.current_frame = Frame.worldXY()
                    meshes = tool.draw_visual()
                    color = TOOL_CHANGER_COLOR
                for i, m in enumerate(meshes):
                    cm = CollisionMesh(m, object_id + '_{}'.format(i))
                    cm.scale(scale)
                    # add mesh to environment at origin
                    client.add_collision_mesh(cm, {'color':color})
            # end if initialize

            if object_state.current_frame is not None:
                current_frame = copy(object_state.current_frame)
                current_frame.point *= scale
                # * set pose according to state
                client.set_object_frame(object_id, current_frame,
                    options={'wildcard' : '^{}'.format(object_id)})

            # * attachment management
            wildcard = '^{}'.format(object_id)
            # -1 if not attached and not collision object
            # 0 if collision object, 1 if attached
            status = client._get_body_statues(wildcard)
            if not object_state.attached_to_robot:
                # * demote attachedCM to collision_objects
                if status == 1:
                    client.detach_attached_collision_mesh(object_id, {'wildcard' : wildcard})
            else:
                # * promote to attached_collision_object if needed
                if status == 0:
                    # * attached collision objects haven't been added
                    assert initialize or state_from_object['robot'].current_frame is not None
                    # object_from_flange = get_object_from_flange(state_from_object, object_id)
                    flange_frame = copy(state_from_object['robot'].current_frame)
                    object_frame = copy(state_from_object[object_id].current_frame)
                    flange_frame.point *= scale
                    object_frame.point *= scale
                    flange_pose = pose_from_frame(flange_frame, scale=scale)
                    # * sample from a ball near the pose
                    base_gen_fn = uniform_pose_generator(robot_uid, flange_pose, reachable_range=(0.2,1.5))
                    for _ in range(gantry_attempts):
                        x, y, yaw = next(base_gen_fn)
                        # TODO a more formal gantry_base_from_world_base
                        y *= -1
                        z, = gantry_z_sample_fn()
                        gantry_xyz_vals = [x,y,z]
                        client.set_robot_configuration(robot, Configuration(gantry_xyz_vals, gantry_arm_joint_types, sorted_gantry_joint_names))

                        conf = client.inverse_kinematics(robot, flange_frame, group=GANTRY_ARM_GROUP, options={'avoid_collisions' : False})
                        if conf is not None:
                            break
                    else:
                        raise RuntimeError('no attach conf found for {} after {} attempts.'.format(object_state, gantry_attempts))

                    # ? do we need WorldSaver?
                    client.set_robot_configuration(robot, conf)

                    # * create attachments
                    wildcard = '^{}'.format(object_id)
                    names = client._get_collision_object_names(wildcard)
                    # ! ACM for attachments
                    touched_links = ['{}_tool0'.format(MAIN_ROBOT_ID), '{}_link_6'.format(MAIN_ROBOT_ID)] if object_id.startswith('t') else []
                    for name in names:
                        # a faked AttachedCM since we are not adding a new mesh, just promoting collision meshes to AttachedCMes
                        client.add_attached_collision_mesh(AttachedCollisionMesh(CollisionMesh(None, name),
                            flange_link_name, touch_links=touched_links), options={'robot' : robot})
            # end if attached_to_robot

##############################

def _get_sample_bare_arm_ik_fn(client, robot):
    robot_uid = client.get_robot_pybullet_uid(robot)
    ik_base_link_name = robot.get_base_link_name(group=BARE_ARM_GROUP)
    ik_joint_names = robot.get_configurable_joint_names(group=BARE_ARM_GROUP)
    # * joint indices in pybullet
    ik_base_link = link_from_name(robot_uid, ik_base_link_name)
    ik_joints = joints_from_names(robot_uid, ik_joint_names)
    ikfast_fn = ikfast_abb_irb4600_40_255.get_ik
    def get_sample_ik_fn(robot, ik_fn, robot_base_link, ik_joints, tool_from_root=None):
        def sample_ik_fn(world_from_tcp):
            if tool_from_root:
                world_from_tcp = multiply(world_from_tcp, tool_from_root)
            return sample_tool_ik(ik_fn, robot, ik_joints, world_from_tcp, robot_base_link, get_all=True)
        return sample_ik_fn
    sample_ik_fn = get_sample_ik_fn(robot_uid, ikfast_fn, ik_base_link, ik_joints)
    return sample_ik_fn

##############################

def compute_linear_movement(client, robot, process, movement, options=None):
    assert isinstance(movement, RoboticLinearMovement)
    robot_uid = client.get_robot_pybullet_uid(robot)

    # * options
    # sampling attempts
    gantry_attempts = options.get('gantry_attempts') or 200
    pose_step_size = options.get('pose_step_size') or 0.01 # meter
    reachable_range = options.get('reachable_range') or (0.2, 1.5)
    debug = options.get('debug') or False

    # * custom limits
    ik_joint_names = robot.get_configurable_joint_names(group=BARE_ARM_GROUP)
    tool_link_name = robot.get_end_effector_link_name(group=BARE_ARM_GROUP)
    # pb ids
    ik_joints = joints_from_names(robot_uid, ik_joint_names)
    ik_tool_link = link_from_name(robot_uid, tool_link_name)

    gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    gantry_arm_joint_types = robot.get_joint_types_by_names(gantry_arm_joint_names)
    # gantry_joint_names = list(set(gantry_arm_joint_names) - set(ik_joint_names))

    # * construct IK function
    sample_ik_fn = _get_sample_bare_arm_ik_fn(client, robot)
    # TODO switch to client IK
    # ikfast_fn = get_ik_fn_from_ikfast(ikfast_abb_irb4600_40_255.get_ik)
    # ik_solver = InverseKinematicsSolver(robot, move_group, ikfast_fn, base_frame, robotA_tool.frame)
    # client.planner.inverse_kinematics = ik_solver.inverse_kinematics_function()

    # * get target T0CF pose
    start_state = process.get_movement_start_state(movement)
    end_state = process.get_movement_end_state(movement)
    start_t0cf_frame = start_state['robot'].current_frame
    end_t0cf_frame = end_state['robot'].current_frame
    # TODO wait for Victor's change
    end_conf = end_state['robot'].kinematic_config
    # end_conf = None

    # TODO remove later, use client
    start_tool_pose = pose_from_frame(start_t0cf_frame, scale=1e-3)
    if end_conf is not None:
        client.set_robot_configuration(robot, end_conf)
        end_tool_pose = get_link_pose(robot_uid, ik_tool_link)
    else:
        end_tool_pose = pose_from_frame(end_t0cf_frame, scale=1e-3)
        # TODO print warning if en_tool_pose and current_frame is not close
    interp_poses = list(interpolate_poses(start_tool_pose, end_tool_pose, pos_step_size=pose_step_size))
    # if debug:
    # for p in interp_poses:
    #     draw_pose(p)
    # wait_for_user('Viz cartesian poses')

    custom_limit_from_names = get_gantry_custom_limits(MAIN_ROBOT_ID)
    # gantry_xyz_sample_fn = get_sample_fn(robot_uid, gantry_joints,
    #     custom_limits={joint_from_name(robot_uid, jn) : limits for jn, limits in custom_limit_from_names.items()})
    sorted_gantry_joint_names = get_gantry_control_joint_names(MAIN_ROBOT_ID)
    gantry_joints = joints_from_names(robot_uid, sorted_gantry_joint_names)
    gantry_z_joint = joint_from_name(robot_uid, sorted_gantry_joint_names[2])
    gantry_z_sample_fn = get_sample_fn(robot_uid, [gantry_z_joint], custom_limits={gantry_z_joint : GANTRY_Z_LIMIT})
    # * sample from a ball near the pose
    base_gen_fn = uniform_pose_generator(robot_uid, start_tool_pose, reachable_range=reachable_range)

    solution_found = False
    samples_cnt = ik_failures = path_failures = 0

    if end_conf is None:
        for _ in range(gantry_attempts):
            x, y, yaw = next(base_gen_fn)
            # TODO a more formal gantry_base_from_world_base
            y *= -1
            z, = gantry_z_sample_fn()
            gantry_xyz_vals = [x,y,z]
            # print('gantry_xyz_vals: ({})'.format(gantry_xyz_vals))
            # TODO swicth to client set_robot_configuration
            set_joint_positions(robot_uid, gantry_joints, gantry_xyz_vals)
            samples_cnt += 1

            # check if collision-free ik solution exist for the four Cartesian poses
            arm_conf_vals = sample_ik_fn(start_tool_pose)
            # if arm_conf_vals and len(arm_conf_vals) > 0:
            #     wait_for_user('Conf found!')

            for arm_conf_val in arm_conf_vals:
                if arm_conf_val is None:
                    continue
                gantry_arm_conf = Configuration(list(gantry_xyz_vals) + list(arm_conf_val),
                    gantry_arm_joint_types, gantry_arm_joint_names)
                # print('extra_disabled_collision_links: ', client.extra_disabled_collision_links)
                if not client.check_collisions(robot, gantry_arm_conf, options=options):
                    # * set start pick conf
                    with WorldSaver():
                        # * Cartesian planning, only for the six-axis arm (aka sub_conf)
                        # TODO replace with client one
                        # client.plan_cartesian_motion(robot, frames_WCF, start_configuration=None, group=None, options=None)
                        try:
                            cart_conf_vals = plan_cartesian_motion(robot_uid, ik_joints[0], ik_tool_link, interp_poses, get_sub_conf=True)
                        except pybullet.error as e:
                            cart_conf_vals = None
                            # cprint(e, 'red')
                            # print('{} | {} | {}'.format(ik_joint_names, tool_link_name, interp_poses))
                        if cart_conf_vals is not None:
                            solution_found = True
                            cprint('Collision free! After {} ik, {} path failure over {} samples.'.format(
                                ik_failures, path_failures, samples_cnt), 'green')
                            break
                        else:
                            path_failures += 1
            else:
                ik_failures += 1
            if solution_found:
                break
        else:
            cprint('Cartesian Path planning failure after {} attempts | {} due to IK, {} due to Cart.'.format(
                samples_cnt, ik_failures, path_failures), 'yellow')
    else:
        gantry_xyz_vals = end_conf.prismatic_values
        cart_conf_vals = plan_cartesian_motion(robot_uid, ik_joints[0], ik_tool_link, interp_poses, get_sub_conf=True)
        if cart_conf_vals is not None:
            solution_found = True
            cprint('Collision free! After {} ik, {} path failure over {} samples.'.format(
                ik_failures, path_failures, samples_cnt), 'green')
        else:
            cprint('Cartesian Path planning failure with prescribed end conf.', 'yellow')

    traj = None
    if solution_found:
        # * translate to compas_fab Trajectory
        jt_traj_pts = []
        for i, arm_conf_val in enumerate(cart_conf_vals):
            jt_traj_pt = JointTrajectoryPoint(values=list(gantry_xyz_vals) + list(arm_conf_val),
                types=gantry_arm_joint_types, time_from_start=Duration(i*1,0))
            jt_traj_pt.joint_names = gantry_arm_joint_names
            jt_traj_pts.append(jt_traj_pt)
        traj = JointTrajectory(trajectory_points=jt_traj_pts, \
            joint_names=gantry_arm_joint_names, start_configuration=jt_traj_pts[0], fraction=1.0)
    else:
        cprint('No linear movement found for {}.'.format(movement), 'red')
    return traj

##############################

def compute_free_movement(client, robot, process, movement, options=None):
    assert isinstance(movement, RoboticFreeMovement)
    # robot_uid = client.get_robot_pybullet_uid(robot)

    # * options
    # sampling attempts, needed only if start/end conf not specified
    debug = options.get('debug') or False

    start_state = process.get_movement_start_state(movement)
    end_state = process.get_movement_end_state(movement)
    start_conf = start_state['robot'].kinematic_config
    end_conf = end_state['robot'].kinematic_config
    print('end conf: ', end_conf)
    print('end conf joint_names: ', end_conf.joint_names)
    initial_conf = process.initial_state['robot'].kinematic_config
    if start_conf is None and end_conf is None:
        # * sample from t0cp if no conf is provided for the robot
        cprint('No robot start/end conf is specified in {}, performing random IK solves.'.format(movement), 'yellow')
        start_t0cf_frame = start_state['robot'].current_frame
        end_t0cf_frame = end_state['robot'].current_frame
        if start_t0cf_frame is not None:
            start_conf = client.inverse_kinematics(robot, start_t0cf_frame, group=GANTRY_ARM_GROUP, options=options)
        else:
            if initial_conf is not None:
                cprint('No robot start frame is specified in {}, using initial conf.'.format(movement), 'yellow')
                start_conf = initial_conf
            else:
                cprint('Underspecified problem, solve fails.', 'red')
                return None
        if end_t0cf_frame is not None:
            end_conf = client.inverse_kinematics(robot, end_t0cf_frame, group=GANTRY_ARM_GROUP, options=options)
        else:
            if initial_conf is not None:
                cprint('No robot end frame is specified in {}, using initial conf.'.format(movement), 'yellow')
                end_conf = initial_conf
            else:
                cprint('Underspecified problem, solve fails.', 'red')
                return None
        if start_conf is None or end_conf is None:
            cprint('IK solves for start and end conf fails.', 'red')
            return None

    # * custom limits
    # gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    # gantry_arm_joint_types = robot.get_joint_types_by_names(gantry_arm_joint_names)
    # gantry_joint_names = list(set(gantry_arm_joint_names) - set(ik_joint_names))
    custom_limits = get_gantry_custom_limits(MAIN_ROBOT_ID)
    if 'custom_limits' not in options:
        options.update({'custom_limits' : custom_limits})

    # return client.plan_motion(robot, end_conf, start_configuration=start_conf, group=GANTRY_ARM_GROUP, options=options)
    goal_constraints = robot.constraints_from_configuration(end_conf, [0.01], [0.01], group=GANTRY_ARM_GROUP)
    with LockRenderer():
        traj = client.plan_motion(robot, goal_constraints, start_configuration=start_conf, group=GANTRY_ARM_GROUP, options=options)
    return traj

