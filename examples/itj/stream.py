from termcolor import cprint
from copy import copy

from compas.geometry import Frame
from compas_fab.robots import Configuration, JointTrajectoryPoint, JointTrajectory
from compas_fab.robots import CollisionMesh, Duration

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement

import ikfast_abb_irb4600_40_255
from pybullet_planning import GREY
from pybullet_planning import link_from_name, get_link_pose, draw_pose, multiply, Pose, Euler, set_joint_positions, \
    joints_from_names, LockRenderer, WorldSaver, wait_for_user, joint_from_name
from pybullet_planning import get_sample_fn, link_from_name, sample_tool_ik, interpolate_poses, get_joint_positions
from pybullet_planning import plan_cartesian_motion, uniform_pose_generator

from compas_fab_pychoreo.backend_features.pychoreo_configuration_collision_checker import PyChoreoConfigurationCollisionChecker
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo_examples.ik_solver import InverseKinematicsSolver, get_ik_fn_from_ikfast
from compas_fab_pychoreo.utils import divide_list_chunks, values_as_list
from compas_fab_pychoreo.conversions import pose_from_frame

from .visualization import BEAM_COLOR, GRIPPER_COLOR, CLAMP_COLOR, TOOL_CHANGER_COLOR
from .robot_setup import R11_INTER_CONF_VALS, MAIN_ROBOT_ID, BARE_ARM_GROUP, GANTRY_ARM_GROUP, GANTRY_Z_LIMIT
from .robot_setup import get_gantry_control_joint_names, get_cartesian_control_joint_names, get_gantry_custom_limits

##############################

def set_state(client, robot, process, state_from_object, initialize=False, scale=1e-3):
    for object_id, object_state in state_from_object.items():
        if object_id.startswith('robot'):
            if object_state.kinematic_config is not None:
                client.set_robot_configuration(robot, object_state.kinematic_config)
        else:
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
            # * set pose according to state
            if object_state.current_frame is not None:
                current_frame = copy(object_state.current_frame)
                # assert current_frame is not None, 'object id : {} , state : {}'.format(object_id, object_state)
                current_frame.point *= scale
                client.set_collision_mesh_frame(object_id, current_frame,
                    options={'wildcard' : '{}_*'.format(object_id)})

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
    gantry_attempts = options.get('gantry_attempts') or 100
    pose_step_size = options.get('pose_step_size') or 0.01 # meter
    debug = options.get('debug') or False
    reachable_range = options.get('reachable_range') or (0., 1.)

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

    # TODO remove later, use client
    start_tool_pose = pose_from_frame(start_t0cf_frame, scale=1e-3)
    end_tool_pose = pose_from_frame(end_t0cf_frame, scale=1e-3)
    interp_poses = list(interpolate_poses(start_tool_pose, end_tool_pose, pos_step_size=pose_step_size))
    # if debug:
    for p in interp_poses:
        draw_pose(p)
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
        if arm_conf_vals and len(arm_conf_vals) > 0:
            wait_for_user('Conf found!')

        for arm_conf_val in arm_conf_vals:
            if arm_conf_val is None:
                continue
            gantry_arm_conf = Configuration(list(gantry_xyz_vals) + list(arm_conf_val),
                gantry_arm_joint_types, gantry_arm_joint_names)
            if not client.check_collisions(robot, gantry_arm_conf, options={'diagnosis':False}):
                # * set start pick conf
                with WorldSaver():
                    # * Cartesian planning, only for the six-axis arm (aka sub_conf)
                    # TODO replace with client one
                    # client.plan_cartesian_motion(robot, frames_WCF, start_configuration=None, group=None, options=None)
                    cart_conf_vals = plan_cartesian_motion(robot_uid, ik_joints[0], ik_tool_link, interp_poses, get_sub_conf=True)
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
    return traj

##############################

def compute_free_movement(client, robot, process, movement, options=None):
    assert isinstance(movement, RoboticLinearMovement)
    robot_uid = client.get_robot_pybullet_uid(robot)

    # * options
    # sampling attempts, needed only if start/end conf not specified
    debug = options.get('debug') or False
    # gantry_attempts = options.get('gantry_attempts') or 100
    # reachable_range = options.get('reachable_range') or (0., 1.)

    # * custom limits
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

    # TODO remove later, use client
    start_tool_pose = pose_from_frame(start_t0cf_frame, scale=1e-3)
    end_tool_pose = pose_from_frame(end_t0cf_frame, scale=1e-3)
    interp_poses = list(interpolate_poses(start_tool_pose, end_tool_pose, pos_step_size=pose_step_size))
    # if debug:
    for p in interp_poses:
        draw_pose(p)
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
        if arm_conf_vals and len(arm_conf_vals) > 0:
            wait_for_user('Conf found!')

        for arm_conf_val in arm_conf_vals:
            if arm_conf_val is None:
                continue
            gantry_arm_conf = Configuration(list(gantry_xyz_vals) + list(arm_conf_val),
                gantry_arm_joint_types, gantry_arm_joint_names)
            if not client.check_collisions(robot, gantry_arm_conf, options={'diagnosis':False}):
                # * set start pick conf
                with WorldSaver():
                    # * Cartesian planning, only for the six-axis arm (aka sub_conf)
                    # TODO replace with client one
                    cart_conf_vals = plan_cartesian_motion(robot_uid, ik_joints[0], ik_tool_link, interp_poses, get_sub_conf=True)
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
    return traj

