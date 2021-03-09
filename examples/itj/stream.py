import os, sys
import pybullet
from termcolor import cprint
from copy import copy, deepcopy
from itertools import product

from compas.geometry import Frame, distance_point_point
from compas_fab.robots import Configuration, JointTrajectoryPoint, JointTrajectory
from compas_fab.robots import CollisionMesh, Duration, AttachedCollisionMesh

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement
from integral_timber_joints.process.state import get_object_from_flange

import ikfast_abb_irb4600_40_255
from pybullet_planning import GREY
from pybullet_planning import link_from_name, get_link_pose, draw_pose, multiply, Pose, Euler, set_joint_positions, \
    joints_from_names, LockRenderer, WorldSaver, wait_for_user, joint_from_name, wait_if_gui, load_pybullet, HideOutput
from pybullet_planning import get_sample_fn, link_from_name, sample_tool_ik, interpolate_poses, get_joint_positions
from pybullet_planning import plan_cartesian_motion, uniform_pose_generator, dump_body

from compas_fab_pychoreo.backend_features.pychoreo_configuration_collision_checker import PyChoreoConfigurationCollisionChecker
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo_examples.ik_solver import InverseKinematicsSolver, get_ik_fn_from_ikfast
from compas_fab_pychoreo.utils import divide_list_chunks, values_as_list
from compas_fab_pychoreo.utils import wildcard_keys

from .visualization import BEAM_COLOR, GRIPPER_COLOR, CLAMP_COLOR, TOOL_CHANGER_COLOR
from .robot_setup import R11_INTER_CONF_VALS, MAIN_ROBOT_ID, BARE_ARM_GROUP, GANTRY_ARM_GROUP, GANTRY_Z_LIMIT
from .robot_setup import get_gantry_control_joint_names, get_cartesian_control_joint_names, get_gantry_robot_custom_limits
from .parsing import DATA_DIR
from .utils import reverse_trajectory

# in meter
FRAME_TOL = 1e-4

##############################

def set_state(client, robot, process, state_from_object, initialize=False, scale=1e-3, options=None):
    options = options or {}
    gantry_attempts = options.get('gantry_attempts') or 50
    debug = options.get('debug', False)
    include_env = options.get('include_env', True)
    reinit_tool = options.get('reinit_tool', False)

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
                if not robot_state.current_frame.__eq__(FK_tool_frame, tol=FRAME_TOL*1e3):
                  msg = 'Robot FK tool pose and current frame diverge: {:.3f} (mm)'.format(distance_point_point(robot_state.current_frame.point, FK_tool_frame.point))
                  cprint(msg, 'yellow')
            if initialize:
                # update tool_changer's current_frame
                # ! change if tool_changer has a non-trivial grasp pose
                state_from_object['tool_changer'].current_frame = FK_tool_frame

        # * environment meshes
        if initialize and include_env:
            for name, m in process.environment_models.items():
                cm = CollisionMesh(m, 'env_' + name)
                cm.scale(scale)
                client.add_collision_mesh(cm, {'color':GREY})

        for object_id, object_state in state_from_object.items():
            # print('====')
            # print('{} : {}'.format(object_id, object_state))
            if object_id.startswith('robot'):
                continue
            obj = process.get_object_from_id(object_id)
            if initialize:
                # * create each object in the state dictionary
                # create objects in pybullet, convert mm to m
                color = GREY
                if object_id.startswith('b'):
                    # ! notice that the notch geometry will be convexified in pybullet
                    color = BEAM_COLOR
                    cm = CollisionMesh(obj.mesh, object_id)
                    cm.scale(scale)
                    # add mesh to environment at origin
                    client.add_collision_mesh(cm, {'color':color})
                else:
                    urdf_path = obj.get_urdf_path(DATA_DIR)
                    if reinit_tool or not os.path.exists(urdf_path):
                        obj.save_as_urdf(DATA_DIR, scale=1e-3)
                        cprint('Tool {} ({}) URDF generated to {}'.format(object_id, obj.name, urdf_path), 'green')
                    with HideOutput():
                        tool_robot = load_pybullet(urdf_path, fixed_base=False)
                    client.collision_objects[object_id] = [tool_robot]
            # end if initialize

            if object_state.current_frame is not None:
                current_frame = copy(object_state.current_frame)
                current_frame.point *= scale
                # * set pose according to state
                client.set_object_frame('^{}'.format(object_id), current_frame)

            if object_state.kinematic_config is not None:
                assert object_id.startswith('c') or object_id.startswith('g')
                # this might be in millimeter, but that's not related to pybullet's business (if we use static meshes)
                obj._set_kinematic_state(object_state.kinematic_config)
                tool_conf = obj.current_configuration.scaled(1e-3)
                tool_bodies = client._get_bodies('^{}$'.format(object_id))
                for b in tool_bodies:
                    client._set_body_configuration(b, tool_conf)

            # * attachment management
            wildcard = '^{}'.format(object_id)
            # -1 if not attached and not collision object
            # 0 if collision object, 1 if attached
            status = client._get_body_status(wildcard)
            if not object_state.attached_to_robot:
                # * demote attachedCM to collision_objects
                if status == 1:
                    client.detach_attached_collision_mesh(object_id, {'wildcard' : wildcard})
            else:
                # * promote to attached_collision_object if needed
                # if status == 1:
                #     client.detach_attached_collision_mesh(object_id, {'wildcard' : wildcard})
                # status = client._get_body_status(wildcard)
                # assert status == 0
                if status == 0:
                    # * attached collision objects haven't been added
                    assert initialize or state_from_object['robot'].current_frame is not None
                    # object_from_flange = get_object_from_flange(state_from_object, object_id)
                    flange_frame = deepcopy(state_from_object['robot'].current_frame)
                    object_frame = deepcopy(state_from_object[object_id].current_frame)
                    flange_frame.point *= scale
                    object_frame.point *= scale
                    flange_pose = pose_from_frame(flange_frame, scale=1)

                    # print('flange frame: {} | object frame {} | flange pose {}'.format(flange_frame, object_frame, flange_pose))
                    # TODO wrap this base sampling + 6-axis IK into inverse_kinematics for the client
                    # * sample from a ball near the pose
                    base_gen_fn = uniform_pose_generator(robot_uid, flange_pose, reachable_range=(0.2,1.5))
                    for _ in range(gantry_attempts):
                        # TODO a more formal gantry_base_from_world_base
                        x, y, yaw = next(base_gen_fn)
                        y *= -1
                        z, = gantry_z_sample_fn()
                        gantry_xyz_vals = [x,y,z]
                        client.set_robot_configuration(robot, Configuration(gantry_xyz_vals, gantry_arm_joint_types, sorted_gantry_joint_names))
                        conf = client.inverse_kinematics(robot, flange_frame, group=GANTRY_ARM_GROUP,
                            options={'avoid_collisions' : False})
                        if conf is not None:
                            break
                    else:
                        raise RuntimeError('no attach conf found for {} after {} attempts.'.format(object_state, gantry_attempts))
                    client.set_robot_configuration(robot, conf)
                    # wait_if_gui('Conf set for attachment')

                    # * create attachments
                    wildcard = '^{}'.format(object_id)
                    names = client._get_collision_object_names(wildcard)
                    # touched_links is only for the adjacent Robot links
                    touched_links = ['{}_tool0'.format(MAIN_ROBOT_ID), '{}_link_6'.format(MAIN_ROBOT_ID)] \
                        if object_id.startswith('t') else []
                    # TODO auto derive from URDF link tree
                    attached_child_link_name = None
                    if object_id.startswith('t'):
                        attached_child_link_name = 'toolchanger_base'
                    elif object_id.startswith('g') or object_id.startswith('c'):
                        attached_child_link_name = 'gripper_base'
                    for name in names:
                        # a faked AttachedCM since we are not adding a new mesh, just promoting collision meshes to AttachedCMes
                        client.add_attached_collision_mesh(AttachedCollisionMesh(CollisionMesh(None, name),
                            flange_link_name, touch_links=touched_links), options=
                            {'robot' : robot, 'attached_child_link_name' : attached_child_link_name})

                    # * attachments disabled collisions
                    extra_disabled_bodies = []
                    if object_id.startswith('b'):
                        # gripper and beam
                        g_id = None
                        for o_id, o_st in state_from_object.items():
                            if o_id.startswith('g') and o_st.attached_to_robot:
                                g_id = o_id
                                break
                        assert g_id is not None, 'At least one gripper should be attached to the robot when the beam is attached.'
                        extra_disabled_bodies = client._get_bodies('^{}'.format(g_id))
                    elif object_id.startswith('c') or object_id.startswith('g'):
                        # tool_changer and gripper
                        extra_disabled_bodies = client._get_bodies('^{}'.format('tool_changer'))
                    for name in names:
                        at_bodies = client._get_attached_bodies('^{}$'.format(name))
                        assert len(at_bodies) > 0
                        for parent_body, child_body in product(extra_disabled_bodies, at_bodies):
                            client.extra_disabled_collision_links[name].add(
                                ((parent_body, None), (child_body, None))
                                )

                    # x, y, yaw = next(base_gen_fn)
                    # y *= -1
                    # z, = gantry_z_sample_fn()
                    # gantry_xyz_vals = [x,y,z]
                    # client.set_robot_configuration(robot, Configuration(gantry_xyz_vals, gantry_arm_joint_types, sorted_gantry_joint_names))
                    # wait_if_gui('Update conf set for attachment')

                # cprint('Set state: attached_to_robot: (previous status {})'.format(status), 'blue')
                # for name, attachments in client.pychoreo_attachments.items():
                #     print('attached name: ', name)
                #     for attachment in attachments:
                #         print('Grasp pose: {}'.format(attachment.grasp_pose))

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
    options = options or {}
    gantry_attempts = options.get('gantry_attempts') or 10
    reachable_range = options.get('reachable_range') or (0.2, 1.5)
    debug = options.get('debug', False)

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
    start_conf = start_state['robot'].kinematic_config
    end_conf = end_state['robot'].kinematic_config

    # * set start state
    set_state(client, robot, process, start_state)

    start_t0cf_frame = copy(start_state['robot'].current_frame)
    start_t0cf_frame.point *= 1e-3
    end_t0cf_frame = copy(end_state['robot'].current_frame)
    end_t0cf_frame.point *= 1e-3

    # TODO: ignore beam / env collision in the first pickup pose
    temp_name = '_tmp'
    for o1_name, o2_name in movement.allowed_collision_matrix:
        o1_bodies = client._get_bodies('^{}'.format(o1_name))
        o2_bodies = client._get_bodies('^{}'.format(o2_name))
        for parent_body, child_body in product(o1_bodies, o2_bodies):
            client.extra_disabled_collision_links[temp_name].add(
                ((parent_body, None), (child_body, None))
                )
    # print('tmp ignored: ', client.extra_disabled_collision_links[temp_name])

    with WorldSaver():
        if start_conf is not None:
            client.set_robot_configuration(robot, start_conf)
            start_tool_pose = get_link_pose(robot_uid, ik_tool_link)
            start_t0cf_frame_temp = frame_from_pose(start_tool_pose, scale=1)
            if not start_t0cf_frame_temp.__eq__(start_t0cf_frame, tol=FRAME_TOL):
                cprint('start conf FK inconsistent ({:.5f} m) with given current frame in start state.'.format(
                    distance_point_point(start_t0cf_frame_temp.point, start_t0cf_frame.point)), 'yellow')
            # start_t0cf_frame = start_t0cf_frame_temp
            # , overwriting with the FK one.
            # start_state['robot'].current_frame = start_t0cf_frame_temp
        if end_conf is not None:
            client.set_robot_configuration(robot, end_conf)
            end_tool_pose = get_link_pose(robot_uid, ik_tool_link)
            end_t0cf_frame_temp = frame_from_pose(end_tool_pose, scale=1)
            if not end_t0cf_frame_temp.__eq__(end_t0cf_frame, tol=FRAME_TOL):
                cprint('end conf FK inconsistent ({:.5f} m) with given current frame in end state.'.format(
                    distance_point_point(end_t0cf_frame_temp.point, end_t0cf_frame.point)), 'yellow')
            # end_t0cf_frame = end_t0cf_frame_temp
            # , overwriting with the FK one
            # end_state['robot'].current_frame = end_t0cf_frame_temp

    interp_frames = [start_t0cf_frame, end_t0cf_frame]

    sorted_gantry_joint_names = get_gantry_control_joint_names(MAIN_ROBOT_ID)
    gantry_joints = joints_from_names(robot_uid, sorted_gantry_joint_names)
    gantry_z_joint = joint_from_name(robot_uid, sorted_gantry_joint_names[2])
    gantry_z_sample_fn = get_sample_fn(robot_uid, [gantry_z_joint], custom_limits={gantry_z_joint : GANTRY_Z_LIMIT})

    solution_found = False
    samples_cnt = ik_failures = path_failures = 0

    # TODO custom limits

    if start_conf is None and end_conf is None:
        # * sample from a ball near the pose
        base_gen_fn = uniform_pose_generator(robot_uid, pose_from_frame(interp_frames[0], scale=1), reachable_range=reachable_range)
        for _ in range(gantry_attempts):
            x, y, yaw = next(base_gen_fn)
            # TODO a more formal gantry_base_from_world_base
            y *= -1
            z, = gantry_z_sample_fn()
            gantry_xyz_vals = [x,y,z]
            gantry_conf = Configuration(gantry_xyz_vals,
                    gantry_arm_joint_types[:3], gantry_arm_joint_names[:3])
            client.set_robot_configuration(robot, gantry_conf)
            samples_cnt += 1

            arm_conf_vals = sample_ik_fn(pose_from_frame(interp_frames[0], scale=1))
            # iterate through all 6-axis IK solution
            for arm_conf_val in arm_conf_vals:
                if arm_conf_val is None:
                    continue
                gantry_arm_conf = Configuration(list(gantry_xyz_vals) + list(arm_conf_val),
                    gantry_arm_joint_types, gantry_arm_joint_names)
                if not client.check_collisions(robot, gantry_arm_conf, options=options):
                    # * Cartesian planning, only for the six-axis arm (aka sub_conf)
                    # cart_conf_vals = plan_cartesian_motion(robot_uid, ik_joints[0], ik_tool_link, interp_poses, get_sub_conf=True)
                    for _ in range(5):
                        cart_conf = client.plan_cartesian_motion(robot, interp_frames, start_configuration=gantry_arm_conf,
                            group=GANTRY_ARM_GROUP, options=options)
                        if cart_conf is not None:
                            solution_found = True
                            cprint('Collision free! After {} ik, {} path failure over {} samples.'.format(
                                ik_failures, path_failures, samples_cnt), 'green')
                            break
                        else:
                            path_failures += 1
                if solution_found:
                    break
            else:
                ik_failures += 1
            if solution_found:
                break
        else:
            cprint('Cartesian Path planning failure after {} attempts | {} due to IK, {} due to Cart.'.format(
                samples_cnt, ik_failures, path_failures), 'yellow')
    else:
        # TODO make sure start/end conf coincides if provided
        if start_conf is not None and end_conf is not None:
            cprint('Both start/end confs are pre-specified, problem might be too stiff to be solved.', 'yellow')
        if start_conf:
            forward = True
            gantry_arm_conf = start_conf
        else:
            forward = False
            gantry_arm_conf = end_conf
            interp_frames = interp_frames[::-1]

        samples_cnt = 0
        for _ in range(5):
            cart_conf = client.plan_cartesian_motion(robot, interp_frames, start_configuration=gantry_arm_conf,
                group=GANTRY_ARM_GROUP, options=options)
            samples_cnt += 1
            if cart_conf is not None:
                solution_found = True
                cprint('Collision free! After {} path failure over {} samples.'.format(
                    path_failures, samples_cnt), 'green')
                if not forward:
                    cart_conf = reverse_trajectory(cart_conf)
                break
            else:
                path_failures += 1
        else:
            cprint('Cartesian Path planning (w/ prespecified st/end conf) failure after {} attempts.'.format(
                samples_cnt), 'yellow')

    if temp_name in client.extra_disabled_collision_links:
        del client.extra_disabled_collision_links[temp_name]

    traj = None
    if solution_found:
        traj = cart_conf
        if start_conf is not None and not start_conf.close_to(traj.points[0], tol=1e-3):
            cprint('Start conf not coincided - max diff {:.5f}'.format(start_conf.max_difference(traj.points[0])), 'red')
            wait_for_user()
        if end_conf is not None and not end_conf.close_to(traj.points[-1], tol=1e-3):
            cprint('End conf not coincided - max diff {:.5f}'.format(end_conf.max_difference(traj.points[-1])), 'red')
            wait_for_user()
    else:
        cprint('No linear movement found for {}.'.format(movement.short_summary), 'red')
    return traj

##############################

def compute_free_movement(client, robot, process, movement, options=None):
    assert isinstance(movement, RoboticFreeMovement)
    options = options or {}
    # * options
    # sampling attempts, needed only if start/end conf not specified
    debug = options.get('debug', False)

    start_state = process.get_movement_start_state(movement)
    end_state = process.get_movement_end_state(movement)
    start_conf = start_state['robot'].kinematic_config
    end_conf = end_state['robot'].kinematic_config

    # * set start state
    set_state(client, robot, process, start_state)
    client._print_object_summary()

    if start_conf is None or end_conf is None:
        cprint('At least one of robot start/end conf is NOT specified in {}, return None'.format(movement.short_summary), 'red')
        cprint('Start {} | End {}'.format(start_conf, end_conf), 'red')
        wait_for_user()
        return None

    # * custom limits
    custom_limits = get_gantry_robot_custom_limits(MAIN_ROBOT_ID)
    if 'custom_limits' not in options:
        options.update({'custom_limits' : custom_limits})

    goal_constraints = robot.constraints_from_configuration(end_conf, [0.01], [0.01], group=GANTRY_ARM_GROUP)
    with LockRenderer():
        traj = client.plan_motion(robot, goal_constraints, start_configuration=start_conf, group=GANTRY_ARM_GROUP, options=options)
    if traj is None:
        cprint('No free movement found for {}.'.format(movement.short_summary), 'red')
    else:
        cprint('Free movement found for {}!'.format(movement.short_summary), 'green')
    return traj


##########################################
# archived
# auto sample IK for FreeMovements
        # * sample from t0cp if no conf is provided for the robot
        # cprint('No robot start/end conf is specified in {}, performing random IK solves.'.format(movement), 'yellow')
        # start_t0cf_frame = start_state['robot'].current_frame
        # end_t0cf_frame = end_state['robot'].current_frame
        # if start_t0cf_frame is not None:
        #     start_conf = client.inverse_kinematics(robot, start_t0cf_frame, group=GANTRY_ARM_GROUP, options=options)
        # else:
        #     if initial_conf is not None:
        #         cprint('No robot start frame is specified in {}, using initial conf.'.format(movement), 'yellow')
        #         start_conf = initial_conf
        #     else:
        #         cprint('Underspecified problem, solve fails.', 'red')
        #         return None
        # if end_t0cf_frame is not None:
        #     end_conf = client.inverse_kinematics(robot, end_t0cf_frame, group=GANTRY_ARM_GROUP, options=options)
        # else:
        #     if initial_conf is not None:
        #         cprint('No robot end frame is specified in {}, using initial conf.'.format(movement), 'yellow')
        #         end_conf = initial_conf
        #     else:
        #         cprint('Underspecified problem, solve fails.', 'red')
        #         return None
        # if start_conf is None or end_conf is None:
        #     cprint('IK solves for start and end conf fails.', 'red')
        #     return None

