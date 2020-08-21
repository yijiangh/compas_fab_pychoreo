import os
import time
import numpy as np
import argparse
import datetime
import json
import jsonpickle
from math import radians as rad
from termcolor import cprint
from itertools import product
from copy import copy, deepcopy
from numpy.testing import assert_almost_equal

from compas.geometry import Scale
from integral_timber_joints.process.algorithms import *
# from integral_timber_joints.process.pb_planning.pb_path_planner import PbRFLPathPlanner

import cProfile
import pstats

from compas.datastructures import Mesh
from compas.geometry import Transformation
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh, JointTrajectory, Duration, JointTrajectoryPoint

from pybullet_planning import link_from_name, get_link_pose, draw_pose, get_bodies, multiply, Pose, Euler, set_joint_positions, \
    joints_from_names, quat_angle_between, get_collision_fn, create_obj, unit_pose, set_camera_pose, pose_from_tform, set_pose, \
    joint_from_name, LockRenderer, unit_quat, WorldSaver, body_from_end_effector
from pybullet_planning import wait_if_gui, wait_for_duration, wait_for_user
from pybullet_planning import plan_cartesian_motion, plan_cartesian_motion_lg
from pybullet_planning import randomize, elapsed_time, apply_alpha, RED, BLUE, YELLOW, GREEN, GREY
from pybullet_planning import get_sample_fn, link_from_name, sample_tool_ik, interpolate_poses, get_joint_positions, pairwise_collision, \
    get_floating_body_collision_fn

from compas_fab_pychoreo.backend_features.pybullet_configuration_collision_checker import PybulletConfigurationCollisionChecker
from compas_fab_pychoreo.client import PyBulletClient
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo_examples.ik_solver import ik_abb_irb4600_40_255, InverseKinematicsSolver, get_ik_fn_from_ikfast
from compas_fab_pychoreo.utils import divide_list_chunks, values_as_list

from parsing import rfl_setup, itj_TC_PG500_cms, itj_TC_PG1000_cms, itj_rfl_obstacle_cms, itj_rfl_pipe_cms
from utils import to_rlf_robot_full_conf, rfl_camera, notify

MIL2M = 1e-3

# RETREAT_DISTANCE = 0.05
RETREAT_DISTANCE = 0.00

WORLD_FROM_DESIGN_POSE = pose_from_tform(np.array([
    [-6.79973677324631E-05,0.99999999327,9.40019036398793E-05,24141.9306103356*MIL2M],
    [-0.99999999612,-6.80026317324415E-05,5.59968037531752E-05,12149.6554847809*MIL2M],
    [5.60031957531521E-05,-9.39980956398953E-05,0.999999994014,26.1092114057604*MIL2M],
    [0,0,0,1]]))

PREV_BEAM_COLOR = apply_alpha(RED, 1)
CUR_BEAM_COLOR = apply_alpha(GREEN, 1)

CART_PROCESS_NAME_FROM_ID = {
    0 : 'approach_to_inclamp',
    1 : 'inclamp_to_final',
    2 : 'final_to_retract',
}

###########################################

HERE = os.path.dirname(os.path.abspath(__file__))

#TODO make argparse options
JSON_PATH_IN = os.path.join(HERE, 'data', "rfl_assembly_process.json")
JSON_OUT_DIR = os.path.join(HERE, 'results')

R11_START_CONF_VALS = np.array([22700.0, 0.0, -4900.0, 0.0, -80.0, 65.0, 65.0, 20.0, -20.0])
R12_START_CONF_VALS = np.array([-4056.0883789999998, -4000.8486330000001, 0.0, -22.834741999999999, -30.711554, 0.0, 57.335655000000003, 0.0])

R11_INTER_CONF_VALS = [21000.0, 0.0, -4900.0, 0.0, -80.0, 65.0, 65.0, 20.0, -20.0]
# R12_INTER_CONF_VALS = [-11753.0, -4915.0, 0.0, -45.0, 16.0, 10.0, 19.0, 0.0]
R12_INTER_CONF_VALS = [-4056.0883789999998, -4000.8486330000001, 0.0, -22.834741999999999, -30.711554, 0.0, 57.335655000000003, 0.0]

def convert_r12_mil_deg(conf_vals, scale=MIL2M):
    return [conf_vals[i]*scale for i in range(0, 2)] + [rad(conf_vals[i]) for i in range(2, 8)]

def compute_movement(json_path_in=JSON_PATH_IN, json_out_dir=JSON_OUT_DIR, viewer=False, debug=False, write=False, \
    seq_i=1, parse_transfer_motion=False, disable_attachment=False, disable_env=False):
    # * Connect to path planning backend and initialize robot parameters
    urdf_filename, robot = rfl_setup()

    arm_move_group = 'robot12'
    ik_base_link_name = robot.get_base_link_name(group=arm_move_group)
    ik_joint_names = robot.get_configurable_joint_names(group=arm_move_group)
    ik_joint_types = robot.get_joint_types_by_names(ik_joint_names)
    tool_link_name = robot.get_end_effector_link_name(group=arm_move_group)

    yzarm_move_group = 'robot12_eaYZ'
    yzarm_joint_names = robot.get_configurable_joint_names(group=yzarm_move_group)
    yzarm_joint_types = robot.get_joint_types_by_names(yzarm_joint_names)

    gantry_x_joint_name = 'bridge1_joint_EA_X'
    gantry_joint_names = []
    for jt_n, jt_t in zip(yzarm_joint_names, yzarm_joint_types):
        if jt_t == 2:
            gantry_joint_names.append(jt_n)
    flange_link_name = robot.get_end_effector_link_name(group=yzarm_move_group)

    # * Load process from file
    with open(json_path_in, 'r') as f:
        json_str = f.read()
        cprint ("{} loaded: json_str len: {}".format(json_path_in, len(json_str)), 'green')
        process = jsonpickle.decode(json_str, keys=True) # type: RobotClampAssemblyProcess
    assembly = process.assembly # For convinence
    toolchanger = process.robot_toolchanger
    beam_ids = [b for b in process.assembly.sequence]
    beam_id = beam_ids[seq_i-1]
    cprint('Beam #{} | previous beams: {}'.format(beam_id, assembly.get_already_built_beams(beam_id)), 'cyan')

    # if parse_transfer_motion:
    #     # * parse existing transfer motion
    #     with open(os.path.join(HERE, 'data', "trajectory_s{}_{}_transfer.json").format(seq_i, beam_id)) as json_file:
    #         transfer_path_data = json.load(json_file)
    #     transfer_traj = JointTrajectory.from_data(transfer_path_data)

    gripper_type = assembly.get_beam_attribute(beam_id, 'gripper_type')
    ee_touched_link_names = ['robot12_tool0', 'robot12_link_6']
    ee_cms_fn = itj_TC_PG1000_cms if 'PG1000' in gripper_type else itj_TC_PG500_cms
    ee_acms = [AttachedCollisionMesh(ee_cm, flange_link_name, ee_touched_link_names) for ee_cm in ee_cms_fn()]
    cprint('Using gripper {}'.format(gripper_type), 'yellow')

    pipe_cms = itj_rfl_pipe_cms()
    # Attached CM (Pipes around Robot)
    attached_cm_pipe2 = AttachedCollisionMesh(pipe_cms[0], 'robot12_link_2', ['robot12_link_2'])
    attached_cm_pipe3 = AttachedCollisionMesh(pipe_cms[1], 'robot12_link_3', ['robot12_link_3'])

    with PyBulletClient(viewer=viewer) as client:
        client.load_robot_from_urdf(urdf_filename)
        client.compas_fab_robot = robot
        robot_uid = client.robot_uid

        # joint indices
        gantry_x_joint = joint_from_name(robot_uid, gantry_x_joint_name)
        gantry_joints = joints_from_names(robot_uid, gantry_joint_names)
        ik_base_link = link_from_name(robot_uid, ik_base_link_name)
        ik_joints = joints_from_names(robot_uid, ik_joint_names)
        ik_tool_link = link_from_name(robot_uid, tool_link_name)
        yzarm_joints = joints_from_names(robot_uid, yzarm_joint_names)

        draw_pose(unit_pose(), length=1.)
        cam = rfl_camera()
        set_camera_pose(cam['target'], cam['location'])

        # * start conf
        full_start_conf = to_rlf_robot_full_conf(R11_INTER_CONF_VALS, R12_INTER_CONF_VALS)
        full_joints = joints_from_names(robot_uid, full_start_conf.joint_names)
        set_joint_positions(robot_uid, full_joints, full_start_conf.values)

        # * attachments
        for ee_acm in ee_acms:
            client.add_attached_collision_mesh(ee_acm, YELLOW)
        # pipe attachments
        # if not disable_attachment:
        #     client.add_attached_collision_mesh(attached_cm_pipe2, BLUE)
        #     client.add_attached_collision_mesh(attached_cm_pipe3, BLUE)

        # # * Add static collision mesh to planning scene
        if not disable_env:
            for o_cm in itj_rfl_obstacle_cms():
                client.add_collision_mesh(o_cm, GREY)

        # * collision sanity check
        # start_confval = np.hstack([r12_start_conf_vals[:2]*1e-3, np.radians(r12_start_conf_vals[2:])])
        # conf = Configuration(values=start_confval.tolist(), types=yzarm_joint_types, joint_names=yzarm_joint_names)
        # assert not client.configuration_in_collision(conf, group=yzarm_move_group, options={'diagnosis':True})

        # * Individual process path planning

        def flange_frame_at(attribute_name):
            gripper_t0cp = process.get_gripper_t0cp_for_beam_at(beam_id, attribute_name)
            toolchanger.set_current_frame_from_tcp(gripper_t0cp)
            return toolchanger.current_frame.copy()

        if process.get_gripper_t0cp_for_beam_at(beam_id, 'assembly_wcf_inclampapproach'):
            assembly_wcf_inclampapproach =  flange_frame_at('assembly_wcf_inclampapproach')
        else:
            assembly_wcf_inclampapproach =  flange_frame_at('assembly_wcf_inclamp')
        assembly_wcf_inclamp =  flange_frame_at('assembly_wcf_inclamp')
        assembly_wcf_final =  flange_frame_at('assembly_wcf_final')
        assembly_wcf_finalretract =  flange_frame_at('assembly_wcf_finalretract')

        # these tcp are all for flange frame
        wcf_inclamp_approach_pose = multiply(WORLD_FROM_DESIGN_POSE, pose_from_frame(assembly_wcf_inclampapproach, MIL2M))
        wcf_inclamp_pose = multiply(WORLD_FROM_DESIGN_POSE, pose_from_frame(assembly_wcf_inclamp, MIL2M))
        wcf_final_detach = multiply(WORLD_FROM_DESIGN_POSE, pose_from_frame(assembly_wcf_final, MIL2M))
        wcf_final_retract = multiply(WORLD_FROM_DESIGN_POSE, pose_from_frame(assembly_wcf_finalretract, MIL2M))

        # draw target poses
        set_camera_pose(wcf_inclamp_approach_pose[0] + np.array([0,-0.5,0]), wcf_inclamp_approach_pose[0])

        # * add previously assembled beam to the scene
        for prev_beam_id in assembly.get_already_built_beams(beam_id):
            prev_beam = assembly.beam(prev_beam_id)
            beam_cm = CollisionMesh(prev_beam.cached_mesh, prev_beam.name)
            beam_cm.scale(MIL2M)
            beam_body = client.add_collision_mesh(beam_cm, PREV_BEAM_COLOR)[0]
            set_pose(beam_body, WORLD_FROM_DESIGN_POSE)

        # * add current beam to be transferred
        # Select the current beam
        cur_beam = assembly.beam(beam_id)
        gripper_type = assembly.get_beam_attribute(beam_id, 'gripper_type')
        gripper = process.get_one_gripper_by_type(gripper_type)
        gripper.current_frame = process.robot_toolchanger.tool_coordinate_frame.copy()
        # Beam (In Gripper def pose)
        gripper_t0cp = process.get_gripper_t0cp_for_beam_at(beam_id, 'assembly_wcf_final')
        T = Transformation.from_frame_to_frame(gripper_t0cp, gripper.current_frame.copy())
        cur_beam_cm = CollisionMesh(cur_beam.cached_mesh.transformed(T), cur_beam.name)
        cur_beam_cm.scale(MIL2M)
        cur_beam_acm = AttachedCollisionMesh(cur_beam_cm, flange_link_name, ee_touched_link_names)
        if not disable_attachment:
            cur_beam_attach = client.add_attached_collision_mesh(cur_beam_acm, CUR_BEAM_COLOR)

        import ikfast_abb_irb4600_40_255
        ikfast_fn = ikfast_abb_irb4600_40_255.get_ik

        retreat_vector = RETREAT_DISTANCE*np.array([0, 0, -1])
        cart_key_poses = [multiply(wcf_inclamp_approach_pose, (retreat_vector, unit_quat())),
                          wcf_inclamp_pose, wcf_final_detach,
                          multiply(wcf_final_retract, (retreat_vector, unit_quat()))]
        pos_step_size = 0.01
        cart_pose_groups = []
        cart_group_sizes = []
        for p1, p2 in zip(cart_key_poses[:-1], cart_key_poses[1:]):
            c_interp_poses = list(interpolate_poses(p1, p2, pos_step_size=pos_step_size))
            cart_pose_groups.extend(c_interp_poses)
            cart_group_sizes.append(len(c_interp_poses))
            # interpolate_poses(pose1, pose2, pos_step_size=0.01, ori_step_size=np.pi/16):
        print('cart_group_size: ', cart_group_sizes)
        for ckp in cart_key_poses:
            draw_pose(ckp)

        def get_sample_ik_fn(robot, ik_fn, robot_base_link, ik_joints, tool_from_root=None):
            def sample_ik_fn(world_from_tcp):
                if tool_from_root:
                    world_from_tcp = multiply(world_from_tcp, tool_from_root)
                return sample_tool_ik(ik_fn, robot, ik_joints, world_from_tcp, robot_base_link, get_all=True)
            return sample_ik_fn
        sample_ik_fn = get_sample_ik_fn(robot_uid, ikfast_fn, ik_base_link, ik_joints)
        collision_fn = PybulletConfigurationCollisionChecker(client)._get_collision_fn(group=arm_move_group)

        # * start planning
        # if parse_transfer_motion:
            # if a transfer plan is given, set the x, y, z axis based on the parsed transfer motion
            # full_start_conf = transfer_traj.start_configuration.copy()
            # full_start_conf.scale(MIL2M)
            # set_joint_positions(robot_uid, full_joints, full_start_conf.values)
            # if client.configuration_in_collision(transfer_traj.points[0].scaled(MIL2M), group=yzarm_move_group, options={'diagnosis':True}):
            #     wait_if_gui('collision detected!!')
            # else:
            #     print('Transfer start conf passed')
            # pass

        # gantry x limit
        # 20.500 to 25.000
        custom_yz_limits = {gantry_x_joint : (20.5, 25.0)}
        # custom_yz_limits = {gantry_x_joint : (20., 25.0)}
        gantry_x_sample_fn = get_sample_fn(robot_uid, [gantry_x_joint], custom_limits=custom_yz_limits)
        # custom_yz_limits = {gantry_joints[0] : (-12.237, -9.5),
        custom_yz_limits = {gantry_joints[0] : (-12.237, -5),
                            gantry_joints[1] : (-4.915, -3.5)}

        gantry_yz_sample_fn = get_sample_fn(robot_uid, gantry_joints, custom_limits=custom_yz_limits)

        # end_conf = transfer_traj.points[-1]
        # end_conf.scale(MIL2M)
        transfer_plan_options = {
            'diagnosis' : False,
            # 'resolution' : 0.01,
            'resolution' : 0.001,
        }

        x_attempts = 100
        yz_attempts = 50
        transfer_attempts = 10
        solution_found = False
        transfer_traj = None

        ik_failures = 0
        path_failures = 0
        samples_cnt = 0
        # wait_if_gui('Transfer planning starts')

        # sanity check, is beam colliding with the obstacles?
        with WorldSaver():
            env_obstacles = values_as_list(client.collision_objects)
            for attachment in client.attachments.values():
                body_collision_fn = get_floating_body_collision_fn(attachment.child, obstacles=env_obstacles)

                inclamp_approach_pose = body_from_end_effector(cart_key_poses[0], attachment.grasp_pose)
                if body_collision_fn(inclamp_approach_pose, diagnosis=True):
                    cprint('wcf_inclamp_approach_pose in collision!', 'red')
                    wait_for_user()
                # final_retract_pose = body_from_end_effector(cart_key_poses[3], attachment.grasp_pose)
                # if body_collision_fn(final_retract_pose, diagnosis=True):
                #     cprint('final_retract_pose in collision!', 'red')
                #     wait_for_user()
            wait_if_gui('Check the inclamp approach pose.')

        # 50 mm/s cartesian
        # 2 mm/s sync
        # TODO check why beam attachment does not lead to collision
        # ee_vel = 0.001
        ee_vel = None
        jt_vel_vals = np.array([2.618, 2.618, 2.618, 6.2832, 6.2832, 7.854]) * 0.1
        custom_vel_limits = {jt : vel for jt, vel in zip(ik_joints, jt_vel_vals)}

        with LockRenderer(True):
            for _ in range(x_attempts):
                # sample x
                gantry_x_val = gantry_x_sample_fn()
                set_joint_positions(robot_uid, [gantry_x_joint], gantry_x_val)
                print('x: ', gantry_x_val)

                for _ in range(yz_attempts):
                    # sample y-z
                    gantry_yz_vals = gantry_yz_sample_fn()
                    print('yz: ', gantry_yz_vals)
                    set_joint_positions(robot_uid, gantry_joints, gantry_yz_vals)
                    samples_cnt += 1

                    # check if collision-free ik solution exist for the four Cartesian poses
                    conf_vals = sample_ik_fn(cart_pose_groups[0])
                    for conf_val in conf_vals:
                        if conf_val is None:
                            continue
                        cart_yzarm_vals = np.hstack([gantry_yz_vals, conf_val]).tolist()
                        cart_yzarm_conf = Configuration(cart_yzarm_vals, yzarm_joint_types, yzarm_joint_names)

                        if not client.configuration_in_collision(cart_yzarm_conf, group=yzarm_move_group, options={'diagnosis':False}):
                            # set start pick conf
                            # start_conf = Configuration(convert_r12_mil_deg(R12_INTER_CONF_VALS), yzarm_joint_types, yzarm_joint_names)
                            # if client.configuration_in_collision(start_conf, group=yzarm_move_group, options={'diagnosis':True}):
                            #     cprint('initial pose collision???', 'red')
                            #     wait_if_gui()
                            # print('start conf outsidez: ', start_conf)
                            transfer_cnt = 0
                            cart_cnt = 0
                            set_joint_positions(robot_uid, yzarm_joints, convert_r12_mil_deg(R12_INTER_CONF_VALS))
                            for _ in range(transfer_attempts):
                                with WorldSaver():
                                    transfer_traj = client.plan_motion(cart_yzarm_conf, group=yzarm_move_group, options=transfer_plan_options)
                                    if transfer_traj is not None:
                                        set_joint_positions(robot_uid, yzarm_joints, transfer_traj.points[-1].values)

                                        # * Cartesian planning
                                        # gradient-based one
                                        # cart_conf_vals = plan_cartesian_motion(robot_uid, ik_joints[0], ik_tool_link, cart_pose_groups, get_sub_conf=True)
                                        # if cart_conf_vals is not None and all([not collision_fn(conf_val) for conf_val in cart_conf_vals]):

                                        # ladder graph
                                        cart_conf_vals, cost = plan_cartesian_motion_lg(robot_uid, ik_joints, cart_pose_groups, sample_ik_fn, collision_fn,
                                            ee_vel=ee_vel, custom_vel_limits=custom_vel_limits)
                                        # print('Cart sol len: {} | cost: {}'.format(len(cart_conf_vals), cost))
                                        # cart_conf_vals = plan_cartesian_motion(robot_uid, ik_joints[0], ik_tool_link, cart_pose_groups, get_sub_conf=True)

                                        if cart_conf_vals is not None:
                                            solution_found = True
                                            cprint('Collision free! After {} ik, {} path failure over {} samples.'.format(
                                                ik_failures, path_failures, samples_cnt), 'green')
                                            break
                                        else:
                                            cart_cnt += 1
                                    else:
                                        transfer_cnt += 1
                            else:
                                cprint('Path planning failure after {} attempts | {} due to transfer, {} due to Cart.'.format(
                                    transfer_attempts, transfer_cnt, cart_cnt), 'yellow')
                                path_failures += 1
                        else:
                            ik_failures += 1

                        if solution_found:
                            break
                    if solution_found:
                        break
                if solution_found:
                    break

        assert transfer_traj is not None, 'transfer plan failed after {}(X)x{}(YZ) samples (total {}) | {} due to IK, {} due to path'.format(
            x_attempts, yz_attempts, samples_cnt, ik_failures, path_failures)
        transfer_traj.start_configuration = full_start_conf.copy()
        transfer_traj.start_configuration.values[0] = gantry_x_val[0] * 1./MIL2M

        if debug:
            notify('Transfer + Cartesian Planning done.')
            wait_if_gui('Transfer + Cartesian Planning done. Start simulation...')
        # * sim transfer motion
        traj_joints = joints_from_names(robot_uid, transfer_traj.joint_names)
        for traj_pt in transfer_traj.points:
            set_joint_positions(robot_uid, traj_joints, traj_pt.values)
            traj_pt.scale(1/MIL2M)

            for _, attach in client.attachments.items():
                attach.assign()
            if debug:
                # print(traj_pt_scaled)
                # wait_for_duration(0.1)
                wait_if_gui('sim transfer.')

        assert cart_conf_vals is not None, 'Cartesian planning failure'
        confval_groups = divide_list_chunks(cart_conf_vals, cart_group_sizes)

        cart_process_start_conf = transfer_traj.start_configuration.copy()
        for i in range(9, 9+8):
            cart_process_start_conf.values[i] = transfer_traj.points[-1].values[i-9]
        cart_gantry_yz_vals = [cart_process_start_conf.values[i] for i in range(9, 11)]

        cart_process = {}
        for k, conf_vals in enumerate(confval_groups):
            cprint('Cartesian phase: {} - {}'.format(k, CART_PROCESS_NAME_FROM_ID[k]), 'cyan')
            jt_traj_pts = []
            for i, conf_val in enumerate(conf_vals):
                yzarm_conf = Configuration(values=cart_gantry_yz_vals+list(conf_val), types=yzarm_joint_types, joint_names=yzarm_joint_names)
                jt_traj_pt = JointTrajectoryPoint(values=yzarm_conf.values, types=yzarm_conf.types, \
                    time_from_start=Duration(i*1,0))
                jt_traj_pt.joint_names = yzarm_joint_names
                jt_traj_pts.append(jt_traj_pt)

                # simulation with unscaled
                yzarm_conf_scaled = yzarm_conf.scaled(MIL2M)
                set_joint_positions(robot_uid, yzarm_joints, yzarm_conf_scaled.values)
                for attach_name, attach in client.attachments.items():
                    if attach_name != cur_beam.name:
                        attach.assign()
                    elif k < 2:
                        attach.assign()

                if debug:
                    print(jt_traj_pt)
                    wait_if_gui('step Cartesian.')

            # if k == 0:
            #     assert_almost_equal(np.array(cart_process_start_conf.values)[9:17], np.array(jt_traj_pts[0].values), decimal=4)
            trajectory = JointTrajectory(trajectory_points=jt_traj_pts, \
                joint_names=yzarm_joint_names, start_configuration=cart_process_start_conf, fraction=1.0)

            cart_process[CART_PROCESS_NAME_FROM_ID[k]] = trajectory
            # update start conf
            cart_process_start_conf = transfer_traj.start_configuration.copy()
            for i in range(9, 9+8):
                cart_process_start_conf.values[i] = jt_traj_pts[-1].values[i-9]

        client.remove_attached_collision_mesh(cur_beam.name)

        cur_beam_assembled_cm = CollisionMesh(assembly.beam(beam_id).cached_mesh, assembly.beam(beam_id).name)
        cur_beam_assembled_cm.scale(MIL2M)
        placed_beam_body = client.add_collision_mesh(cur_beam_assembled_cm, PREV_BEAM_COLOR)[0]
        set_pose(placed_beam_body, WORLD_FROM_DESIGN_POSE)

        # * Transit planning
        last_cart_conf = cart_process[CART_PROCESS_NAME_FROM_ID[2]].points[-1]
        last_cart_conf_scaled = last_cart_conf.scaled(MIL2M)
        reset_conf = Configuration(convert_r12_mil_deg(R12_INTER_CONF_VALS), yzarm_joint_types, yzarm_joint_names)
        print('last_conf: ', last_cart_conf)
        print('rest_conf: ', reset_conf)

        transit_plan_options = {
            'diagnosis' : True,
            # 'resolution' : 0.01,
            'resolution' : 0.1,
        }

        transit_attempts = 10
        transit_traj = None
        for _ in range(transit_attempts):
            with LockRenderer(False):
                set_joint_positions(robot_uid, yzarm_joints, last_cart_conf_scaled.values)
                with WorldSaver():
                    transit_traj = client.plan_motion(reset_conf, group=yzarm_move_group, options=transit_plan_options)
                if transit_traj is not None:
                    break
        else:
            cprint('transit plan failed after {} attempts!'.format(transit_attempts), 'red')

        # assert transit_traj is not None, 'transfer plan failed!'
        if transit_traj is not None:
            if debug:
                wait_if_gui('Transit Planning done. Start simulation...')

            # * sim transit motion
            traj_joints = joints_from_names(robot_uid, transit_traj.joint_names)
            for traj_pt in transit_traj.points:
                # set_joint_positions(robot_uid, traj_joints, traj_pt.scaled(MIL2M).values)
                set_joint_positions(robot_uid, traj_joints, traj_pt.values)
                traj_pt.scale(1/MIL2M)
                for _, attach in client.attachments.items():
                    attach.assign()
                if debug:
                    wait_if_gui('sim transit.')
                # print(traj_pt)
                    # wait_for_duration(0.1)
                    # yzarm_conf = Configuration(traj_pt_scaled.values, yzarm_joint_types, yzarm_joint_names)
                    # if client.configuration_in_collision(yzarm_conf, group=yzarm_move_group, options={'diagnosis':True}):
                    #     wait_if_gui('collision detected!!')

        assembly_plan_data = {k : v.data for k, v in cart_process.items()}
        assembly_plan_data.update({'transfer' : transfer_traj.data})
        if transit_traj is not None:
            assembly_plan_data.update({'transit' : transit_traj.data})
        assembly_plan_data['generated_time'] = str(datetime.datetime.now())

        # * Save Results
        if write:
            output_data_path = os.path.join(JSON_OUT_DIR, 'trajectory_dict_s{}_{}_full.json'.format(seq_i, beam_id))
            with open(output_data_path, "w") as outfile:
                json.dump(assembly_plan_data, outfile)
            cprint('Computed plan trajectories saved to {}'.format(output_data_path), 'green')

            # with open(json_path_out, 'w') as f:
            #     json_str = jsonpickle.encode(process, keys=True)
            #     f.write(json_str)
            #     cprint ("Planned path saved to {}, out json_str len: {}".format(json_path_out, len(json_str)), 'green')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning, default False')
    parser.add_argument('-w', '--write', action='store_true', help='Write output json.')
    parser.add_argument('-ptm', '--parse_transfer_motion', action='store_true', help='Parse saved transfer motion.')
    parser.add_argument('-db', '--debug', action='store_true', help='Debug mode')
    parser.add_argument('-si', '--seq_i', default=1, help='individual step to plan, Victor starts from one.')
    parser.add_argument('-na', '--disable_attachment', action='store_true', help='Disable beam and pipe attachments.')
    parser.add_argument('-env', '--disable_env', action='store_true', help='Disable .')
    args = parser.parse_args()
    print('Arguments:', args)

    compute_movement(viewer=args.viewer, debug=args.debug, write=args.write, seq_i=int(args.seq_i), parse_transfer_motion=args.parse_transfer_motion, \
        disable_attachment=args.disable_attachment, disable_env=args.disable_env)


if __name__ == '__main__':
    main()
