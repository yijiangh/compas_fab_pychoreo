import os
import time
import numpy as np
import argparse
import datetime
import json
import sys
from os import path

from math import radians as rad
from termcolor import cprint
from itertools import product
from copy import copy, deepcopy
from numpy.testing import assert_almost_equal

from compas.geometry import Scale
# from integral_timber_joints.process.algorithms import *

from compas.datastructures import Mesh
from compas.geometry import Transformation
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh, JointTrajectory, Duration, JointTrajectoryPoint

from pybullet_planning import link_from_name, get_link_pose, draw_pose, get_bodies, multiply, Pose, Euler, set_joint_positions, \
    joints_from_names, quat_angle_between, get_collision_fn, create_obj, unit_pose, set_camera_pose, pose_from_tform, set_pose, \
    joint_from_name, LockRenderer, unit_quat, WorldSaver, body_from_end_effector, CLIENTS
from pybullet_planning import wait_if_gui, wait_for_duration, wait_for_user
from pybullet_planning import plan_cartesian_motion, plan_cartesian_motion_lg
from pybullet_planning import randomize, elapsed_time, apply_alpha, RED, BLUE, YELLOW, GREEN, GREY
from pybullet_planning import get_sample_fn, link_from_name, sample_tool_ik, interpolate_poses, get_joint_positions, pairwise_collision, \
    get_floating_body_collision_fn

# ikfast_pybind
import ikfast_abb_irb4600_40_255

from compas_fab_pychoreo.backend_features.pychoreo_configuration_collision_checker import PyChoreoConfigurationCollisionChecker
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo_examples.ik_solver import InverseKinematicsSolver, get_ik_fn_from_ikfast
from compas_fab_pychoreo.utils import divide_list_chunks, values_as_list

# HERE = os.path.dirname(__file__)
# if HERE not in sys.path:
#     sys.path.append(HERE)

from .parsing import parse_process
# rfl_setup, itj_TC_PG500_cms, itj_TC_PG1000_cms, itj_rfl_obstacle_cms, itj_rfl_pipe_cms,
from .visualization import rfl_camera
from .robot_setup import to_rlf_robot_full_conf, GANTRY_X_LIMIT, GANTRY_Y_LIMIT, GANTRY_Z_LIMIT, \
    R11_INTER_CONF_VALS, R12_INTER_CONF_VALS, load_RFL_world
from .utils import notify, MIL2M, convert_rfl_robot_conf_unit
from .stream import set_state

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement


PREV_BEAM_COLOR = apply_alpha(RED, 1)
CUR_BEAM_COLOR = apply_alpha(GREEN, 1)

CART_PROCESS_NAME_FROM_ID = {
    0 : 'approach_to_inclamp',
    1 : 'inclamp_to_final',
    2 : 'final_to_retract',
}

# TODO add clamp collision geometry to transit/transfer planning

# TODO use linkstatistics joint weight and resolutions
# TODO wrap into stream functions

###########################################

#TODO make argparse options
HERE = os.path.dirname(__file__)
JSON_OUT_DIR = os.path.join(HERE, 'results')

def compute_movement(client, robot, process, movement):
    print(type(movement))
    # robot_uid = client.get_robot_pybullet_uid(robot)
    if not (isinstance(movement, RoboticLinearMovement) or isinstance(movement, RoboticFreeMovement)):
        return None
    start_state = process.get_movement_start_state(movement)
    end_state = process.get_movement_end_state(movement)

    # * visualize states
    set_state(client, robot, process, start_state)
    wait_if_gui('Start state')
    set_state(client, robot, process, end_state)
    wait_if_gui('End state')

    if isinstance(movement, RoboticLinearMovement):
        pass
    elif isinstance(movement, RoboticFreeMovement):
        pass
    else:
        raise ValueError()

    # movement.trajectory = traj

    return movement

    # # * link/joint info
    # ik_base_link_name = robot.get_base_link_name(group=arm_move_group)
    # ik_joint_names = robot.get_configurable_joint_names(group=arm_move_group)
    # ik_joint_types = robot.get_joint_types_by_names(ik_joint_names)
    # tool_link_name = robot.get_end_effector_link_name(group=arm_move_group)

    # yzarm_move_group = 'robot12_eaYZ'
    # yzarm_joint_names = robot.get_configurable_joint_names(group=yzarm_move_group)
    # yzarm_joint_types = robot.get_joint_types_by_names(yzarm_joint_names)
    # flange_link_name = robot.get_end_effector_link_name(group=yzarm_move_group)

    # gantry_x_joint_name = 'bridge1_joint_EA_X'
    # custom_x_limits = {gantry_x_joint_name : GANTRY_X_LIMIT}
    # custom_yz_limits = {}

    # gantry_joint_names = []
    # for jt_n, jt_t in zip(yzarm_joint_names, yzarm_joint_types):
    #     if 'Y' in jt_n:
    #         custom_yz_limits[jt_n] = GANTRY_Y_LIMIT
    #     elif 'Z' in jt_n:
    #         custom_yz_limits[jt_n] = GANTRY_Z_LIMIT
    #     if jt_t == 2:
    #         gantry_joint_names.append(jt_n)

    # gantry_x_joint = joint_from_name(robot_uid, gantry_x_joint_name)
    # gantry_joints = joints_from_names(robot_uid, gantry_joint_names)

    # # * custom limits
    # gantry_x_sample_fn = get_sample_fn(robot_uid, [gantry_x_joint],
    #     custom_limits={joint_from_name(robot_uid, jn) : limits for jn, limits in custom_x_limits.items()})
    # gantry_yz_sample_fn = get_sample_fn(robot_uid, gantry_joints,
    #     custom_limits={joint_from_name(robot_uid, jn) : limits for jn, limits in custom_yz_limits.items()})

    # # * joint indices in pybullet
    # ik_base_link = link_from_name(robot_uid, ik_base_link_name)
    # ik_joints = joints_from_names(robot_uid, ik_joint_names)
    # ik_tool_link = link_from_name(robot_uid, tool_link_name)
    # yzarm_joints = joints_from_names(robot_uid, yzarm_joint_names)

    # # * attachments
    # gripper_type = assembly.get_beam_attribute(beam_id, 'gripper_type')
    # ee_touched_link_names = ['robot12_tool0', 'robot12_link_6']
    # ee_cms_fn = itj_TC_PG1000_cms if 'PG1000' in gripper_type else itj_TC_PG500_cms
    # ee_acms = [AttachedCollisionMesh(ee_cm, flange_link_name, ee_touched_link_names) for ee_cm in ee_cms_fn()]
    # cprint('Using gripper {}'.format(gripper_type), 'yellow')

    # for ee_acm in ee_acms:
    #     # ! note that options must contain 'robot' and 'mass' entries
    #     client.add_attached_collision_mesh(ee_acm, options={'robot': robot, 'mass': 1, 'color': YELLOW})

    # # * Individual process path planning
    # def flange_frame_at(subprocess_name):
    #     gripper_t0cp = process.get_gripper_t0cp_for_beam_at(beam_id, subprocess_name)
    #     toolchanger.set_current_frame_from_tcp(gripper_t0cp)
    #     return toolchanger.current_frame.copy()

    # if process.get_gripper_t0cp_for_beam_at(beam_id, 'assembly_wcf_inclampapproach'):
    #     assembly_wcf_inclampapproach =  flange_frame_at('assembly_wcf_inclampapproach')
    # else:
    #     assembly_wcf_inclampapproach =  flange_frame_at('assembly_wcf_inclamp')
    # assembly_wcf_inclamp =  flange_frame_at('assembly_wcf_inclamp')
    # assembly_wcf_final =  flange_frame_at('assembly_wcf_final')
    # assembly_wcf_finalretract =  flange_frame_at('assembly_wcf_finalretract')

    # # these tcp are all for flange frame
    # wcf_inclamp_approach_pose = multiply(WORLD_FROM_DESIGN_POSE, pose_from_frame(assembly_wcf_inclampapproach, MIL2M))
    # wcf_inclamp_pose = multiply(WORLD_FROM_DESIGN_POSE, pose_from_frame(assembly_wcf_inclamp, MIL2M))
    # wcf_final_detach = multiply(WORLD_FROM_DESIGN_POSE, pose_from_frame(assembly_wcf_final, MIL2M))
    # wcf_final_retract = multiply(WORLD_FROM_DESIGN_POSE, pose_from_frame(assembly_wcf_finalretract, MIL2M))

    # # draw target poses
    # set_camera_pose(wcf_inclamp_approach_pose[0] + np.array([0,-0.5,0]), wcf_inclamp_approach_pose[0])

    # # * add previously assembled beam to the scene
    # for prev_beam_id in assembly.get_already_built_beams(beam_id):
    #     prev_beam = assembly.beam(prev_beam_id)
    #     beam_cm = CollisionMesh(prev_beam.cached_mesh, prev_beam.name)
    #     beam_cm.scale(MIL2M)
    #     client.add_collision_mesh(beam_cm, {'color':PREV_BEAM_COLOR})
    #     beam_body = client.collision_objects[beam_cm.id][0]
    #     set_pose(beam_body, WORLD_FROM_DESIGN_POSE)

    # # * add current beam to be transferred
    # # Select the current beam
    # cur_beam = assembly.beam(beam_id)
    # gripper_type = assembly.get_beam_attribute(beam_id, 'gripper_type')
    # gripper = process.get_one_gripper_by_type(gripper_type)
    # gripper.current_frame = process.robot_toolchanger.tool_coordinate_frame.copy()
    # # Beam (In Gripper def pose)
    # gripper_t0cp = process.get_gripper_t0cp_for_beam_at(beam_id, 'assembly_wcf_final')
    # T = Transformation.from_frame_to_frame(gripper_t0cp, gripper.current_frame.copy())
    # cur_beam_cm = CollisionMesh(cur_beam.cached_mesh.transformed(T), cur_beam.name)
    # cur_beam_cm.scale(MIL2M)
    # cur_beam_acm = AttachedCollisionMesh(cur_beam_cm, flange_link_name, ee_touched_link_names)

    # if not disable_attachment:
    #     client.add_attached_collision_mesh(cur_beam_acm, {'robot': robot, 'color':CUR_BEAM_COLOR, 'mass':1})

    # retreat_vector = RETREAT_DISTANCE*np.array([0, 0, -1])
    # cart_key_poses = [multiply(wcf_inclamp_approach_pose, (retreat_vector, unit_quat())),
    #                   wcf_inclamp_pose, wcf_final_detach,
    #                   multiply(wcf_final_retract, (retreat_vector, unit_quat()))]
    # cart_pose_groups = []
    # cart_group_sizes = []
    # for p1, p2 in zip(cart_key_poses[:-1], cart_key_poses[1:]):
    #     c_interp_poses = list(interpolate_poses(p1, p2, pos_step_size=POS_STEP_SIZE))
    #     cart_pose_groups.extend(c_interp_poses)
    #     cart_group_sizes.append(len(c_interp_poses))
    #     # interpolate_poses(pose1, pose2, pos_step_size=0.01, ori_step_size=np.pi/16):
    # print('cart_group_size: ', cart_group_sizes)
    # for ckp in cart_key_poses:
    #     draw_pose(ckp)

    # ikfast_fn = ikfast_abb_irb4600_40_255.get_ik
    # def get_sample_ik_fn(robot, ik_fn, robot_base_link, ik_joints, tool_from_root=None):
    #     def sample_ik_fn(world_from_tcp):
    #         if tool_from_root:
    #             world_from_tcp = multiply(world_from_tcp, tool_from_root)
    #         return sample_tool_ik(ik_fn, robot, ik_joints, world_from_tcp, robot_base_link, get_all=True)
    #     return sample_ik_fn
    # sample_ik_fn = get_sample_ik_fn(robot_uid, ikfast_fn, ik_base_link, ik_joints)

    # # ikfast_fn = get_ik_fn_from_ikfast(ikfast_abb_irb4600_40_255.get_ik)
    # # ik_solver = InverseKinematicsSolver(robot, move_group, ikfast_fn, base_frame, robotA_tool.frame)
    # # client.planner.inverse_kinematics = ik_solver.inverse_kinematics_function()

    # transfer_plan_options = {
    #     'diagnosis' : False,
    #     # 'resolution' : 0.01,
    #     'resolution' : 0.001,
    #     'custom_limits' : custom_yz_limits,
    # }

    # x_attempts = 100
    # yz_attempts = 50
    # transfer_attempts = 2
    # solution_found = False
    # transfer_traj = None

    # ik_failures = 0
    # path_failures = 0
    # samples_cnt = 0
    # # wait_if_gui('Transfer planning starts')

    # # * sanity check, is beam colliding with the obstacles?
    # with WorldSaver():
    #     env_obstacles = values_as_list(client.collision_objects)
    #     for attachments in client.pychoreo_attachments.values():
    #         for attachment in attachments:
    #             body_collision_fn = get_floating_body_collision_fn(attachment.child, obstacles=env_obstacles)

    #             inclamp_approach_pose = body_from_end_effector(cart_key_poses[0], attachment.grasp_pose)
    #             if body_collision_fn(inclamp_approach_pose, diagnosis=True):
    #                 cprint('wcf_inclamp_approach_pose in collision!', 'red')
    #                 wait_for_user()
    #             # final_retract_pose = body_from_end_effector(cart_key_poses[3], attachment.grasp_pose)
    #             # if body_collision_fn(final_retract_pose, diagnosis=True):
    #             #     cprint('final_retract_pose in collision!', 'red')
    #             #     wait_for_user()
    #     # wait_if_gui('Check the inclamp approach pose.')

    # with LockRenderer(True):
    #     for _ in range(x_attempts):
    #         # sample x
    #         gantry_x_val = gantry_x_sample_fn()
    #         set_joint_positions(robot_uid, [gantry_x_joint], gantry_x_val)
    #         print('x: ', gantry_x_val)

    #         for _ in range(yz_attempts):
    #             # sample y-z
    #             gantry_yz_vals = gantry_yz_sample_fn()
    #             print('yz: ', gantry_yz_vals)
    #             set_joint_positions(robot_uid, gantry_joints, gantry_yz_vals)
    #             samples_cnt += 1

    #             # check if collision-free ik solution exist for the four Cartesian poses
    #             conf_vals = sample_ik_fn(cart_pose_groups[0])
    #             # print(conf_vals)
    #             # wait_for_user()

    #             for conf_val in conf_vals:
    #                 if conf_val is None:
    #                     continue
    #                 cart_yzarm_vals = np.hstack([gantry_yz_vals, conf_val]).tolist()
    #                 cart_yzarm_conf = Configuration(cart_yzarm_vals, yzarm_joint_types, yzarm_joint_names)

    #                 if not client.check_collisions(robot, cart_yzarm_conf, options={'diagnosis':False}):
    #                 # if not client.configuration_in_collision(cart_yzarm_conf, group=yzarm_move_group, options={'diagnosis':False}):
    #                     transfer_cnt = 0
    #                     cart_cnt = 0
    #                     # start_conf = Configuration(convert_r12_mil_deg(R12_INTER_CONF_VALS), yzarm_joint_types, yzarm_joint_names)
    #                     # if client.configuration_in_collision(start_conf, group=yzarm_move_group, options={'diagnosis':True}):
    #                     #     cprint('initial pose collision???', 'red')
    #                     #     wait_if_gui()
    #                     # print('start conf outsidez: ', start_conf)
    #                     # set_joint_positions(robot_uid, yzarm_joints, R12_INTER_CONF_VALS)
    #                     # * set start pick conf
    #                     client.set_robot_configuration(robot, Configuration(R12_INTER_CONF_VALS, yzarm_joint_types, yzarm_joint_names))
    #                     for _ in range(transfer_attempts):
    #                         with WorldSaver():
    #                             goal_constraints = robot.constraints_from_configuration(cart_yzarm_conf, [0.01], [0.01], group=yzarm_move_group)
    #                             transfer_traj = client.plan_motion(robot, goal_constraints, group=yzarm_move_group, options=transfer_plan_options)
    #                             if transfer_traj is not None:
    #                                 set_joint_positions(robot_uid, yzarm_joints, transfer_traj.points[-1].values)

    #                                 # * Cartesian planning
    #                                 # TODO replace with client one
    #                                 # gradient-based one
    #                                 cart_conf_vals = plan_cartesian_motion(robot_uid, ik_joints[0], ik_tool_link, cart_pose_groups, get_sub_conf=True)
    #                                 # if cart_conf_vals is not None and all([not collision_fn(conf_val) for conf_val in cart_conf_vals]):

    #                                 # ladder graph
    #                                 # cart_conf_vals, cost = plan_cartesian_motion_lg(robot_uid, ik_joints, cart_pose_groups, sample_ik_fn, collision_fn,
    #                                 #     ee_vel=ee_vel, custom_vel_limits=custom_vel_limits)
    #                                 # print('Cart sol len: {} | cost: {}'.format(len(cart_conf_vals), cost))

    #                                 if cart_conf_vals is not None:
    #                                     solution_found = True
    #                                     cprint('Collision free! After {} ik, {} path failure over {} samples.'.format(
    #                                         ik_failures, path_failures, samples_cnt), 'green')
    #                                     break
    #                                 else:
    #                                     cart_cnt += 1
    #                             else:
    #                                 transfer_cnt += 1
    #                     else:
    #                         cprint('Cartesian Path planning failure after {} attempts | {} due to transfer, {} due to Cart.'.format(
    #                             transfer_attempts, transfer_cnt, cart_cnt), 'yellow')
    #                         path_failures += 1
    #                 else:
    #                     ik_failures += 1

    #                 if solution_found:
    #                     break
    #             if solution_found:
    #                 break
    #         if solution_found:
    #             break

    # assert transfer_traj is not None, 'transfer plan failed after {}(X)x{}(YZ) samples (total {}) | {} due to IK, {} due to path'.format(
    #     x_attempts, yz_attempts, samples_cnt, ik_failures, path_failures)
    # # transfer_traj.start_configuration = full_start_conf.copy()
    # # transfer_traj.start_configuration.values[0] = gantry_x_val[0] * 1./MIL2M

    # # if debug:
    # notify('Transfer + Cartesian Planning done.')
    # wait_if_gui('Transfer + Cartesian Planning done. Start simulation...')

    # # * sim transfer motion
    # for traj_pt in transfer_traj.points:
    #     client.set_robot_configuration(robot, traj_pt) #, group=yzarm_move_group
    #     if debug:
    #         wait_for_duration(0.1)
    #         # wait_for_duration(time_step)
    #         # wait_if_gui('sim transfer.')

    # assert cart_conf_vals is not None, 'Cartesian planning failure'
    # confval_groups = divide_list_chunks(cart_conf_vals, cart_group_sizes)

    # # cart_process_start_conf = transfer_traj.start_configuration.copy()
    # # for i in range(9, 9+8):
    # #     cart_process_start_conf.values[i] = transfer_traj.points[-1].values[i-9]
    # # cart_gantry_yz_vals = [cart_process_start_conf.values[i] for i in range(9, 11)]

    # cart_process_start_conf = transfer_traj.points[-1].copy()
    # cart_gantry_yz_vals = [cart_process_start_conf.values[i] for i in range(0, 2)]

    # cart_process = {}
    # for k, conf_vals in enumerate(confval_groups):
    #     cprint('Cartesian phase: {} - {}'.format(k, CART_PROCESS_NAME_FROM_ID[k]), 'cyan')
    #     jt_traj_pts = []
    #     # * drop the beam before final_to_retreat
    #     if k==2:
    #         client.detach_attached_collision_mesh(cur_beam.name)
    #     for i, conf_val in enumerate(conf_vals):
    #         yzarm_conf = Configuration(values=cart_gantry_yz_vals+list(conf_val), types=yzarm_joint_types, joint_names=yzarm_joint_names)
    #         jt_traj_pt = JointTrajectoryPoint(values=yzarm_conf.values, types=yzarm_conf.types, \
    #             time_from_start=Duration(i*1,0))
    #         jt_traj_pt.joint_names = yzarm_joint_names
    #         jt_traj_pts.append(jt_traj_pt)
    #         client.set_robot_configuration(robot, jt_traj_pt) #, group=yzarm_move_group
    #         if debug:
    #             wait_if_gui('step Cartesian.')
    #     # if k == 0:
    #     #     assert_almost_equal(np.array(cart_process_start_conf.values)[9:17], np.array(jt_traj_pts[0].values), decimal=4)
    #     trajectory = JointTrajectory(trajectory_points=jt_traj_pts, \
    #         joint_names=yzarm_joint_names, start_configuration=jt_traj_pts[0], fraction=1.0)
    #     cart_process[CART_PROCESS_NAME_FROM_ID[k]] = trajectory

    # cur_beam_assembled_cm = CollisionMesh(assembly.beam(beam_id).cached_mesh, assembly.beam(beam_id).name)
    # cur_beam_assembled_cm.scale(MIL2M)
    # client.add_collision_mesh(cur_beam_assembled_cm, {'color':PREV_BEAM_COLOR})
    # placed_beam_body = client.collision_objects[cur_beam_assembled_cm.id][0]
    # set_pose(placed_beam_body, WORLD_FROM_DESIGN_POSE)

    # # * Transit planning
    # last_cart_conf = cart_process[CART_PROCESS_NAME_FROM_ID[2]].points[-1]
    # reset_conf = Configuration(R12_INTER_CONF_VALS, yzarm_joint_types, yzarm_joint_names)

    # transit_plan_options = {
    #     'diagnosis' : True,
    #     'resolution' : 0.1, # 0.01
    #     'custom_limits' : custom_yz_limits,
    # }
    # goal_constraints = robot.constraints_from_configuration(reset_conf, [0.01], [0.01], group=yzarm_move_group)

    # transit_attempts = 2
    # transit_traj = None
    # for _ in range(transit_attempts):
    #     with LockRenderer(True):
    #         set_joint_positions(robot_uid, yzarm_joints, last_cart_conf.values)
    #         with WorldSaver():
    #             transit_traj = client.plan_motion(robot, goal_constraints, start_configuration=last_cart_conf,
    #                 group=yzarm_move_group, options=transit_plan_options)
    #         if transit_traj is not None:
    #             break
    # else:
    #     cprint('transit plan failed after {} attempts!'.format(transit_attempts), 'red')

    # # assert transit_traj is not None, 'transfer plan failed!'
    # if transit_traj is not None:
    #     if debug:
    #         wait_if_gui('Transit Planning done. Start simulation...')
    #     # * sim transit motion
    #     for traj_pt in transit_traj.points:
    #         client.set_robot_configuration(robot, traj_pt)
    #         if debug:
    #             wait_if_gui('sim transit.')

    # assembly_plan_data = {k : v.data for k, v in cart_process.items()}
    # assembly_plan_data.update({'transfer' : transfer_traj.data})
    # if transit_traj is not None:
    #     assembly_plan_data.update({'transit' : transit_traj.data})
    # assembly_plan_data['generated_time'] = str(datetime.datetime.now())

    # # * Save Results
    # if write:
    #     output_data_path = os.path.join(JSON_OUT_DIR, 'trajectory_dict_s{}_{}_full.json'.format(seq_i, beam_id))
    #     with open(output_data_path, "w") as outfile:
    #         json.dump(assembly_plan_data, outfile)
    #     cprint('Computed plan trajectories saved to {}'.format(output_data_path), 'green')

    #     # with open(json_path_out, 'w') as f:
    #     #     json_str = jsonpickle.encode(process, keys=True)
    #     #     f.write(json_str)
    #     #     cprint ("Planned path saved to {}, out json_str len: {}".format(json_path_out, len(json_str)), 'green')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--problem', default='pavilion_process.json',
                        help='The name of the problem to solve')
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning, default False')
    parser.add_argument('-w', '--write', action='store_true', help='Write output json.')
    parser.add_argument('-ptm', '--parse_transfer_motion', action='store_true', help='Parse saved transfer motion.')
    parser.add_argument('--debug', action='store_true', help='Debug mode')
    parser.add_argument('-si', '--seq_i', default=1, help='individual step to plan, Victor starts from one.')
    parser.add_argument('-na', '--disable_attachment', action='store_true', help='Disable beam and pipe attachments.')
    parser.add_argument('--disable_env', action='store_true', help='Disable environment collision geometry.')
    args = parser.parse_args()
    print('Arguments:', args)

    # * Connect to path planning backend and initialize robot parameters
    arm_move_group = 'robot11'
    seq_i = seq_i=int(args.seq_i)
    client, robot, robot_uid = load_RFL_world(viewer=args.viewer, disable_env=True)

    process = parse_process(args.problem)
    assembly = process.assembly
    beam_ids = [b for b in process.assembly.sequence]
    beam_id = beam_ids[seq_i-1]
    cprint('Beam #{} | previous beams: {}'.format(beam_id, assembly.get_already_built_beams(beam_id)), 'cyan')

    set_state(client, robot, process, process.initial_state, initialize=True)
    wait_if_gui('Initial state.')

    movements = process.get_movements_by_beam_id(beam_id)
    for movement in movements:
        updated_movement = compute_movement(client, robot, process, movement)


    # * simulate ends
    client.disconnect()


if __name__ == '__main__':
    main()
