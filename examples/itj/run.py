import os
import time
import numpy as np
import argparse
import json
import jsonpickle
from termcolor import cprint
from itertools import product
from copy import copy

from compas.geometry import Scale
from integral_timber_joints.process.algorithms import *
# from integral_timber_joints.process.pb_planning.pb_path_planner import PbRFLPathPlanner

import cProfile
import pstats

from compas.datastructures import Mesh
from compas.geometry import Transformation
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh, JointTrajectory, Duration, JointTrajectoryPoint

from pybullet_planning import link_from_name, get_link_pose, draw_pose, get_bodies, multiply, Pose, Euler, set_joint_positions, \
    joints_from_names, quat_angle_between, get_collision_fn, create_obj, unit_pose, set_camera_pose, pose_from_tform, set_pose
from pybullet_planning import wait_if_gui, wait_for_duration
from pybullet_planning import plan_cartesian_motion, plan_cartesian_motion_lg
from pybullet_planning import randomize, elapsed_time, apply_alpha, RED, BLUE, YELLOW, GREEN, GREY
from pybullet_planning import get_sample_fn, link_from_name, sample_tool_ik, interpolate_poses, get_joint_positions

from compas_fab_pychoreo.backend_features.pybullet_configuration_collision_checker import PybulletConfigurationCollisionChecker
from compas_fab_pychoreo.client import PyBulletClient
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo_examples.ik_solver import ik_abb_irb4600_40_255, InverseKinematicsSolver, get_ik_fn_from_ikfast
from compas_fab_pychoreo.utils import divide_list_chunks

from parsing import rfl_setup, itj_TC_PG500_cms, itj_TC_PG1000_cms, itj_rfl_obstacle_cms
from utils import to_rlf_robot_full_conf, rfl_camera

MIL2M = 1e-3

WORLD_FROM_DESIGN_POSE = pose_from_tform(np.array([
    [-6.79973677324631E-05,0.99999999327,9.40019036398793E-05,24141.9306103356*MIL2M],
    [-0.99999999612,-6.80026317324415E-05,5.59968037531752E-05,12149.6554847809*MIL2M],
    [5.60031957531521E-05,-9.39980956398953E-05,0.999999994014,26.1092114057604*MIL2M],
    [0,0,0,1]]))

PREV_BEAM_COLOR = apply_alpha(RED, 0.5)
CUR_BEAM_COLOR = apply_alpha(GREEN, 0.8)

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

TRANSFER_DATA_PATH = os.path.join(HERE, 'data', "trajectory_b37_transfer.json")

R11_START_CONF_VALS = np.array([22700.0, 0.0, -4900.0, 0.0, -80.0, 65.0, 65.0, 20.0, -20.0])
R12_START_CONF_VALS = np.array([-4056.0883789999998, -4000.8486330000001, 0.0, -22.834741999999999, -30.711554, 0.0, 57.335655000000003, 0.0])

def compute_movement(json_path_in=JSON_PATH_IN, json_out_dir=JSON_OUT_DIR, viewer=False, debug=False, write=False, seq_i=1):
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

    # gantry x limit
    # 20500 to 25000

    gantry_joint_names = []
    for jt_n, jt_t in zip(yzarm_joint_names, yzarm_joint_types):
        if jt_t == 2:
            gantry_joint_names.append(jt_n)
    flange_link_name = robot.get_end_effector_link_name(group=yzarm_move_group)

    ee_touched_link_names = ['robot12_tool0', 'robot12_link_6']
    ee_acms = [AttachedCollisionMesh(ee_cm, flange_link_name, ee_touched_link_names) for ee_cm in itj_TC_PG1000_cms()]

    # * Load process from file
    with open(json_path_in, 'r') as f:
        json_str = f.read()
        cprint ("{} loaded: json_str len: {}".format(json_path_in, len(json_str)), 'green')
        process = jsonpickle.decode(json_str, keys=True) # type: RobotClampAssemblyProcess
    assembly = process.assembly # For convinence
    toolchanger = process.robot_toolchanger

    # * parse existing transfer motion
    with open(TRANSFER_DATA_PATH) as json_file:
        transfer_path_data = json.load(json_file)
    transfer_traj = JointTrajectory.from_data(transfer_path_data)

    with PyBulletClient(viewer=viewer) as client:
        client.load_robot_from_urdf(urdf_filename)
        client.compas_fab_robot = robot
        robot_uid = client.robot_uid

        # joint indices
        gantry_joints = joints_from_names(robot_uid, gantry_joint_names)
        ik_base_link = link_from_name(robot_uid, ik_base_link_name)
        ik_joints = joints_from_names(robot_uid, ik_joint_names)
        ik_tool_link = link_from_name(robot_uid, tool_link_name)
        yzarm_joints = joints_from_names(robot_uid, yzarm_joint_names)

        draw_pose(unit_pose(), length=1.)
        cam = rfl_camera()
        set_camera_pose(cam['target'], cam['location'])

        # * start conf
        # r11_start_conf_vals = np.array([22700.0, 0.0, -4900.0, 0.0, -80.0, 65.0, 65.0, 20.0, -20.0])
        # r12_start_conf_vals = np.array([-4056.0883789999998, -4000.8486330000001, 0.0, -22.834741999999999, -30.711554, 0.0, 57.335655000000003, 0.0])
        # full_start_conf = to_rlf_robot_full_conf(r11_start_conf_vals, r12_start_conf_vals)
        full_start_conf = transfer_traj.start_configuration.copy()
        full_start_conf.scale(MIL2M)
        full_joints = joints_from_names(robot_uid, full_start_conf.joint_names)
        set_joint_positions(robot_uid, full_joints, full_start_conf.values)

        # * attachments
        for ee_acm in ee_acms:
            client.add_attached_collision_mesh(ee_acm, YELLOW)

        # * Add static collision mesh to planning scene
        for o_cm in itj_rfl_obstacle_cms():
            client.add_collision_mesh(o_cm, GREY)

        # * collision sanity check
        # start_confval = np.hstack([r12_start_conf_vals[:2]*1e-3, np.radians(r12_start_conf_vals[2:])])
        # conf = Configuration(values=start_confval.tolist(), types=yzarm_joint_types, joint_names=yzarm_joint_names)
        # assert not client.configuration_in_collision(conf, group=yzarm_move_group, options={'diagnosis':True})

        # * Individual process path planning
        beam_ids = [b for b in process.assembly.sequence]
        beam_id = beam_ids[seq_i-1]
        cprint('Beam #{} | previous beams: {}'.format(beam_id, assembly.get_already_built_beams(beam_id)), 'cyan')

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

        set_camera_pose(wcf_inclamp_approach_pose[0] + np.array([0.5,0,0]), wcf_inclamp_approach_pose[0], )
        draw_pose(wcf_inclamp_approach_pose)
        draw_pose(wcf_inclamp_pose)
        draw_pose(wcf_final_detach)
        draw_pose(wcf_final_retract)

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
        cur_beam_attach = client.add_attached_collision_mesh(cur_beam_acm, CUR_BEAM_COLOR)

        gantry_yz_sample_fn = get_sample_fn(robot_uid, gantry_joints)

        # * sim transfer motion
        traj_joints = joints_from_names(robot_uid, transfer_traj.joint_names)
        for traj_pt in transfer_traj.points:
            traj_pt.scale(MIL2M)
            set_joint_positions(robot_uid, traj_joints, traj_pt.values)
            for _, attach in client.attachments.items():
                attach.assign()
            # wait_for_duration(0.1)
            # wait_if_gui()

            # yzarm_conf = Configuration(traj_pt.values, yzarm_joint_types, yzarm_joint_names)
            # if client.configuration_in_collision(traj_pt, group=yzarm_move_group, options={'diagnosis':True}):
            #     wait_if_gui('collision detected!!')
        # wait_if_gui('wcf_inclamp_approach_pose reached.')
        cart_start_conf = transfer_traj.points[-1]

        import ikfast_abb_irb4600_40_255
        ikfast_fn = ikfast_abb_irb4600_40_255.get_ik
        cart_key_poses = [wcf_inclamp_approach_pose, wcf_inclamp_pose, wcf_final_detach, wcf_final_retract]
        pos_step_size = 0.01
        cart_pose_groups = []
        cart_group_sizes = []
        for p1, p2 in zip(cart_key_poses[:-1], cart_key_poses[1:]):
            c_interp_poses = list(interpolate_poses(p1, p2, pos_step_size=pos_step_size))
            cart_pose_groups.extend(c_interp_poses)
            cart_group_sizes.append(len(c_interp_poses))
            # interpolate_poses(pose1, pose2, pos_step_size=0.01, ori_step_size=np.pi/16):
        print('cart_group_size: ', cart_group_sizes)

        def get_sample_ik_fn(robot, ik_fn, robot_base_link, ik_joints, tool_from_root=None):
            def sample_ik_fn(world_from_tcp):
                if tool_from_root:
                    world_from_tcp = multiply(world_from_tcp, tool_from_root)
                return sample_tool_ik(ik_fn, robot, ik_joints, world_from_tcp, robot_base_link, get_all=True)
            return sample_ik_fn
        sample_ik_fn = get_sample_ik_fn(robot_uid, ikfast_fn, ik_base_link, ik_joints)
        collision_fn = PybulletConfigurationCollisionChecker(client)._get_collision_fn(group=arm_move_group)

        # for k, cart_poses in enumerate(cart_pose_groups):
            # conf_val = sample_tool_ik(ikfast_fn, robot_uid, ik_joints, cart_poses[0], ik_base_link, closest_only=True) #, get_all=True)
            # set_joint_positions(robot_uid, ik_joints, conf_val)
            # for _, attach in client.attachments.items():
            #     attach.assign()
            # cprint('Cartesian phase#{}'.format(k))
            # conf_vals = plan_cartesian_motion(robot_uid, ik_base_link, ik_tool_link, cart_poses)

        conf_vals, cost = plan_cartesian_motion_lg(robot_uid, ik_joints, cart_pose_groups, sample_ik_fn, collision_fn)
            # custom_vel_limits=vel_limits, ee_vel=ee_vel)
        confval_groups = divide_list_chunks(conf_vals, cart_group_sizes)

        wait_if_gui('start cartesian sim...')

        cart_process_start_conf = cart_start_conf
        cart_gantry_yz_vals = [cart_start_conf.values[i] for i in range(2)]
        cart_process_data = {}
        for k, conf_vals in enumerate(confval_groups):
            jt_traj_pts = []
            for i, conf_val in enumerate(conf_vals):
                yzarm_conf = Configuration(values=cart_gantry_yz_vals+conf_val, types=yzarm_joint_types, joint_names=yzarm_joint_names)
                yzarm_conf_scaled = yzarm_conf.scaled(1./MIL2M)
                jt_traj_pt = JointTrajectoryPoint(values=yzarm_conf_scaled.values, types=yzarm_conf_scaled.types, \
                    time_from_start=Duration(i*1,0))
                jt_traj_pt.joint_names = yzarm_joint_names
                jt_traj_pts.append(jt_traj_pt)

                # simulation with unscaled
                set_joint_positions(robot_uid, yzarm_joints, yzarm_conf.values)
                for attach_name, attach in client.attachments.items():
                    if attach_name != cur_beam.name:
                        attach.assign()
                    elif k < 2:
                        attach.assign()
                wait_if_gui()

            trajectory = JointTrajectory(trajectory_points=jt_traj_pts, \
                joint_names=yzarm_joint_names, start_configuration=cart_process_start_conf, fraction=1.0)
            cart_process_start_conf = jt_traj_pts[-1]
            cart_process_data[CART_PROCESS_NAME_FROM_ID[k]] = trajectory.data

        # * Save Results
        if write:
            cart_process_path = os.path.join(JSON_OUT_DIR, 'trajectory_dict_{}_cartesian.json'.format(beam_id))
            with open(cart_process_path, "w") as outfile:
                json.dump(cart_process_data, outfile)
            cprint('Cartesian trajectories saved to {}'.format(cart_process_path), 'green')

            # with open(json_path_out, 'w') as f:
            #     json_str = jsonpickle.encode(process, keys=True)
            #     f.write(json_str)
            #     cprint ("Planned path saved to {}, out json_str len: {}".format(json_path_out, len(json_str)), 'green')

        # cart_conf = Configuration(conf_val, ik_joint_types, ik_joint_names)
        # if not client.configuration_in_collision(cart_conf, group=arm_move_group, options={'diagnosis':True}):
        #     cprint('Collision free', 'green')
        #     wait_if_gui()

        # cannot move x axis when only running robot12, free x axis within each single transfer process
        # first sample x axis
        # n_attempts = 100
        # for _ in range(n_attempts):
        #     # sample y-z
        #     # check if collision-free ik solution exist for the four Cartesian poses
        #     # TODO ignore grasped beam when approaching for now
        #     # transfer to be relative to the robot's base

        #     gantry_conf_vals = gantry_yz_sample_fn()
        #     set_joint_positions(robot_uid, gantry_joints, gantry_conf_vals)
        #     # if client.configuration_in_collision()
        #     # draw_pose(wcf_inclamp_approach_pose)

        #     wait_if_gui()

        #     # if cur_beam.name in client.attachments:
        #     #     del client.attachments[cur_beam.name]
        #     # draw_pose(wcf_inclamp_pose)
        #     # draw_pose(wcf_final_detach)
        #     # draw_pose(wcf_final_retract)
        #     # * check if transfer motion exist

        # transfer motion freeze x axis

        #####################################

        # * Sequentially copy movement target_frame to next movement source_frame
        # _source_frame = None
        # for ia, action in enumerate(process.actions):
        #     for im, movement in enumerate(action.movements):
        #         if isinstance(movement, RoboticMovement):
        #             movement.source_frame = _source_frame
        #             _source_frame = movement.target_frame

        # * Sequential path planning
        # last_configuration = pp.rfl_timber_start_configuration()

        # t = time.time()
        # for ia, action in enumerate(process.actions):
        #     # if ia not in list(range(0, 20)): continue
        #     seq_n = action.seq_n
        #     act_n = action.act_n

        #     for im, movement in enumerate(action.movements):
        #         cprint ("Seq(%s) Act (%s) Mov (%s) - %s" % (seq_n, act_n, im, movement), 'cyan')

        #         if isinstance(movement, RoboticMovement):
        #             # Add already built beams to planning scene
        #             # beam_id = assembly.sequence[seq_n]
        #             # already_built_beam_ids = assembly.get_already_built_beams(beam_id)
        #             # pp.remove_collision_mesh('already_built_beams')
        #             # for already_beam_id in already_built_beam_ids:
        #             #     pp.append_collision_mesh(assembly.beam(already_beam_id).cached_mesh, 'already_built_beams')

        #             # Attach Tool and Beam to robot

        #             # Prepare Starting Configuration
        #             if last_configuration is not None:
        #                 # retrive last planned trajectory's last config
        #                 start_configuration = last_configuration
        #             else:
        #                 # Previous planning failed, compute config based on source_frame
        #                 # Source frame is created in the beginning of this file.
        #                 start_configuration = pp.ik_solution(movement.source_frame, verbose=True)

        #             # Perform planning if start_configuration is not None.
        #             if start_configuration is not None:
        #                 trajectory = pp.plan_motion(
        #                     movement.target_frame,
        #                     start_configuration = start_configuration,
        #                     free_motion = True,
        #                     verbose = True)
        #             else:
        #                 trajectory = None

        #             # Post Planning
        #             if trajectory and trajectory.fraction == 1.0:
        #                 # Path planning succeeded
        #                 print("> > Motion Planned (%s pts, %.1f secs)" % (len(trajectory.points), trajectory.time_from_start))
        #                 print("> > Last Point: %s" % trajectory.points[-1])

        #                 # Assign Last frame of the path to next configuration.
        #                 last_configuration = Configuration(values=trajectory.points[-1].values, types=pp.joint_types, joint_names=pp.joint_names)

        #                 # Save trajectory to movement
        #                 movement.trajectory = trajectory
        #             else:
        #                 # Path planning failed.
        #                 print("> > Motion Plan Fail !!!")
        #                 last_configuration = None
        #         else:
        #             print("> > No Robotic Motion")
        #         print("")
        # print("> Total Path Plan Time: %.2fs" % (time.time() - t))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning, default False')
    parser.add_argument('-w', '--write', action='store_true', help='Write output json.')
    parser.add_argument('-db', '--debug', action='store_true', help='Debug mode')
    parser.add_argument('-si', '--seq_i', default=1, help='individual step to plan, Victor starts from one.')
    args = parser.parse_args()
    print('Arguments:', args)

    compute_movement(viewer=args.viewer, debug=args.debug, write=args.write, seq_i=int(args.seq_i))


if __name__ == '__main__':
    main()
