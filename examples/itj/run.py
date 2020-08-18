import os
import time
import numpy as np
import argparse
import jsonpickle
from termcolor import cprint
from itertools import product

from compas.geometry import Scale
from integral_timber_joints.process.algorithms import *
# from integral_timber_joints.process.pb_planning.pb_path_planner import PbRFLPathPlanner

import cProfile
import pstats

from compas.datastructures import Mesh
from compas.geometry import Transformation
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh

from pybullet_planning import link_from_name, get_link_pose, draw_pose, get_bodies, multiply, Pose, Euler, set_joint_positions, \
    joints_from_names, quat_angle_between, get_collision_fn, create_obj, unit_pose, set_camera_pose, pose_from_tform, set_pose
from pybullet_planning import wait_if_gui, wait_for_duration
from pybullet_planning import plan_cartesian_motion, plan_cartesian_motion_lg
from pybullet_planning import randomize, elapsed_time, apply_alpha, RED, BLUE, YELLOW, GREEN, GREY

from compas_fab_pychoreo.client import PyBulletClient
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo_examples.ik_solver import ik_abb_irb4600_40_255, InverseKinematicsSolver, get_ik_fn_from_ikfast

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

###########################################

HERE = os.path.dirname(os.path.abspath(__file__))

#TODO make argparse options
JSON_PATH_IN = os.path.join(HERE, 'data', "rfl_assembly_process.json")
JSON_OUT_DIR = os.path.join(HERE, 'results', "rfl_assembly_plan_results.json")

R11_START_CONF_VALS = np.array([22700.0, 0.0, -4900.0, 0.0, -80.0, 65.0, 65.0, 20.0, -20.0])
R12_START_CONF_VALS = np.array([-4056.0883789999998, -4000.8486330000001, 0.0, -22.834741999999999, -30.711554, 0.0, 57.335655000000003, 0.0])

def compute_movement(json_path_in=JSON_PATH_IN, json_out_dir=JSON_OUT_DIR, viewer=False, debug=False, write=False, seq_i=1):
    # * Connect to path planning backend and initialize robot parameters
    urdf_filename, robot = rfl_setup()

    move_group = 'robot12_eaYZ'
    arm_move_group = 'robot12'
    ik_joint_names = robot.get_configurable_joint_names(group=move_group)
    ik_joint_types = robot.get_joint_types_by_names(ik_joint_names)
    flange_link_name = robot.get_end_effector_link_name(group=move_group)

    ee_touched_link_names = ['robot12_tool0', 'robot12_link_6']
    ee_acms = [AttachedCollisionMesh(ee_cm, flange_link_name, ee_touched_link_names) for ee_cm in itj_TC_PG1000_cms()]

    # * Load process from file
    with open(json_path_in, 'r') as f:
        json_str = f.read()
        cprint ("{} loaded: json_str len: {}".format(json_path_in, len(json_str)), 'green')
        process = jsonpickle.decode(json_str, keys=True) # type: RobotClampAssemblyProcess
    assembly = process.assembly # For convinence

    with PyBulletClient(viewer=viewer) as client:
        client.load_robot_from_urdf(urdf_filename)
        client.compas_fab_robot = robot
        robot_uid = client.robot_uid

        draw_pose(unit_pose(), length=1.)
        cam = rfl_camera()
        set_camera_pose(cam['target'], cam['location'])

        # * attachments
        for ee_acm in ee_acms:
            client.add_attached_collision_mesh(ee_acm, YELLOW)

        # * Add static collision mesh to planning scene
        for o_cm in itj_rfl_obstacle_cms():
            client.add_collision_mesh(o_cm, GREY)

        # * start conf
        r11_start_conf_vals = np.array([22700.0, 0.0, -4900.0, 0.0, -80.0, 65.0, 65.0, 20.0, -20.0])
        r12_start_conf_vals = np.array([-4056.0883789999998, -4000.8486330000001, 0.0, -22.834741999999999, -30.711554, 0.0, 57.335655000000003, 0.0])
        full_start_conf = to_rlf_robot_full_conf(r11_start_conf_vals, r12_start_conf_vals)
        full_joints = joints_from_names(robot_uid, full_start_conf.joint_names )
        set_joint_positions(robot_uid, full_joints, full_start_conf.values)

        # * collision sanity check
        start_confval = np.hstack([r12_start_conf_vals[:2]*1e-3, np.radians(r12_start_conf_vals[2:])])
        conf = Configuration(values=start_confval.tolist(), types=ik_joint_types, joint_names=ik_joint_names)
        assert not client.configuration_in_collision(conf, group=move_group, options={'diagnosis':True})

        # * Individual process path planning
        toolchanger = process.robot_toolchanger
        beam_ids = [b for b in process.assembly.sequence]
        beam_id = beam_ids[seq_i-1]
        cprint('Beam #{} | previous beams: '.format(beam_id), 'cyan')

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
        client.add_attached_collision_mesh(cur_beam_acm, CUR_BEAM_COLOR)

        # transfer motion freeze x axis
        wait_if_gui()

        #####################################

        # * Sequentially copy movement target_frame to next movement source_frame
        _source_frame = None
        for ia, action in enumerate(process.actions):
            for im, movement in enumerate(action.movements):
                if isinstance(movement, RoboticMovement):
                    movement.source_frame = _source_frame
                    _source_frame = movement.target_frame

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

    # * Save Results
    # if write:
    #     with open(json_path_out, 'w') as f:
    #         json_str = jsonpickle.encode(process, keys=True)
    #         f.write(json_str)
    #         cprint ("Planned path saved to {}, out json_str len: {}".format(json_path_out, len(json_str)), 'green')


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
