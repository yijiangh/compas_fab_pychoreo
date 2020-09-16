import os
import argparse
import random
import time
import datetime
import jsonpickle
from termcolor import cprint

from compas.datastructures import Mesh
from compas.geometry import Transformation
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh, JointTrajectory, Duration, JointTrajectoryPoint

from pybullet_planning import create_box, disconnect, add_data_path, connect, get_movable_joints, get_joint_positions, \
    sample_placement, set_pose, multiply, invert, set_joint_positions, pairwise_collision, inverse_kinematics, \
    get_link_pose, get_body_name, write_pickle, uniform_pose_generator, set_base_values, \
    load_pybullet, HideOutput, wait_for_user, draw_point, point_from_pose, has_gui, elapsed_time, \
    sub_inverse_kinematics, BodySaver, joints_from_names, draw_pose, unit_pose, set_camera_pose
from pybullet_planning import randomize, elapsed_time, apply_alpha, RED, BLUE, YELLOW, GREEN, GREY

from compas_fab_pychoreo.backend_features.pybullet_configuration_collision_checker import PybulletConfigurationCollisionChecker
from compas_fab_pychoreo.client import PyBulletClient
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo_examples.ik_solver import ik_abb_irb4600_40_255, InverseKinematicsSolver, get_ik_fn_from_ikfast
from compas_fab_pychoreo.utils import divide_list_chunks, values_as_list

from parsing import rfl_setup, itj_TC_PG500_cms, itj_TC_PG1000_cms, itj_rfl_obstacle_cms, itj_rfl_pipe_cms
from utils import to_rlf_robot_full_conf, rfl_camera, notify, R11_START_CONF_VALS, R12_START_CONF_VALS


HERE = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(HERE, 'data')
DATABASE_DIR = os.path.join(HERE, 'results')
IR_FILENAME = '{}_{}_ir.json'

############################################

def write_jsonpickle_file(data, file_path, indent=None):
    jsonpickle.set_encoder_options('json', sort_keys=True, indent=indent)
    with open(file_path, 'wb') as f:
        f.write(jsonpickle.encode(data))
        f.close()
    cprint('File saved to {}.'.format(file_path))

def save_inverse_reachability(robot, arm, grasp_type, tool_link, gripper_from_base_list):
    # TODO: store value of torso and roll joint for the IK database. Sample the roll joint.
    # TODO: hash the pr2 urdf as well
    filename = IR_FILENAME.format(grasp_type, arm)
    path = get_database_file(filename)
    data = {
        'filename': filename,
        'generated_time': str(datetime.datetime.now()),
        'robot': 'r12', #get_body_name(robot),
        # 'grasp_type': grasp_type,
        # 'arm': arm,
        # 'torso': get_group_conf(robot, 'torso'),
        # 'carry_conf': get_carry_conf(arm, grasp_type),
        'tool_link': tool_link,
        # 'ikfast': is_ik_compiled(),
        'gripper_from_base': gripper_from_base_list,
    }
    write_jsonpickle_file(data, path)

    # visualize base positions
    if has_gui():
        handles = []
        for gripper_from_base in gripper_from_base_list:
            handles.extend(draw_point(point_from_pose(gripper_from_base), color=(1, 0, 0)))
        wait_for_user()
    return path

#######################################################

def create_inverse_reachability(robot, body, table, arm, grasp_type, max_attempts=500, num_samples=500):
    tool_link = get_gripper_link(robot, arm)
    robot_saver = BodySaver(robot)
    gripper_from_base_list = []
    grasps = GET_GRASPS[grasp_type](body)

    start_time = time.time()
    while len(gripper_from_base_list) < num_samples:
        box_pose = sample_placement(body, table)
        set_pose(body, box_pose)
        grasp_pose = random.choice(grasps)
        gripper_pose = multiply(box_pose, invert(grasp_pose))
        for attempt in range(max_attempts):
            robot_saver.restore()
            base_conf = next(uniform_pose_generator(robot, gripper_pose)) #, reachable_range=(0., 1.)))
            #set_base_values(robot, base_conf)
            set_group_conf(robot, 'base', base_conf)
            if pairwise_collision(robot, table):
                continue
            grasp_conf = pr2_inverse_kinematics(robot, arm, gripper_pose) #, nearby_conf=USE_CURRENT)
            #conf = inverse_kinematics(robot, link, gripper_pose)
            if (grasp_conf is None) or pairwise_collision(robot, table):
                continue
            gripper_from_base = multiply(invert(get_link_pose(robot, tool_link)), get_base_pose(robot))
            #wait_for_user()
            gripper_from_base_list.append(gripper_from_base)
            print('{} / {} | {} attempts | [{:.3f}]'.format(
                len(gripper_from_base_list), num_samples, attempt, elapsed_time(start_time)))
            if has_gui():
                wait_for_user()
            break
        else:
            print('Failed to find a kinematic solution after {} attempts'.format(max_attempts))
    return save_inverse_reachability(robot, arm, grasp_type, tool_link, gripper_from_base_list)

#######################################################

class MockProblem(object):
    def __init__(self, robot, fixed=[], grasp_types=[]):
        self.robot = robot
        self.fixed = fixed
        self.grasp_types = grasp_types

def create_inverse_reachability2(robot, body, table, arm, grasp_type, max_attempts=500, num_samples=500):
    tool_link = get_gripper_link(robot, arm)
    problem = MockProblem(robot, fixed=[table], grasp_types=[grasp_type])
    placement_gen_fn = get_stable_gen(problem)
    grasp_gen_fn = get_grasp_gen(problem, collisions=True)
    ik_ir_fn = get_ik_ir_gen(problem, max_attempts=max_attempts, learned=False, teleport=True)
    placement_gen = placement_gen_fn(body, table)
    grasps = list(grasp_gen_fn(body))
    print('Grasps:', len(grasps))

    # TODO: sample the torso height
    # TODO: consider IK with respect to the torso frame
    start_time = time.time()
    gripper_from_base_list = []
    while len(gripper_from_base_list) < num_samples:
        # object pose
        [(p,)] = next(placement_gen)
        # grasp
        (g,) = random.choice(grasps)
        output = next(ik_ir_fn(arm, body, p, g), None)
        if output is None:
            print('Failed to find a solution after {} attempts'.format(max_attempts))
        else:
            (_, ac) = output
            [at,] = ac.commands
            at.path[-1].assign()
            gripper_from_base = multiply(invert(get_link_pose(robot, tool_link)), get_base_pose(robot))
            gripper_from_base_list.append(gripper_from_base)
            print('{} / {} [{:.3f}]'.format(
                len(gripper_from_base_list), num_samples, elapsed_time(start_time)))
            if has_gui():
                wait_for_user()
    return save_inverse_reachability(robot, arm, grasp_type, tool_link, gripper_from_base_list)

#######################################################

def retrieve_tcp_poses(file_path):
    with open(file_path , 'r') as f:
        json_str = f.read()
        process = jsonpickle.decode(json_str, keys=True) # type: RobotClampAssemblyProcess
        cprint ("{} loaded: json_str len: {}".format(file_path, len(json_str)), 'green')

    beam_ids = [b for b in process.assembly.sequence]

#######################################################

def main():
    parser = argparse.ArgumentParser()  # Automatically includes help
    # parser.add_argument('-arm', required=True)
    parser.add_argument('-mg', '--move_group', default='robot12_eaXYZ', help='manipulation group name in the SRDF.')
    parser.add_argument('-p', '--problem', default='rfl_assembly_process.json', help='name of the itj process file.')
    # parser.add_argument('-grasp', required=True)
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    args = parser.parse_args()
    print('Arguments:', args)

    # arm = args.arm
    # other_arm = get_other_arm(arm)
    # grasp_type = args.grasp

    urdf_filename, robot = rfl_setup()

    # Attached CM (Pipes around Robot)
    pipe_cms = itj_rfl_pipe_cms()
    attached_cm_pipe2 = AttachedCollisionMesh(pipe_cms[0], 'robot12_link_2', ['robot12_link_2'])
    attached_cm_pipe3 = AttachedCollisionMesh(pipe_cms[1], 'robot12_link_3', ['robot12_link_3'])

    with PyBulletClient(viewer=args.viewer) as client:
        # load robot
        client.load_robot_from_urdf(urdf_filename)
        client.compas_fab_robot = robot
        robot_uid = client.robot_uid

        # set up camera
        draw_pose(unit_pose(), length=1.)
        cam = rfl_camera()
        set_camera_pose(cam['target'], cam['location'])

        # set initial configurations
        full_start_conf = to_rlf_robot_full_conf(R11_START_CONF_VALS, R12_START_CONF_VALS)
        full_joints = joints_from_names(robot_uid, full_start_conf.joint_names)
        set_joint_positions(robot_uid, full_joints, full_start_conf.values)

        # attachment
        client.add_attached_collision_mesh(attached_cm_pipe2, BLUE)
        client.add_attached_collision_mesh(attached_cm_pipe3, BLUE)

        for o_cm in itj_rfl_obstacle_cms():
            client.add_collision_mesh(o_cm, GREY)

        # create_inverse_reachability(client, arm=arm, grasp_type=grasp_type)
        # create_inverse_reachability2(robot, box, table, arm=arm, grasp_type=grasp_type)

        wait_for_user('Finished.')

if __name__ == '__main__':
    main()
