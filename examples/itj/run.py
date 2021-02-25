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

from compas.geometry import Scale

from compas.datastructures import Mesh
from compas.geometry import Transformation
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh, JointTrajectory, Duration, JointTrajectoryPoint

from pybullet_planning import wait_if_gui, wait_for_duration, wait_for_user, LockRenderer
from pybullet_planning import apply_alpha, RED, BLUE, YELLOW, GREEN, GREY

# HERE = os.path.dirname(__file__)
# if HERE not in sys.path:
#     sys.path.append(HERE)

from .parsing import parse_process
from .robot_setup import load_RFL_world, to_rlf_robot_full_conf, R11_INTER_CONF_VALS, R12_INTER_CONF_VALS, \
    RFL_SINGLE_ARM_JOINT_TYPES, rfl_robot_joint_names
from .utils import notify, MIL2M, convert_rfl_robot_conf_unit
from .stream import set_state, compute_linear_movement, compute_free_movement

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticMovement

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

def compute_movement(client, robot, process, movement, options=None):
    cprint(movement, 'cyan')
    if not (isinstance(movement, RoboticLinearMovement) or isinstance(movement, RoboticFreeMovement)):
        return None

    debug = options.get('debug') or False
    start_state = process.get_movement_start_state(movement)
    # end_state = process.get_movement_end_state(movement)
    set_state(client, robot, process, start_state)
    if debug:
        wait_if_gui('Start state')
    # set_state(client, robot, process, end_state)
    # wait_if_gui('End state')

    obstacles = []
    traj = None
    # TODO handle collision objects
    # TODO compile built elements as obstacles

    # TODO handle attached collision objects

    if isinstance(movement, RoboticLinearMovement):
        # linear movement has built-in kinematics sampler
        traj = compute_linear_movement(client, robot, process, movement, options)
        # type compas_fab : JointTrajectory
    elif isinstance(movement, RoboticFreeMovement):
        # free movement needs exterior samplers for start/end configurations
        traj = compute_free_movement(client, robot, process, movement, options)
        # type compas_fab : JointTrajectory
        pass
    else:
        raise ValueError()

    movement.trajectory = traj
    return movement

#################################

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--problem', default='twelve_pieces_process.json', # pavilion.json
                        help='The name of the problem to solve')
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning, default False')
    parser.add_argument('-si', '--seq_i', default=1, help='individual step to plan, Victor starts from one.')
    parser.add_argument('--write', action='store_true', help='Write output json.')
    parser.add_argument('--watch', action='store_true', help='Watch computed trajectories in the pybullet GUI.')
    parser.add_argument('--debug', action='store_true', help='Debug mode')
    parser.add_argument('--step_sim', action='store_true', help='Parse after each')
    # parser.add_argument('--disable_env', action='store_true', help='Disable environment collision geometry.')
    # parser.add_argument('-ptm', '--parse_transfer_motion', action='store_true', help='Parse saved transfer motion.')
    args = parser.parse_args()
    print('Arguments:', args)
    print('='*10)

    # * Connect to path planning backend and initialize robot parameters
    seq_i = seq_i=int(args.seq_i)
    client, robot, robot_uid = load_RFL_world(viewer=args.viewer, disable_env=True)

    process = parse_process(args.problem)
    assembly = process.assembly
    beam_ids = [b for b in process.assembly.sequence]
    beam_id = beam_ids[seq_i-1]
    cprint('Beam #{} | previous beams: {}'.format(beam_id, assembly.get_already_built_beams(beam_id)), 'cyan')

    # set all other unused robot
    full_start_conf = to_rlf_robot_full_conf(R11_INTER_CONF_VALS, R12_INTER_CONF_VALS)
    client.set_robot_configuration(robot, full_start_conf)
    # if args.debug:
    #     wait_if_gui('Pre Initial state.')

    # process.initial_state['robot'].kinematic_config = full_start_conf
    process.initial_state['robot'].kinematic_config = process.robot_initial_config
    set_state(client, robot, process, process.initial_state, initialize=True)
    # * collision sanity check
    assert not client.check_collisions(robot, full_start_conf, options={'diagnosis':True})
    # if args.debug:
    #     wait_if_gui('Initial state.')

    options = {
        'debug' : args.debug,
        'diagnosis' : True,
    }

    movements = process.get_movements_by_beam_id(beam_id)
    # p1_movements = process.get_movements_by_planning_priority(beam_id, 1)
    with LockRenderer(not args.debug):
        # for movement in movements:
        # ! let's just compute free and pick up gripper for now
        movements[0].end_state['robot'].kinematic_config = Configuration(values=R11_INTER_CONF_VALS,
            types=[2] + RFL_SINGLE_ARM_JOINT_TYPES, joint_names = rfl_robot_joint_names('robot11', include_gantry=True))

        compute_movement(client, robot, process, movements[2], options)
        movements[1].end_state['robot'].kinematic_config = movements[2].trajectory.points[0]
        compute_movement(client, robot, process, movements[1], options)

    # for updated_movement in process.get_movements_by_planning_priority(beam_id, 1):
    for m in movements:
        if isinstance(m, RoboticMovement):
            if m.trajectory is not None:
                cprint('Solution found for {} : {}'.format(m, m.trajectory), 'green')
                for jt_traj_pt in m.trajectory.points:
                    if args.watch:
                        client.set_robot_configuration(robot, jt_traj_pt) #, group=yzarm_move_group
                        if args.step_sim:
                            wait_if_gui('Step conf.')
                        else:
                            wait_for_duration(0.1)
            else:
                cprint('No solution found for {}'.format(m), 'red')
    p1_movements = process.get_movements_by_planning_priority(beam_id, 1)
    # * simulate ends
    client.disconnect()


if __name__ == '__main__':
    main()
