import os
import time
import numpy as np
import argparse
import json
import sys
from os import path

from math import radians as rad
from termcolor import cprint
from itertools import product
from copy import copy, deepcopy

from compas.utilities import DataDecoder, DataEncoder

from pybullet_planning import wait_if_gui, wait_for_duration, wait_for_user, LockRenderer
from pybullet_planning import apply_alpha, RED, BLUE, YELLOW, GREEN, GREY, WorldSaver, has_gui

from .parsing import parse_process, save_process_and_movements
from .robot_setup import load_RFL_world, to_rlf_robot_full_conf, R11_INTER_CONF_VALS, R12_INTER_CONF_VALS
from .utils import notify
from .stream import set_state, compute_linear_movement, compute_free_movement
from .visualization import visualize_movement_trajectory

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticMovement

PREV_BEAM_COLOR = apply_alpha(RED, 1)
CUR_BEAM_COLOR = apply_alpha(GREEN, 1)

# TODO add clamp collision geometry to transit/transfer planning
# TODO use linkstatistics joint weight and resolutions

print_title = lambda x: cprint(x, 'red', 'on_cyan', attrs=['bold'])

###########################################

#TODO make argparse options
HERE = os.path.dirname(__file__)
JSON_OUT_DIR = os.path.join(HERE, 'results')

def compute_movement(client, robot, process, movement, options=None):
    cprint(movement, 'cyan')
    if not isinstance(movement, RoboticMovement):
        return None

    debug = options.get('debug') or False
    start_state = process.get_movement_start_state(movement)
    # end_state = process.get_movement_end_state(movement)
    set_state(client, robot, process, start_state)
    # if debug:
    #     wait_if_gui('Start state')
    # set_state(client, robot, process, end_state)
    # wait_if_gui('End state')

    traj = None
    if isinstance(movement, RoboticLinearMovement):
        # * linear movement has built-in kinematics sampler
        traj = compute_linear_movement(client, robot, process, movement, options)
    elif isinstance(movement, RoboticFreeMovement):
        # * free movement needs exterior samplers for start/end configurations
        traj = compute_free_movement(client, robot, process, movement, options)
        pass
    else:
        raise ValueError()

    if traj is not None:
        movement.trajectory = traj
        start_state = process.get_movement_start_state(movement)
        start_state['robot'].kinematic_config = traj.points[0]
        end_state = process.get_movement_end_state(movement)
        end_state['robot'].kinematic_config = traj.points[-1]
    return movement

def propagate_states(process, sub_movements, all_movements):
    for m in sub_movements:
        if m.trajectory is None:
            continue
        m_id = all_movements.index(m)
        start_state = process.get_movement_start_state(m)
        end_state = process.get_movement_end_state(m)
        print('~~~')
        print('Propagate states for ({}) : {}'.format(m_id, m))
        # * backward fill all adjacent (-1) movements
        back_id = m_id-1
        while back_id > 0 and all_movements[back_id].planning_priority == -1:
            back_m = all_movements[back_id]
            print('backward: ({}) {} - {}'.format(back_id, back_m, back_m.tag))
            process.set_movement_end_state(back_m, start_state, deep_copy=True)
            back_id -= 1

        # * forward fill all adjacent (-1) movements
        forward_id = m_id+1
        while forward_id < len(all_movements) and all_movements[forward_id].planning_priority == -1:
            forward_m = all_movements[forward_id]
            print('forward: ({}) {} - {}'.format(forward_id, forward_m, forward_m.tag))
            process.set_movement_start_state(forward_m, end_state, deep_copy=True)
            forward_id += 1
        return all_movements

#################################

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--problem', default='twelve_pieces_process.json', # pavilion.json
                        help='The name of the problem to solve')
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning, default False')
    parser.add_argument('-si', '--seq_i', default=0, help='individual step to plan.')
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
    seq_i = int(args.seq_i)
    client, robot, _ = load_RFL_world(viewer=args.viewer)

    process = parse_process(args.problem)
    assembly = process.assembly
    beam_ids = [b for b in process.assembly.sequence]
    beam_id = beam_ids[seq_i]
    cprint('Beam #{} | previous beams: {}'.format(beam_id, assembly.get_already_built_beams(beam_id)), 'cyan')

    # set all other unused robot
    full_start_conf = to_rlf_robot_full_conf(R11_INTER_CONF_VALS, R12_INTER_CONF_VALS)
    client.set_robot_configuration(robot, full_start_conf)
    # if args.debug:
    #     wait_if_gui('Pre Initial state.')

    process.initial_state['robot'].kinematic_config = process.robot_initial_config
    set_state(client, robot, process, process.initial_state, initialize=True, options={'debug' : False})
    # * collision sanity check
    assert not client.check_collisions(robot, full_start_conf, options={'diagnosis':True})
    # if args.debug:
    #     wait_if_gui('Initial state.')

    options = {
        'debug' : args.debug,
        'diagnosis' : False,
    }

    all_movements = process.get_movements_by_beam_id(beam_id)

    print_title('0) Before planning')
    process.get_movement_summary_by_beam_id(beam_id)

    with LockRenderer(not args.debug):
        # * 1) compute Linear movements (priority 1)
        print_title('1) compute Linear movements (priority 1)')
        p1_movements = process.get_movements_by_planning_priority(beam_id, 1)
        for m in p1_movements:
            print('-'*10)
            m_id = all_movements.index(m)
            print('{})'.format(m_id))
            all_movements[m_id] = compute_movement(client, robot, process, m, options)
            # if args.debug:
            #     with WorldSaver():
            #         visualize_movement_trajectory(client, robot, process, m, step_sim=args.step_sim)
        process.get_movement_summary_by_beam_id(beam_id)

        # * 2) propagate (priority 1) movement's start/end state to adjacent (prio -1) movements
        print_title('2) propagate')
        propagate_states(process, p1_movements, all_movements)
        process.get_movement_summary_by_beam_id(beam_id)

        # * 3) compute linear movements with one state (start OR end) specified
        print_title('3) compute linear movements with one state (start OR end) specified')
        p0_movements = process.get_movements_by_planning_priority(beam_id, 0)
        for m in p0_movements:
            if isinstance(m, RoboticLinearMovement):
                has_start_conf = process.movement_has_start_robot_config(m)
                has_end_conf = process.movement_has_end_robot_config(m)
                has_traj = m.trajectory is not None
                if has_start_conf and has_end_conf and not has_traj:
                    cprint('{} has both start, end conf specified, but no traj computer. This is BAD!!'.format(m), 'yellow')
                    wait_for_user()
                if not has_traj and (has_traj ^ has_end_conf): # XOR
                    print('-'*10)
                    m_id = all_movements.index(m)
                    print('{})'.format(m_id))
                    all_movements[m_id] = compute_movement(client, robot, process, m, options)
                    # if args.debug:
                    #     with WorldSaver():
                    #         visualize_movement_trajectory(client, robot, process, m, step_sim=args.step_sim)
        process.get_movement_summary_by_beam_id(beam_id)

        # * 4) propagate newly computed movement's start/end state to adjacent (prio -1) movements
        print_title('4) Propagate')
        propagate_states(process, p1_movements, all_movements)
        process.get_movement_summary_by_beam_id(beam_id)

        # * 5) compute all free movements, start and end should be both specified by now?
        print_title('5) Compute free move')
        for m in p0_movements:
            if isinstance(m, RoboticFreeMovement):
                print('-'*10)
                m_id = all_movements.index(m)
                print('{})'.format(m_id))
                all_movements[m_id] = compute_movement(client, robot, process, m, options)
                # if args.debug:
                #     with WorldSaver():
                #         visualize_movement_trajectory(client, robot, process, m, step_sim=args.step_sim)
        process.get_movement_summary_by_beam_id(beam_id)

    # * final visualization
    print('='*20)
    if args.watch:
        print_title('Visualize results')
        # for updated_movement in process.get_movements_by_planning_priority(beam_id, 1):
        for m in all_movements:
            visualize_movement_trajectory(client, robot, process, m, step_sim=args.step_sim)

    # export computed movements
    if args.write:
        save_process_and_movements(args.problem, process, all_movements, overwrite=False, include_traj_in_process=False)

    # * simulate ends
    client.disconnect()


if __name__ == '__main__':
    main()
