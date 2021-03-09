import os
import time
import numpy as np
import argparse
import json
import sys
from os import path

from termcolor import cprint
from copy import copy, deepcopy

from pybullet_planning import wait_if_gui, wait_for_duration, wait_for_user, LockRenderer
from pybullet_planning import apply_alpha, RED, BLUE, YELLOW, GREEN, GREY, WorldSaver, has_gui

from .parsing import parse_process, save_process_and_movements
from .robot_setup import load_RFL_world, to_rlf_robot_full_conf, R11_INTER_CONF_VALS, R12_INTER_CONF_VALS
from .utils import notify
from .stream import set_state, compute_free_movement, compute_linear_movement
from .visualization import visualize_movement_trajectory

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticMovement

# * NOW!!
# TODO gantry/joint value jump between cartesian movements
# TODO in transit/transfer, gripper hits the ground

# * Next steps
# TODO add clamp collision geometry to transit/transfer planning
# TODO use linkstatistics joint weight and resolutions

def print_title(x):
    print('\n\n')
    cprint(x, 'yellow', 'on_cyan', attrs=['bold'])

###########################################

HERE = os.path.dirname(__file__)
JSON_OUT_DIR = os.path.join(HERE, 'results')

def compute_movement(client, robot, process, movement, options=None):
    if not isinstance(movement, RoboticMovement):
        return None
    cprint(movement.short_summary, 'cyan')
    options = options or {}
    traj = None
    if isinstance(movement, RoboticLinearMovement):
        # * linear movement has built-in kinematics sampler
        # interpolation step size, in meter
        lm_options = options.copy()
        lm_options.update({'max_step' : 0.01, 'distance_threshold':0.0013})
        traj = compute_linear_movement(client, robot, process, movement, lm_options)
    elif isinstance(movement, RoboticFreeMovement):
        # * free movement needs exterior samplers for start/end configurations
        traj = compute_free_movement(client, robot, process, movement, options)
        pass
    else:
        raise ValueError()

    if traj is not None:
        # update start/end states
        movement.trajectory = traj
        start_state = process.get_movement_start_state(movement)
        start_state['robot'].kinematic_config = traj.points[0]
        end_state = process.get_movement_end_state(movement)
        end_state['robot'].kinematic_config = traj.points[-1]
    else:
        wait_for_user('Planning fails, press Enter to continue.')
    return movement

def propagate_states(process, sub_movements, all_movements):
    for target_m in sub_movements:
        if not isinstance(target_m, RoboticMovement) or target_m.trajectory is None:
            continue
        m_id = all_movements.index(target_m)
        target_start_state = process.get_movement_start_state(target_m)
        target_end_state = process.get_movement_end_state(target_m)
        target_start_conf = target_start_state['robot'].kinematic_config
        target_end_conf = target_end_state['robot'].kinematic_config
        print('~~~')
        print('Propagate states for ({}) : {}'.format(m_id, target_m.short_summary))
        # * backward fill all adjacent (-1) movements
        back_id = m_id-1
        while back_id > 0 and all_movements[back_id].planning_priority == -1:
            back_m = all_movements[back_id]
            print('past (backward): ({}) {}'.format(back_id, back_m.short_summary))
            back_start_state = process.get_movement_start_state(back_m)
            back_end_state = process.get_movement_end_state(back_m)
            back_end_conf = back_end_state['robot'].kinematic_config
            if back_end_conf is not None and not back_end_conf.close_to(target_start_conf, tol=1e-3):
                cprint('Start conf not coincided - max diff {}'.format(back_end_conf.max_difference(target_start_conf)), 'red')
                wait_for_user()
            back_end_state['robot'].kinematic_config = target_start_conf
            back_start_state['robot'].kinematic_config = target_start_conf
            back_id -= 1

            # tmp_back_start_state = process.get_movement_start_state(back_m)
            # tmp_back_end_state = process.get_movement_end_state(back_m)
            # assert tmp_back_start_state['robot'].kinematic_config.close_to(tmp_back_end_state['robot'].kinematic_config)

        # * forward fill all adjacent (-1) movements
        forward_id = m_id+1
        while forward_id < len(all_movements) and all_movements[forward_id].planning_priority == -1:
            forward_m = all_movements[forward_id]
            print('future (forward): ({}) {}'.format(forward_id, forward_m.short_summary))
            forward_start_state = process.get_movement_start_state(forward_m)
            forward_end_state = process.get_movement_end_state(forward_m)
            forward_start_conf = forward_start_state['robot'].kinematic_config
            if forward_start_conf is not None and not forward_start_conf.close_to(target_end_conf, tol=1e-3):
                cprint('Start conf not coincided - max diff {}'.format(back_end_conf.max_difference(target_end_conf)), 'red')
                wait_for_user()
            forward_start_state['robot'].kinematic_config = target_end_conf
            forward_end_state['robot'].kinematic_config = target_end_conf
            forward_id += 1

            # tmp_forward_start_state = process.get_movement_start_state(forward_m)
            # tmp_forward_end_state = process.get_movement_end_state(forward_m)
            # assert tmp_forward_start_state['robot'].kinematic_config.close_to(tmp_forward_end_state['robot'].kinematic_config)
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
    parser.add_argument('--diagnosis', action='store_true', help='Diagnosis mode')
    parser.add_argument('--step_sim', action='store_true', help='Pause after each conf viz.')
    parser.add_argument('--disable_env', action='store_true', help='Disable environment collision geometry.')
    parser.add_argument('--view_states', action='store_true', help='Visualize states.')
    parser.add_argument('--reinit_tool', action='store_true', help='Regenerate tool URDFs.')
    # parser.add_argument('-ptm', '--parse_transfer_motion', action='store_true', help='Parse saved transfer motion.')
    args = parser.parse_args()
    print('Arguments:', args)
    print('='*10)

    # * Connect to path planning backend and initialize robot parameters
    seq_i = int(args.seq_i)
    client, robot, _ = load_RFL_world(viewer=args.viewer or args.diagnosis or args.view_states)

    process = parse_process(args.problem)
    assembly = process.assembly
    beam_ids = [b for b in process.assembly.sequence]
    beam_id = beam_ids[seq_i]
    cprint('Beam #{} | previous beams: {}'.format(beam_id, assembly.get_already_built_beams(beam_id)), 'cyan')

    # set all other unused robot
    full_start_conf = to_rlf_robot_full_conf(R11_INTER_CONF_VALS, R12_INTER_CONF_VALS)
    client.set_robot_configuration(robot, full_start_conf)
    # wait_if_gui('Pre Initial state.')

    process.initial_state['robot'].kinematic_config = process.robot_initial_config
    set_state(client, robot, process, process.initial_state, initialize=True,
        options={'debug' : args.debug, 'include_env' : not args.disable_env, 'reinit_tool' : args.reinit_tool})
    # * collision sanity check
    assert not client.check_collisions(robot, full_start_conf, options={'diagnosis':True})

    options = {
        'debug' : args.debug,
        'diagnosis' : args.diagnosis,
    }

    all_movements = process.get_movements_by_beam_id(beam_id)

    if args.view_states:
        wait_if_gui('Initial state.')
        for i, m in enumerate(all_movements):
            start_state = process.get_movement_start_state(m)
            end_state = process.get_movement_end_state(m)
            print('----')
            cprint('{}) {}'.format(i, m.short_summary), 'cyan')
            set_state(client, robot, process, start_state, options=options)
            wait_if_gui('Start state')
            set_state(client, robot, process, end_state, options=options)
            wait_if_gui('End state')
        wait_for_user('Enter to exit.')

    # print_title('0) Before planning')
    # process.get_movement_summary_by_beam_id(beam_id)
    # if args.debug:
    #     wait_for_user()

    with LockRenderer(not (args.debug or args.diagnosis)):
        # * 1) compute Linear movements (priority 1)
        print_title('1) compute Linear movements (priority 1)')
        p1_movements = process.get_movements_by_planning_priority(beam_id, 1)
        for m in p1_movements:
            print('-'*10)
            m_id = all_movements.index(m)
            print('{})'.format(m_id))
            all_movements[m_id] = compute_movement(client, robot, process, m, options)
            if args.debug:
                with WorldSaver():
                    visualize_movement_trajectory(client, robot, process, m, step_sim=args.step_sim)
        # * 2) propagate (priority 1) movement's start/end state to adjacent (prio -1) movements
        propagate_states(process, p1_movements, all_movements)
        process.get_movement_summary_by_beam_id(beam_id)

        # * 3) compute linear movements with one state (start OR end) specified
        print_title('3) compute linear movements with one state (start OR end) specified')
        p0_movements = process.get_movements_by_planning_priority(beam_id, 0)
        altered_movements = []
        for m in p0_movements:
            if isinstance(m, RoboticLinearMovement):
                has_start_conf = process.movement_has_start_robot_config(m)
                has_end_conf = process.movement_has_end_robot_config(m)
                has_traj = m.trajectory is not None
                if has_start_conf and has_end_conf and not has_traj:
                    cprint('{} has both start, end conf specified, but no traj computer. This is BAD!!'.format(m), 'yellow')
                    wait_for_user()
                if not has_traj and (has_start_conf ^ has_end_conf): # XOR
                    print('-'*10)
                    m_id = all_movements.index(m)
                    print('{})'.format(m_id))
                    all_movements[m_id] = compute_movement(client, robot, process, m, options)
                    altered_movements.append(all_movements[m_id])
                    if args.debug:
                        with WorldSaver():
                            visualize_movement_trajectory(client, robot, process, m, step_sim=args.step_sim)
        propagate_states(process, altered_movements, all_movements)
        process.get_movement_summary_by_beam_id(beam_id)
        if args.debug:
            wait_for_user()

        # * 5) compute linear movements with neither start nor conf specified
        print_title('5) compute linear movements with NO state specified')
        p0_movements = process.get_movements_by_planning_priority(beam_id, 0)
        altered_movements = []
        for m in p0_movements:
            if isinstance(m, RoboticLinearMovement):
                has_start_conf = process.movement_has_start_robot_config(m)
                has_end_conf = process.movement_has_end_robot_config(m)
                has_traj = m.trajectory is not None
                if has_start_conf and has_end_conf and not has_traj:
                    cprint('{} has both start, end conf specified, but no traj computer. This is BAD!!'.format(m), 'yellow')
                    wait_for_user()
                if not has_traj and (not has_start_conf and not has_end_conf):
                    print('-'*10)
                    m_id = all_movements.index(m)
                    print('{})'.format(m_id))
                    all_movements[m_id] = compute_movement(client, robot, process, m, options)
                    altered_movements.append(all_movements[m_id])
                    if args.debug:
                        with WorldSaver():
                            visualize_movement_trajectory(client, robot, process, m, step_sim=args.step_sim)
                    # * 6) propagate to -1 movements
                    print_title('6) Propagate for no-spec: ({}) {}'.format(m_id, m.short_summary))
                    propagate_states(process, [m], all_movements)
        process.get_movement_summary_by_beam_id(beam_id)
        if args.debug:
            wait_for_user()

        # * 6) compute linear movements with one state (start OR end) specified
        print_title('6) compute linear movements with one state (start OR end) specified')
        p0_movements = process.get_movements_by_planning_priority(beam_id, 0)
        altered_movements = []
        for m in p0_movements:
            if isinstance(m, RoboticLinearMovement):
                has_start_conf = process.movement_has_start_robot_config(m)
                has_end_conf = process.movement_has_end_robot_config(m)
                has_traj = m.trajectory is not None
                if has_start_conf and has_end_conf and not has_traj:
                    cprint('{} has both start, end conf specified, but no traj computer. This is BAD!!'.format(m), 'yellow')
                    wait_for_user()
                if not has_traj and (has_start_conf ^ has_end_conf): # XOR
                    print('-'*10)
                    m_id = all_movements.index(m)
                    print('{})'.format(m_id))
                    all_movements[m_id] = compute_movement(client, robot, process, m, options)
                    altered_movements.append(all_movements[m_id])
                    if args.debug:
                        with WorldSaver():
                            visualize_movement_trajectory(client, robot, process, m, step_sim=args.step_sim)
        propagate_states(process, altered_movements, all_movements)
        process.get_movement_summary_by_beam_id(beam_id)
        if args.debug:
            wait_for_user()

        # * 7) compute all free movements, start and end should be both specified by now?
        print_title('7) Compute free move')
        for m in p0_movements:
            if isinstance(m, RoboticFreeMovement):
                print('-'*10)
                m_id = all_movements.index(m)
                print('{})'.format(m_id))
                all_movements[m_id] = compute_movement(client, robot, process, m, options)
                if args.debug:
                    with WorldSaver():
                        visualize_movement_trajectory(client, robot, process, m, step_sim=args.step_sim)
        process.get_movement_summary_by_beam_id(beam_id)
        if args.debug:
            wait_for_user()

    # * final visualization
    if args.watch:
        print('='*20)
        print_title('Visualize results')
        # client, _, _ = load_RFL_world(viewer=args.viewer or args.diagnosis or args.view_states)
        # set_state(client, robot, process, process.initial_state,
            # initialize=True,
            # options={'debug' : args.debug, 'include_env' : not args.disable_env, 'reinit_tool' : args.reinit_tool})
        set_state(client, robot, process, process.initial_state)
        for m in all_movements:
            visualize_movement_trajectory(client, robot, process, m, step_sim=args.step_sim)
        # client.disconnect()

    # * export computed movements
    if args.write:
        save_process_and_movements(args.problem, process, all_movements, overwrite=False, include_traj_in_process=False)

    client.disconnect()

if __name__ == '__main__':
    main()
