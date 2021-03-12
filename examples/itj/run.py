import os
import time
import numpy as np
import argparse
import json
import sys
from os import path
from enum import Enum, unique

from termcolor import cprint, colored
from copy import copy, deepcopy

from pybullet_planning import wait_if_gui, wait_for_duration, wait_for_user, LockRenderer
from pybullet_planning import apply_alpha, RED, BLUE, YELLOW, GREEN, GREY, WorldSaver, has_gui

from .parsing import parse_process, save_process_and_movements
from .robot_setup import load_RFL_world, to_rlf_robot_full_conf, R11_INTER_CONF_VALS, R12_INTER_CONF_VALS
from .utils import notify
from .stream import set_state, compute_free_movement, compute_linear_movement
from .state import set_state
from .visualization import visualize_movement_trajectory

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticMovement, RoboticClampSyncLinearMovement

# * Next steps
# TODO use linkstatistics joint weight and resolutions
# TODO further smoothing transit/transfer trajectories
# TODO backtrack in case of subsequent sampling cannot find a solution (linear movement with one end specified)

def print_title(x):
    print('\n\n')
    cprint(x, 'green', 'on_red', attrs=['bold'])

###########################################

@unique
class MovementStatus(Enum):
    incorrect_type = 0
    has_traj = 1
    one_sided = 2
    both_done = 3
    neither_done = 4

###########################################

def compute_movement(client, robot, process, movement, options=None):
    if not isinstance(movement, RoboticMovement):
        return None
    cprint(movement.short_summary, 'cyan')
    options = options or {}
    traj = None
    if isinstance(movement, RoboticLinearMovement) or \
       isinstance(movement, RoboticClampSyncLinearMovement):
        lm_options = options.copy()
        # * interpolation step size, in meter
        lm_options.update({
            'max_step' : 0.01,
            'distance_threshold':0.002,
            'gantry_attempts' : 100,
            })
        traj = compute_linear_movement(client, robot, process, movement, lm_options)
    elif isinstance(movement, RoboticFreeMovement):
        fm_options = options.copy()
        fm_options.update({
            'rrt_restarts' : 50,
            'rrt_iterations' : 50,
            'smooth_iterations': 100,
            # 'resolutions' : 0.1,
            'resolutions' : 0.05,
            'max_step' : 0.01
            })
        traj = compute_free_movement(client, robot, process, movement, fm_options)
    else:
        raise ValueError()

    if traj is not None:
        # update start/end states
        movement.trajectory = traj
        movement.path_from_link = traj.path_from_link
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
        print('\tPropagate states for ({}) : {}'.format(m_id, target_m.short_summary))
        # * backward fill all adjacent (-1) movements
        back_id = m_id-1
        while back_id > 0 and all_movements[back_id].planning_priority == -1:
            back_m = all_movements[back_id]
            print('\t- past (backward): ({}) {}'.format(colored(back_id, 'green'), back_m.short_summary))
            back_start_state = process.get_movement_start_state(back_m)
            back_end_state = process.get_movement_end_state(back_m)
            back_end_conf = back_end_state['robot'].kinematic_config
            if back_end_conf is not None and not back_end_conf.close_to(target_start_conf, tol=1e-3):
                cprint('Start conf not coincided - max diff {}'.format(back_end_conf.max_difference(target_start_conf)), 'red')
                wait_for_user()
            back_end_state['robot'].kinematic_config = target_start_conf
            back_start_state['robot'].kinematic_config = target_start_conf
            back_id -= 1

        # * forward fill all adjacent (-1) movements
        forward_id = m_id+1
        while forward_id < len(all_movements) and all_movements[forward_id].planning_priority == -1:
            forward_m = all_movements[forward_id]
            print('\t- future (forward): ({}) {}'.format(colored(forward_id, 'green'), forward_m.short_summary))
            forward_start_state = process.get_movement_start_state(forward_m)
            forward_end_state = process.get_movement_end_state(forward_m)
            forward_start_conf = forward_start_state['robot'].kinematic_config
            if forward_start_conf is not None and not forward_start_conf.close_to(target_end_conf, tol=1e-3):
                cprint('End conf not coincided - max diff {}'.format(back_end_conf.max_difference(target_end_conf)), 'red')
                wait_for_user()
            forward_start_state['robot'].kinematic_config = target_end_conf
            forward_end_state['robot'].kinematic_config = target_end_conf
            forward_id += 1

        return all_movements

def get_movement_status(process, m, movement_types):
    if not isinstance(m, RoboticMovement) or all([not isinstance(m, mt) for mt in movement_types]):
        return MovementStatus.incorrect_type
    has_start_conf = process.movement_has_start_robot_config(m)
    has_end_conf = process.movement_has_end_robot_config(m)
    has_traj = m.trajectory is not None
    # special warning
    if not isinstance(m, RoboticFreeMovement) and \
            has_start_conf and has_end_conf and not has_traj:
        cprint('{} has both start, end conf specified, but no traj computed. This is BAD!!'.format(m.short_summary), 'yellow')
        wait_for_user()
    # imply(has_traj, has_start_conf and has_end_conf)
    assert not has_traj or (has_start_conf and has_end_conf)
    if has_traj:
        return MovementStatus.has_traj
    else:
        if has_start_conf ^ has_end_conf:
            return MovementStatus.one_sided
        elif has_start_conf and has_end_conf:
            return MovementStatus.both_done
        elif not has_start_conf and not has_end_conf:
            return MovementStatus.neither_done

def compute_selected_movements(client, robot, process, beam_id, priority, movement_types, movement_status, options=None,
        viz_upon_found=False, step_sim=False):
    print('='*20)
    print_title('* compute {} (priority {}, status {})'.format([mt.__name__ for mt in movement_types], priority, movement_status))
    all_movements = process.get_movements_by_beam_id(beam_id)
    selected_movements = process.get_movements_by_planning_priority(beam_id, priority)
    for m in selected_movements:
        if get_movement_status(process, m, movement_types) == movement_status:
            altered_movements = []
            print('-'*10)
            m_id = all_movements.index(m)
            print('({})'.format(m_id))
            # all_movements[m_id] =
            compute_movement(client, robot, process, m, options)
            altered_movements.append(m)
            if viz_upon_found:
                with WorldSaver():
                    visualize_movement_trajectory(client, robot, process, m, step_sim=step_sim)

            if movement_status == MovementStatus.neither_done:
                m_next_id = m_id + 1
                while m_next_id < len(all_movements) \
                    and get_movement_status(process, all_movements[m_next_id], movement_status) == MovementStatus.neither_done \
                    and all_movements[m_next_id] in selected_movements:
                    print('({})'.format(m_next_id))
                    compute_movement(client, robot, process, all_movements[m_next_id], options)
                    altered_movements.append(all_movements[m_next_id])
                    m_next_id += 1

            # * propagate to -1 movements
            propagate_states(process, altered_movements, all_movements)

    process.get_movement_summary_by_beam_id(beam_id)

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
    parser.add_argument('--parse_temp', action='store_true', help='Parse temporary process file. Defaults to False.')
    parser.add_argument('--save_temp', action='store_true', help='Save a temporary process file. Defaults to False.')
    parser.add_argument('--viz_upon_found', action='store_true', help='Viz found traj immediately after found. Defaults to False.')
    args = parser.parse_args()
    print('Arguments:', args)
    print('='*10)

    # * Connect to path planning backend and initialize robot parameters
    seq_i = int(args.seq_i)
    client, robot, _ = load_RFL_world(viewer=args.viewer or args.diagnosis or args.view_states)

    process = parse_process(args.problem, parse_temp=args.parse_temp)
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
        options={'debug' : False, 'include_env' : not args.disable_env, 'reinit_tool' : args.reinit_tool})
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
            cprint('({}) {}'.format(i, m.short_summary), 'cyan')
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
    # with LockRenderer():
        # * compute Linear movements (priority 1)
        compute_selected_movements(client, robot, process, beam_id, 1, [RoboticLinearMovement, RoboticClampSyncLinearMovement],
            MovementStatus.neither_done,
            options=options, viz_upon_found=args.viz_upon_found, step_sim=args.step_sim)

        # * compute linear movements with one state (start OR end) specified
        compute_selected_movements(client, robot, process, beam_id, 0, [RoboticLinearMovement], MovementStatus.one_sided,
            options=options, viz_upon_found=args.viz_upon_found, step_sim=args.step_sim)

        # * compute linear movements with neither start nor conf specified
        compute_selected_movements(client, robot, process, beam_id, 0, [RoboticLinearMovement], MovementStatus.neither_done,
            options=options, viz_upon_found=args.viz_upon_found, step_sim=args.step_sim)

        # * compute linear movements with neither start nor conf specified
        compute_selected_movements(client, robot, process, beam_id, 0, [RoboticLinearMovement], MovementStatus.one_sided,
            options=options, viz_upon_found=args.viz_upon_found, step_sim=args.step_sim)

        # * compute all free movements, start and end should be both specified by now?
        compute_selected_movements(client, robot, process, beam_id, 0, [RoboticFreeMovement], MovementStatus.both_done,
            options=options, viz_upon_found=args.viz_upon_found, step_sim=args.step_sim)

    # * export computed movements
    if args.write:
        save_process_and_movements(args.problem, process, all_movements, overwrite=False, include_traj_in_process=False, save_temp=args.save_temp)

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

    client.disconnect()

if __name__ == '__main__':
    main()
