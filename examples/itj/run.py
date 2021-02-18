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

from compas.datastructures import Mesh
from compas.geometry import Transformation
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh, JointTrajectory, Duration, JointTrajectoryPoint

from pybullet_planning import wait_if_gui, wait_for_duration, wait_for_user
from pybullet_planning import apply_alpha, RED, BLUE, YELLOW, GREEN, GREY

# HERE = os.path.dirname(__file__)
# if HERE not in sys.path:
#     sys.path.append(HERE)

from .parsing import parse_process
from .robot_setup import load_RFL_world
from .utils import notify, MIL2M, convert_rfl_robot_conf_unit
from .stream import set_state, compute_linear_movement

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
    if not (isinstance(movement, RoboticLinearMovement) or isinstance(movement, RoboticFreeMovement)):
        return None
    start_state = process.get_movement_start_state(movement)
    end_state = process.get_movement_end_state(movement)

    # * visualize states
    set_state(client, robot, process, start_state)
    wait_if_gui('Start state')
    set_state(client, robot, process, end_state)
    wait_if_gui('End state')

    # TODO compile built elements as obstacles
    obstacles = []
    traj = None

    if isinstance(movement, RoboticLinearMovement):
        # linear movement has built-in kinematics sampler
        traj = compute_linear_movement(client, robot, process, movement)
        # type compas_fab : Trajectory
    elif isinstance(movement, RoboticFreeMovement):
        # free movement needs exterior samplers for start/end configurations
        # traj = compute_free_movement(client, robot, process, movement)
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
    parser.add_argument('-w', '--write', action='store_true', help='Write output json.')
    parser.add_argument('-ptm', '--parse_transfer_motion', action='store_true', help='Parse saved transfer motion.')
    parser.add_argument('--debug', action='store_true', help='Debug mode')
    parser.add_argument('-si', '--seq_i', default=1, help='individual step to plan, Victor starts from one.')
    parser.add_argument('-na', '--disable_attachment', action='store_true', help='Disable beam and pipe attachments.')
    parser.add_argument('--disable_env', action='store_true', help='Disable environment collision geometry.')
    args = parser.parse_args()
    print('Arguments:', args)

    # * Connect to path planning backend and initialize robot parameters
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
