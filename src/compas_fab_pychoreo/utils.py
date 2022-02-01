import logging
import re
from collections.abc import Iterable
import copy
from click import option
import numpy as np
import pybullet_planning as pp
from .conversions import pose_from_frame

###########################################
# borrowed from: https://github.com/compas-dev/compas_fab/blob/3efe608c07dc5b08653ee4132a780a3be9fb93af/src/compas_fab/backends/pybullet/utils.py#L83
def get_logger(name):
    logger = logging.getLogger(name)

    try:
        from colorlog import ColoredFormatter
        formatter = ColoredFormatter("%(log_color)s%(levelname)-8s%(reset)s %(white)s%(message)s",
                                     datefmt=None,
                                     reset=True,
                                     log_colors={'DEBUG': 'cyan', 'INFO': 'green',
                                                 'WARNING': 'yellow',
                                                 'ERROR': 'red', 'CRITICAL': 'red',
                                                 }
                                     )
    except ImportError:
        formatter = logging.Formatter('[%(levelname)s] %(message)s')

    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(logging.DEBUG)

    return logger

LOGGER = get_logger(__name__)

###########################################

def values_as_list(dict_data : dict) -> list:
    vals = []
    for _, v in dict_data.items():
        if isinstance(v, Iterable):
            vals.extend(list(v))
        else:
            vals.append(v)
    return vals

################################################
# result postprocessing utils
# https://github.com/yijiangh/pychoreo/blob/dev/src/pychoreo/cartesian_planner/postprocessing.py
def divide_list_chunks(list, size_list):
    assert(sum(size_list) >= len(list))
    if sum(size_list) < len(list):
        size_list.append(len(list) - sum(size_list))
    for j in range(len(size_list)):
        cur_id = sum(size_list[0:j])
        yield list[cur_id:cur_id+size_list[j]]

###########################################

# TODO replace with options.get(key) or default_value
def is_valid_option(options, key, default_value):
    return default_value if options is None or key not in options else options[key]

def wildcard_keys(data, wildcard):
    # https://docs.python.org/3/library/re.html
    matched_keys = []
    for k in data.keys():
        # if re.search(wildcard, k):
        if re.match(wildcard, k):
            matched_keys.append(k)
    return matched_keys

############################################

def is_configurations_close(conf1, conf2, options=None, fallback_tol=1e-3):
    """Compare configurations using different tolerances for different joints.
    Return True if same, False if different.

    The default fallback_tol is 1e-3, an acceptable number for both prismatic joint (1 mm) and
    revolute joint (0.001 rad = 0.057 degree).

    Parameters
    ----------
    conf1 : Configuration
    conf2 : Configuration
    diff_tol_from_joint_names : dict
        {joint_name : joint diff tolerance}
    fallback_tol : float, optional
        fallback tolerance if joint tol is not specified in `diff_tol_from_joint_names`, by default 1e-3
    verbose : bool, optional
        Printout for each joint diff, by default True

    Returns
    -------
    [type]
        [description]
    """
    options = options or {}
    verbose = options.get('verbose', False)
    joint_compare_tolerances = options.get('joint_compare_tolerances', {})
    _conf1 =conf1.copy()
    _conf2 =conf2.copy()
    if conf1.joint_names != conf2.joint_names:
        if conf1.joint_names < conf2.joint_names:
            _conf1 = _conf2.merged(_conf1)
        else:
            _conf2 = _conf1.merged(_conf2)
    assert _conf1.joint_names == _conf2.joint_names
    joint_names = _conf1.joint_names
    for i, diff in enumerate(_conf1.iter_differences(_conf2)):
        # cprint('Joint #{} diff: {}'.format(joint_names[i], diff), 'yellow')
        tol = joint_compare_tolerances[joint_names[i]] if joint_names[i] in joint_compare_tolerances \
            else fallback_tol
        if abs(diff) > tol:
            if verbose:
                LOGGER.debug('Joint #{} diff: {:.4f} | tol: {:.4f}'.format(joint_names[i],
                    abs(diff), tol))
            return False
    return True

def does_configurations_jump(conf1, conf2, options=None, fallback_tol=1e-1):
    options = options or {}
    joint_jump_tolerances = options.get('joint_jump_tolerances', {})
    _options = options.copy()
    _options['joint_compare_tolerances'] = joint_jump_tolerances
    return not is_configurations_close(conf1, conf2, options=_options, fallback_tol=fallback_tol)

###############################################

def is_poses_close(pose0, pose1, options=None):
    options = options or {}
    frame_compare_distance_tolerance = options.get('frame_compare_distance_tolerance', 0.001) # meter
    frame_compare_axis_angle_tolerance = options.get('frame_compare_axis_angle_tolerance', 0.001) # rad
    verbose = options.get('verbose', False)
    (point0, quat0) = pose0
    (point1, quat1) = pose1
    R0 = pp.matrix_from_quat(quat0)
    R1 = pp.matrix_from_quat(quat1)
    if pp.get_distance(point0, point1) > frame_compare_distance_tolerance:
        if verbose:
            LOGGER.warning("Point distance: {:.6f} m | tol: {:.6f}".format(pp.get_distance(point0, point1), frame_compare_distance_tolerance))
        return False
    for i in range(3):
        if pp.angle_between(R0[:,i], R1[:,i]) > frame_compare_axis_angle_tolerance:
            if verbose:
                LOGGER.warning("Axis {} angle: {:.6f} | tol: {:.6f}".format(i, pp.angle_between(R0[:,i], R1[:,i]), frame_compare_axis_angle_tolerance))
            return False
    return True

def is_frames_close(frame0, frame1, options=None, scale=1.0):
    return is_poses_close(pose_from_frame(frame0, scale), pose_from_frame(frame1, scale), options)
