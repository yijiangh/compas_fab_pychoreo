import re
from termcolor import cprint
from collections.abc import Iterable

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

def compare_configurations(conf1, conf2, diff_tol_from_joint_names, fallback_tol=1e-3, verbose=True):
    """Compare configurations using different tolerances for different joints.

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
    _conf1 =conf1.copy()
    _conf2 =conf2.copy()
    if conf1.joint_names != conf2.joint_names:
        if conf1.joint_names < conf2.joint_names:
            _conf1 = _conf2.merged(_conf1)
        else:
            _conf2 = _conf1.merged(_conf2)
    is_diff = False
    assert _conf1.joint_names == _conf2.joint_names
    joint_names = _conf1.joint_names
    for i, diff in enumerate(_conf1.iter_differences(_conf2)):
        # cprint('Joint #{} diff: {}'.format(joint_names[i], diff), 'yellow')
        tol = diff_tol_from_joint_names[joint_names[i]] if joint_names[i] in diff_tol_from_joint_names \
            else fallback_tol
        if abs(diff) > tol:
            if verbose:
                print('Joint #{} (revolution) jump: {:.4f} | tol: {:.4f}'.format(joint_names[i],
                    abs(diff), tol))
            is_diff = True
    return is_diff

