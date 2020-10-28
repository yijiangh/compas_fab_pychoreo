import logging

###########################################

def values_as_list(dict_data):
    vals = []
    for _, v in dict_data.items():
        if isinstance(v, list):
            vals.extend(v)
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

def is_valid_option(options, key, default_value):
    return default_value if options is None or key not in options else options[key]
