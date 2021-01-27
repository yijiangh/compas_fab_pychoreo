from numpy import deg2rad, rad2deg
from termcolor import cprint

# unit conversion
MIL2M = 1e-3

##########################################

def convert_robot_conf_unit(conf_vals, length_scale=MIL2M, angle_unit='rad', prismatic_ids=range(0,2), revoluted_ids=range(2,8)):
    """angle_unit is the target angle unit to be converted into
    """
    if angle_unit == 'rad':
        angle_fn = deg2rad
    elif angle_unit == 'deg':
        angle_fn = rad2deg
    else:
        raise ValueError
    return [conf_vals[i]*length_scale for i in prismatic_ids] + [angle_fn(conf_vals[i]) for i in revoluted_ids]

def convert_rfl_robot_conf_unit(conf_vals, length_scale=MIL2M, angle_unit='rad'):
    assert len(conf_vals) == 2+6 or len(conf_vals) == 3+6
    prismatic_id_until = 1 if len(conf_vals) == 8 else 2
    return convert_robot_conf_unit(conf_vals, length_scale, angle_unit,
        prismatic_ids=range(0, prismatic_id_until+1), revoluted_ids=range(prismatic_id_until+1, len(conf_vals)))

##########################################

def notify(msg=''):
    try:
        from plyer import notification
        notification.notify(
            title='pybullet planning',
            message=msg,
            app_icon=None,  # e.g. 'C:\\icon_32x32.ico'
            timeout=10,  # seconds
        )
    except ImportError:
        cprint(msg, 'yellow')
