import math
import numpy as np

from pybullet_planning.interfaces.env_manager.pose_transformation import unit_from_theta, point_from_pose
from pybullet_planning import CIRCULAR_LIMITS
from compas_fab.robots import JointConstraint

def set_constraints(arm_limits=[], ea_limits=[], group=None):
    if "11" in group:
        arm_joint_names = ['robot11_joint_1', 'robot11_joint_2', 'robot11_joint_3', 'robot11_joint_4', 'robot11_joint_5', 'robot11_joint_6']
    elif "12" in group:
        arm_joint_names = ['robot12_joint_1', 'robot12_joint_2', 'robot12_joint_3', 'robot12_joint_4', 'robot12_joint_5', 'robot12_joint_6']
    else:
        raise NotImplementedError
    joint_constraints = []
    for n, limits in zip(arm_joint_names, arm_limits):
        joint_constraints.append(JointConstraint(n, 0.0, math.radians(limits[1]), -math.radians(limits[0]), 1.0))
    return joint_constraints

ABB_IRB_4600_LIMITS = [(-179, 179), (-90, 150), (-180, 75), (-181, 181), (-95, 95), (-181, 181)]
# path_constraints = set_constraints(arm_limits=abb_irb_4600_limits, group=group)

def sample_reachable_rfl_base(point, reachable_range=(0.25, 1.0)):
    radius = np.random.uniform(*reachable_range)
    x, y = radius*unit_from_theta(np.random.uniform(-np.pi, np.pi)) + point[:2]
    yaw = np.random.uniform(*CIRCULAR_LIMITS)
    base_values = (x, y, yaw)
    #set_base_values(robot, base_values)
    return base_values

def uniform_rfl_base_pose_generator(gripper_pose, **kwargs):
    point = point_from_pose(gripper_pose)
    while True:
        base_values = sample_reachable_rfl_base(point, **kwargs)
        if base_values is None:
            break
        yield base_values
