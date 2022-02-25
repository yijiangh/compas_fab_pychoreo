import os
import numpy as np
import json

from pybullet_planning import multiply, Pose, Euler

from compas.utilities import DataDecoder, DataEncoder
from compas_fab_pychoreo.utils import is_configurations_close

####################################################

def compute_circle_path(circle_center=np.array([2, 0, 0.2]), circle_r=0.2, angle_range=(-0.5*np.pi, 0.5*np.pi)):
    # generate a circle path to test IK and Cartesian planning
    ee_poses = []
    n_pt = int(abs(angle_range[1]-angle_range[0]) / (np.pi/180 * 5))
    for a in np.linspace(*angle_range, num=n_pt):
        pt = circle_center + circle_r*np.array([np.cos(a), np.sin(a), 0])
        circ_pose = multiply(Pose(point=pt, euler=Euler(yaw=a+np.pi/2)), Pose(euler=Euler(roll=np.pi*3/4)))
        # draw_pose(circ_pose, length=0.01)
        ee_poses.append(circ_pose)
    return ee_poses

###########################

def compute_trajectory_cost(trajectory, init_conf_val=np.zeros(6)):
    cost = np.linalg.norm(init_conf_val - np.array(trajectory.points[0].joint_values))
    for traj_pt1, traj_pt2 in zip(trajectory.points[:-1], trajectory.points[1:]):
        cost += np.linalg.norm(np.array(traj_pt1.joint_values) - np.array(traj_pt2.joint_values))
    return cost

def save_trajectory(traj, file_name='tmp_traj.json'):
    HERE = os.path.dirname(__file__)
    save_path = os.path.join(HERE, "data", file_name)
    with open(save_path, 'w') as f:
        json.dump(traj, f, cls=DataEncoder, indent=None, sort_keys=True)

def get_data_path():
    return os.path.join(os.path.dirname(__file__), 'data')

def parse_trajectory(file_name='tmp_traj.json'):
    save_path = os.path.join(get_data_path(), file_name)
    with open(save_path, 'r') as f:
        traj = json.load(f, cls=DataDecoder)
    return traj

def compare_trajectories(traj0, traj1, options=None):
    options = options or {}
    assert len(traj0.points) == len(traj1.points)
    for conf0, conf1 in zip(traj0.points, traj1.points):
        assert is_configurations_close(conf0, conf1, options=options)

