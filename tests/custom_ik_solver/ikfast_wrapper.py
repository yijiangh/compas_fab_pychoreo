from compas_fab.robots import Configuration
from pybullet_planning import compute_inverse_kinematics
from compas_fab_pychoreo.conversions import pose_from_frame

def get_ik_fn_from_ikfast(ikfast_ik_fn):
    def compas_fab_ik_fn(frame_rcf, sampled=[]):
        tool_pose = pose_from_frame(frame_rcf)
        conf_vals = compute_inverse_kinematics(ikfast_ik_fn, tool_pose, sampled=sampled)
        return [Configuration.from_revolute_values(q) for q in conf_vals if q is not None and len(q) > 0]
    return compas_fab_ik_fn
