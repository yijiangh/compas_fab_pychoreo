import os
import tempfile
from compas.geometry import Frame

import numpy as np
from pybullet_planning import create_obj
from pybullet_planning import set_pose
from pybullet_planning import add_body_name
from pybullet_planning import joints_from_names


def pose_from_frame(frame):
    """Returns a PyBullet pose from a frame.

    Parameters
    ----------
    frame : :class:`compas.geometry.Frame`

    Returns
    -------
    point, quaternion : tuple
    """
    return (list(frame.point), frame.quaternion.xyzw)


def frame_from_pose(pose):
    """Returns a frame from a PyBullet pose.

    Parameters
    ----------
    point, quaternion : tuple

    Returns
    -------
    :class:`compas.geometry.Frame`
    """
    point, (x, y, z, w) = pose
    return Frame.from_quaternion([w, x, y, z], point=point)


def convert_mesh_to_body(mesh, frame, name=None):
    """Convert compas mesh and its frame to a pybullet body

    Parameters
    ----------
    mesh : compas Mesh
    frame : compas Frame
    name : str
        Optional, name of the mesh for tagging in pybullet's GUI

    Returns
    -------
    pybullet body
    """

    """
    vertices, faces = mesh.to_vertices_and_faces()
    indices = [i for f in faces for i in f] # flatten faces
    mesh_id = pybullet.createCollisionShape(pybullet.GEOM_MESH, vertices=vertices, indices=indices)
    point, quaternion = pose_from_frame(frame)
    obj_id = pybullet.createMultiBody(baseMass=0,
                                     baseCollisionShapeIndex=mesh_id,
                                     baseVisualShapeIndex=mesh_id,
                                     basePosition=point,
                                     baseOrientation=quaternion)


    if name:
        add_body_name(obj_id, name)

    return obj_id

    """
    with tempfile.TemporaryDirectory() as temp_dir:
        tmp_obj_path = os.path.join(temp_dir, 'temp.obj')
        mesh.to_obj(tmp_obj_path)
        pyb_body = create_obj(tmp_obj_path)
        body_pose = pose_from_frame(frame)
        set_pose(pyb_body, body_pose)
        if name:
            add_body_name(pyb_body, name)
    return pyb_body
    # """

def joint_values_from_configuration(c_conf, pb_robot, scale=1.0, joint_names=None):
    """Convert a compas_fab Configuration to pybullet conf

    Parameters
    ----------
    c_conf : compas_fab Configuration
        [description]
    pb_robot : int
        pybullet robot index
    scale : [type], optional
        [description], by default MIL2METER

    Returns
    -------
    [type]
        [description]
    """
    if scale != 1.0:
        # data copy here to avoid accidentally overwrite data
        scaled_c_conf = c_conf.scaled(scale)
        conf = np.array(scaled_c_conf.values)
    else:
        conf = np.array(c_conf.values)
    joints = joints_from_names(pb_robot, joint_names or scaled_c_conf.joint_names)
    return joints, conf
