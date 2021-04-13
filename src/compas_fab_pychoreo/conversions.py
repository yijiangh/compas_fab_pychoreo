from compas.geometry import Frame, Transformation

def pose_from_frame(frame, scale=1.0):
    """Returns a PyBullet pose from a frame.

    Parameters
    ----------
    frame : :class:`compas.geometry.Frame`

    Returns
    -------
    point, quaternion : tuple
    """
    return ([v*scale for v in frame.point], frame.quaternion.xyzw)


def frame_from_pose(pose, scale=1.0):
    """Returns a frame from a PyBullet pose.

    Parameters
    ----------
    point, quaternion : tuple

    Returns
    -------
    :class:`compas.geometry.Frame`
    """
    point, (x, y, z, w) = pose
    return Frame.from_quaternion([w, x, y, z], point=[v*scale for v in point])

def pose_from_transformation(tf, scale=1.0):
    frame = Frame.from_transformation(tf)
    return pose_from_frame(frame, scale)

def transformation_from_pose(pose, scale=1.0):
    frame = frame_from_pose(pose, scale)
    return Transformation.from_frame(frame)
