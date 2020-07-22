import math
from compas.geometry import Point
from compas.geometry import Vector
from compas.geometry import Frame
from compas.geometry import Rotation
from compas.geometry import Sphere
from compas.geometry import intersection_sphere_sphere
from compas.geometry import intersection_plane_circle
from compas.geometry import tangent_points_to_circle_xy


def forward_kinematics_spherical_wrist(p1, p2, p3, p4, joint_values):
    """Forward kinematics function for spherical wrist robots.

    Parameters
    ----------
    p1 : Point
    p2 : Point
    p3 : Point
    p4 : Point
    joint_values : list of float
        List of 6 joint values in radians.

    Returns
    -------
    :class:`compas.geometry.Frame`
    """

    a1, a2, a3, a4, a5, a6 = joint_values

    af1 = Frame.worldZX()
    af1.point = p1
    af2 = Frame.worldZX()
    af2.point = p2
    af3 = Frame.worldYZ()
    af3.point = p3
    af4 = Frame.worldZX()
    af4.point = p3
    af5 = Frame.worldYZ()
    af5.point = p3
    af6 = Frame.worldYZ()
    af6.point = p4

    axis_frames = [af1, af2, af3, af4, af5, af6]

    Rot1 = Rotation.from_axis_and_angle((0, 0, 1), -1 * a1)
    for i in range(0, 6):
        axis_frames[i].transform(Rot1)

    Rot2 = Rotation.from_axis_and_angle(axis_frames[0].zaxis, a2 + math.pi / 2, point=axis_frames[0].point)
    for i in range(1, 6):
        axis_frames[i].transform(Rot2)

    Rot3 = Rotation.from_axis_and_angle(axis_frames[1].zaxis, a3 - math.pi / 2, point=axis_frames[1].point)
    for i in range(2, 6):
        axis_frames[i].transform(Rot3)

    Rot4 = Rotation.from_axis_and_angle(axis_frames[2].zaxis, a4 * -1.0, axis_frames[2].point)
    for i in range(3, 6):
        axis_frames[i].transform(Rot4)

    Rot5 = Rotation.from_axis_and_angle(axis_frames[3].zaxis, a5, axis_frames[3].point)
    for i in range(4, 6):
        axis_frames[i].transform(Rot5)

    Rot6 = Rotation.from_axis_and_angle(axis_frames[4].zaxis, -1.0 * a6, axis_frames[4].point)
    for i in range(5, 6):
        axis_frames[i].transform(Rot6)

    return axis_frames[5]


def inverse_kinematics_spherical_wrist(p1, p2, p3, p4, target_frame):
    """Inverse kinematics function for spherical wrist robots.

    This is a modification of the *Lobster* tool for solving the inverse
    kinematics problem for a spherical wrist robots.

    https://www.grasshopper3d.com/forum/topics/lobster-reloaded?groupUrl=lobster&

    This is the instruction on how to determine the 4 points.
    https://api.ning.com/files/KRgE1yt2kgG2IF9H8sre4CWfIDL9ytv5WvVn54zdOx6HE84gDordaHzo0jqwP-Qhry7MyRQ4IQxY1p3cIkqEDj1FAVVR2Xg0/Lobster_IK.pdf

    check p2, p3, p4 must be in one XY plane!
    """
    end_frame = Frame(target_frame.point, target_frame.yaxis, target_frame.xaxis)

    wrist_offset = p4.x - p3.x
    lower_arm_length = p2.z - p1.z
    upper_arm_length = (p3 - p2).length
    pln_offset = math.fabs(p2.y - p1.y)
    axis4_offset_angle = math.atan2(p3.z - p2.z, p3.x - p2.x)
    wrist = end_frame.to_world_coords(Point(0, 0, wrist_offset))

    p1_proj = p1.copy()
    p1_proj.y = p2.y

    axis1_angles = []
    axis2_angles = []
    axis3_angles = []
    axis4_angles = []
    axis5_angles = []
    axis6_angles = []

    if pln_offset == 0:
        axis1_angle = -1 * math.atan2(wrist.y, wrist.x)
        if axis1_angle > math.pi:
            axis1_angle -= 2 * math.pi
        axis1_angles += [axis1_angle] * 4
        axis1_angle += math.pi
        if axis1_angle > math.pi:
            axis1_angle -= 2 * math.pi
        axis1_angles += [axis1_angle] * 4
    else:
        circle = (0, 0, 0), pln_offset
        point = (wrist.x, wrist.y, 0)
        t1, t2 = tangent_points_to_circle_xy(circle, point)
        a1 = Vector(0, 1, 0).angle_signed(t1, (0, 0, -1))
        a2 = Vector(0, 1, 0).angle_signed(t2, (0, 0, -1))
        axis1_angles += [a1] * 4
        axis1_angles += [a2] * 4

    for i in range(2):

        axis1_angle = axis1_angles[i * 4]
        Rot1 = Rotation.from_axis_and_angle([0, 0, 1], -1 * axis1_angle, point=[0, 0, 0])
        p1A = p1_proj.transformed(Rot1)
        elbow_dir = Vector(1, 0, 0).transformed(Rot1)
        sphere1 = Sphere(p1A, lower_arm_length)
        sphere2 = Sphere(wrist, upper_arm_length)
        elbow_frame = Frame(p1A, elbow_dir, [0, 0, 1])
        elbow_plane = (p1A, elbow_frame.normal)

        case, (center, radius, normal) = intersection_sphere_sphere(sphere1, sphere2)
        circle = ((center, normal), radius)

        intersect_pt1, intersect_pt2 = intersection_plane_circle(elbow_plane, circle)

        for j in range(2):
            if j == 0:
                elbow_pt = intersect_pt1
            else:
                elbow_pt = intersect_pt2
            elbow_pt = Point(*elbow_pt)

            elbowx, elbowy, elbowz = elbow_frame.to_local_coords(elbow_pt)
            wristx, wristy, wristz = elbow_frame.to_local_coords(wrist)

            axis2_angle = math.atan2(elbowy, elbowx)
            axis3_angle = math.pi - axis2_angle + math.atan2(wristy - elbowy, wristx - elbowx) - axis4_offset_angle

            for k in range(2):
                axis2_angles.append(-axis2_angle)
                axis3_angle_wrapped = -axis3_angle + math.pi
                while (axis3_angle_wrapped >= math.pi):
                    axis3_angle_wrapped -= 2 * math.pi
                while (axis3_angle_wrapped < -math.pi):
                    axis3_angle_wrapped += 2 * math.pi
                axis3_angles.append(axis3_angle_wrapped)

            for k in range(2):
                axis4 = wrist - elbow_pt
                axis4.transform(Rotation.from_axis_and_angle(elbow_frame.zaxis, -axis4_offset_angle))
                temp_frame = elbow_frame.copy()  # copy yes or no?
                temp_frame.transform(Rotation.from_axis_and_angle(temp_frame.zaxis, axis2_angle + axis3_angle))

                # // B = TempPlane
                axis4_frame = Frame(wrist, temp_frame.zaxis, temp_frame.yaxis * -1.0)
                axis6x, axis6y, axis6z = axis4_frame.to_local_coords(end_frame.point)

                axis4_angle = math.atan2(axis6y, axis6x)
                if k == 1:
                    axis4_angle += math.pi
                    if axis4_angle > math.pi:
                        axis4_angle -= 2 * math.pi

                axis4_angle_wrapped = axis4_angle + math.pi / 2
                while (axis4_angle_wrapped >= math.pi):
                    axis4_angle_wrapped -= 2 * math.pi
                while (axis4_angle_wrapped < -math.pi):
                    axis4_angle_wrapped += 2 * math.pi
                axis4_angles.append(axis4_angle_wrapped)

                axis5_frame = axis4_frame.copy()
                axis5_frame.transform(Rotation.from_axis_and_angle(axis4_frame.zaxis, axis4_angle))
                axis5_frame = Frame(wrist, axis5_frame.zaxis * -1, axis5_frame.xaxis)
                axis6x, axis6y, axis6z = axis5_frame.to_local_coords(end_frame.point)
                axis5_angle = math.atan2(axis6y, axis6x)
                axis5_angles.append(axis5_angle)

                axis6_frame = axis5_frame.copy()
                axis6_frame.transform(Rotation.from_axis_and_angle(axis5_frame.zaxis, axis5_angle))
                axis6_frame = Frame(wrist, axis6_frame.yaxis * -1, axis6_frame.zaxis)
                endx, endy, endz = axis6_frame.to_local_coords(end_frame.to_world_coords(Point(1, 0, 0)))
                axis6_angle = math.atan2(endy, endx)
                axis6_angles.append(axis6_angle)

    # for joint_values in zip(axis1_angles, axis2_angles, axis3_angles, axis4_angles, axis5_angles, axis6_angles):
    #    print(forward_kinematics_spherical_wrist(p1, p2, p3, p4, joint_values))

    return axis1_angles, axis2_angles, axis3_angles, axis4_angles, axis5_angles, axis6_angles
