import pytest
import pybullet_planning as pp
# from compas_fab.backends import PyBulletClient
from compas_fab_pychoreo.client import PyChoreoClient

###################################################

@pytest.mark.client
def test_client(fixed_waam_setup, column_obstacle_cm, base_plate_cm, viewer):
    urdf_filename, semantics, _ = fixed_waam_setup
    move_group = 'robotA'

    # from compas_fab.backends import PyBulletClient
    # with PyBulletClient(connection_type="gui", verbose=False) as client:
    with PyChoreoClient(viewer=viewer, verbose=False) as client:
        with pp.LockRenderer():
            robot = client.load_robot(urdf_filename)
            robot.semantics = semantics

        robot_uid = client.get_robot_pybullet_uid(robot)
        # robot_uid = client.get_uid(robot)
        # robot_uid = client.get_uid(client.get_cached_robot(robot))

        tool_link_name = robot.get_end_effector_link_name(group=move_group)

        # * draw EE pose
        tool_link = pp.link_from_name(robot_uid, tool_link_name)
        tcp_pose = pp.get_link_pose(robot_uid, tool_link)
        pp.draw_pose(tcp_pose)

        assert robot_uid in pp.get_bodies()

        # * add static obstacles
        client.add_collision_mesh(base_plate_cm)
        client.add_collision_mesh(column_obstacle_cm) #, options={'color' : pp.RED})

        client._print_object_summary()

        pp.wait_if_gui()
