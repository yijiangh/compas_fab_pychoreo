import pytest
import os

from pybullet_planning import wait_if_gui, link_from_name, get_link_pose, draw_pose, get_bodies

from compas_fab_pychoreo.client import PyBulletClient, get_disabled_collisions
from compas_fab_pychoreo.conversions import pose_from_frame

@pytest.mark.client
def test_client(fixed_waam_setup, viewer):
    # https://github.com/gramaziokohler/algorithmic_details/blob/e1d5e24a34738822638a157ca29a98afe05beefd/src/algorithmic_details/accessibility/reachability_map.py#L208-L231
    urdf_filename, robot, _, _, _ = fixed_waam_setup
    print('disabled collision: ', get_disabled_collisions(robot.semantics))

    move_group = 'robotA'
    # base_link_name = robot.get_base_link_name(group=move_group)
    # ik_joint_names = robot.get_configurable_joint_names(group=move_group)
    tool_link_name = robot.get_end_effector_link_name(group=move_group)

    with PyBulletClient(viewer=viewer) as client:
        client.load_robot_from_urdf(urdf_filename)

        # robot_start_conf = [0,-np.pi/2,np.pi/2,0,0,0]
        # set_joint_positions(robot, ik_joints, robot_start_conf)

        # * draw EE pose
        tool_link = link_from_name(client.robot_uid, tool_link_name)
        tcp_pose = get_link_pose(client.robot_uid, tool_link)
        draw_pose(tcp_pose)

        assert client.robot_uid in get_bodies()

        wait_if_gui()
