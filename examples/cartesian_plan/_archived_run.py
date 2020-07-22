import os
import sys
import numpy as np
import random
import argparse
from termcolor import cprint
import time
from itertools import product

import matplotlib
import matplotlib.pyplot as plt

from pybullet_planning import BASE_LINK
from pybullet_planning import load_pybullet, connect, wait_for_user, LockRenderer, has_gui, WorldSaver, HideOutput, \
    reset_simulation, disconnect, set_camera_pose, has_gui, set_camera, wait_for_duration, wait_if_gui
from pybullet_planning import Pose, Point, Euler, unit_point
from pybullet_planning import multiply, invert
from pybullet_planning import create_obj, create_attachment, Attachment
from pybullet_planning import link_from_name, get_link_pose, get_moving_links, get_link_name, get_disabled_collisions, \
    get_body_body_disabled_collisions, has_link, are_links_adjacent, get_custom_limits
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints, set_joint_positions, joint_from_name, \
    joints_from_names, get_sample_fn, plan_joint_motion, plan_cartesian_motion
from pybullet_planning import inverse_kinematics, sample_tool_ik, interval_generator
from pybullet_planning import dump_world, set_pose, draw_pose, elapsed_time
from pybullet_planning import get_collision_fn, get_floating_body_collision_fn, expand_links, create_box
from pybullet_planning import plan_cartesian_motion_lg

# ikfast module, compiled by ikfast_pybind
import ikfast_kuka_kr6_r900

HERE = os.path.dirname(__file__)
KUKA_ROBOT_URDF = os.path.join(HERE, '..', 'data', 'kuka_kr6_r900', 'urdf', 'kuka_kr6_r900.urdf')

EE_LINK_NAME = 'robot_tool0'
ROBOT_BASE_LINK_NAME = 'robot_base_link'

def trim_axs(axs, N):
    """Reduce *axs* to *N* Axes. All further Axes are removed from the figure.
    """
    axs = axs.flat
    for ax in axs[N:]:
        ax.remove()
    return axs[:N]

def plot_joint_val(path, lower_limits, upper_limits, method=''):
    confs = np.array(path)
    nT, nJ = np.shape(confs)
    t = list(range(nT))

    figsize = (10, 10)
    cols = 2
    rows = nJ // cols + 1
    fig, axes = plt.subplots(rows, cols, figsize=figsize)
    axes = trim_axs(axes, nJ)
    for i in range(nJ):
        ax = axes[i]
        l_limit = [lower_limits[i] for _ in t]
        u_limit = [upper_limits[i] for _ in t]
        ax.plot(t, confs[:, i], marker='.', label='Joint-'+str(i))
        ax.plot(t, l_limit, label='lower limit')
        ax.plot(t, u_limit, label='upper limit')
        ax.set(ylabel='Jt val/rad', title='J{}'.format(i))
        ax.grid()
        # ax.legend()
        if i == 0:
            fig.suptitle('{} : Joint val evolution'.format(method), fontsize=12)

    fig.savefig(os.path.join(HERE, 'images', 'joint_val-{}.png'.format(method)))

def descartes_demo(viewer=True, scaling_test=False, draw_graph=True):
    connect(use_gui=viewer)
    with HideOutput():
        robot = load_pybullet(KUKA_ROBOT_URDF, fixed_base=True)

    # * adjust camera pose (optional)
    # has_gui checks if the GUI mode is enabled
    if has_gui():
        camera_base_pt = (0,0,0)
        camera_pt = np.array(camera_base_pt) + np.array([1, -0.5, 1])
        set_camera_pose(tuple(camera_pt), camera_base_pt)

    cprint('Hello robot world! <ctrl+left mouse> to pan', 'green')
    wait_if_gui()

    ik_joints = get_movable_joints(robot)
    ik_joint_names = get_joint_names(robot, ik_joints)
    print('ik joint names: {}'.format(ik_joint_names))

    lower_limits, upper_limits = get_custom_limits(robot, ik_joints)
    print('joint lower limit: {}'.format(lower_limits))
    print('joint upper limit: {}'.format(upper_limits))
    # we can also read these velocities from the SRDF file (using e.g. COMPAS_FAB)
    # I'm a bit lazy, just handcode the values here
    vel_limits = {0 : 6.28318530718,
                  1 : 5.23598775598,
                  2 : 6.28318530718,
                  3 : 6.6497044501,
                  4 : 6.77187749774,
                  5 : 10.7337748998}

    # set the robot to a "comfortable" start configuration, optional
    robot_start_conf = [0,-np.pi/2,np.pi/2,0,0,0]
    set_joint_positions(robot, ik_joints, robot_start_conf)

    tool_link = link_from_name(robot, EE_LINK_NAME)
    robot_base_link = link_from_name(robot, ROBOT_BASE_LINK_NAME)
    # tool_from_root = get_relative_pose(robot, root_link, tool_link)

    # * draw EE pose
    if has_gui() :
        tcp_pose = get_link_pose(robot, tool_link)
        draw_pose(tcp_pose)

    circle_center = np.array([0.6, 0, 0.2])
    circle_r = 0.1
    # * generate multiple circles
    ee_poses = []
    full_angle = 2*2*np.pi
    # total num of path pts, one path point per 5 degree
    n_pt = int(full_angle / (np.pi/180 * 5))
    # full_angle = np.pi
    for a in np.linspace(0.0, full_angle, num=n_pt):
        pt = circle_center + circle_r*np.array([np.cos(a), np.sin(a), 0])
        circ_pose = multiply(Pose(point=pt, euler=Euler(yaw=a+np.pi/2)), Pose(euler=Euler(roll=np.pi*3/4)))
        draw_pose(circ_pose, length=0.01)
        ee_poses.append(circ_pose)

    # def get_ee_sample_fn(roll_gen, pitch_gen, yaw_gen):
    #     def ee_sample_fn(ee_pose):
    #         # a finite generator
    #         for roll, pitch, yaw in product(roll_gen, pitch_gen, yaw_gen):
    #             yield multiply(ee_pose, Pose(euler=Euler(roll=roll, pitch=pitch, yaw=yaw)))
    #     return ee_sample_fn
    # roll_sample_size = 3
    # pitch_sample_size = 3
    # yaw_sample_size = 1
    # delta_roll = np.pi/12
    # delta_pitch = np.pi/12
    # roll_gen = np.linspace(-delta_roll, delta_roll, num=roll_sample_size)
    # pitch_gen = np.linspace(-delta_pitch, delta_pitch, num=pitch_sample_size)
    # yaw_gen = np.linspace(0.0, 2*np.pi, num=yaw_sample_size)
    # ee_sample_fn = get_ee_sample_fn(roll_gen, pitch_gen, yaw_gen)

    # for ep in ee_sample_fn(ee_poses[0]):
    #     draw_pose(ep, length=0.03)

    # * baseline, keeping the EE z axis rotational dof fixed
    st_time = time.time()
    path = plan_cartesian_motion(robot, robot_base_link, tool_link, ee_poses)
    print('Solving time: {}'.format(elapsed_time(st_time)))
    if path is None:
        cprint('Gradient-based ik cartesian planning cannot find a plan!', 'red')
    else:
        cprint('Gradient-based ik cartesian planning find a plan!', 'green')
        time_step = 0.03
        for conf in path:
            set_joint_positions(robot, ik_joints, conf)
            wait_for_duration(time_step)
        if draw_graph:
            print('drawing graph...')
            plot_joint_val(path, lower_limits, upper_limits, method='GradIK')
    print('='*20)

    # * Now, let's try if using ladder graph without releasing the ee dof can give us a good trajectory
    # you should be doing something similar in your picture, right?

    # First, we will need ikfast to obtain ik solution variance, same in Descartes
    ik_fn = ikfast_kuka_kr6_r900.get_ik

    # we have to specify ik fn wrapper and feed it into pychoreo
    def get_sample_ik_fn(robot, ik_fn, robot_base_link, ik_joints, tool_from_root=None):
        def sample_ik_fn(world_from_tcp):
            if tool_from_root:
                world_from_tcp = multiply(world_from_tcp, tool_from_root)
            return sample_tool_ik(ik_fn, robot, ik_joints, world_from_tcp, robot_base_link, get_all=True)
        return sample_ik_fn
    # ik generation function stays the same for all cartesian processes
    sample_ik_fn = get_sample_ik_fn(robot, ik_fn, robot_base_link, ik_joints)

    # we ignore self collision in this tutorial, the collision_fn only considers joint limit now
    # See : https://github.com/yijiangh/pybullet_planning/blob/dev/tests/test_collisions.py
    # for more info on examples on using collision function
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[],
                                       attachments=[], self_collisions=False,
                                    #    disabled_collisions=disabled_collisions,
                                    #    extra_disabled_collisions=extra_disabled_collisions,
                                       custom_limits={})

    # Let's check if our ik sampler is working properly
    # uncomment the following if you want to play with this
    # for p in ee_poses:
    #     print('-'*5)
    #     pb_q = inverse_kinematics(robot, tool_link, p)
    #     if pb_q is None:
    #         cprint('pb ik can\'t find an ik solution', 'red')
    #     qs = sample_ik_fn(p)
    #     if qs is not None:
    #         cprint('But Ikfast does find one! {}'.format(qs[0]), 'green')
    #         # set_joint_positions(robot, ik_joints, qs[0])
    #         # wait_if_gui()
    #     else:
    #         cprint('ikfast can\'t find an ik solution', 'red')

    # * Ok, now we have the ik sampler and collision function ready, let's see if we can find a valid cartesian trajectory!
    ee_vel = 0.005 # m/s

    # st_time = time.time()
    # path, cost = plan_cartesian_motion_lg(robot, ik_joints, ee_poses, sample_ik_fn, collision_fn, \
    #     custom_vel_limits=vel_limits, ee_vel=ee_vel)
    # print('Solving time: {}'.format(elapsed_time(st_time)))
    # if path is None:
    #     cprint('ladder graph (w/o releasing dof) cartesian planning cannot find a plan!', 'red')
    # else:
    #     cprint('ladder graph (w/o releasing dof) cartesian planning find a plan!', 'green')
    #     cprint('Cost: {}'.format(cost), 'yellow')
    #     time_step = 0.03
    #     for conf in path:
    #         # cprint('conf: {}'.format(conf))
    #         set_joint_positions(robot, ik_joints, conf)
    #         wait_for_duration(time_step)
    #     if draw_graph:
    #         print('drawing graph...')
    #         plot_joint_val(path, lower_limits, upper_limits, method='IKFast+LadderGraph')
    # print('='*20)

    # * Now, let's see if releasing EE z axis dof can bring down the cost
    # First, let's build an end effector pose sampler to release the EE z axis dof!
    ## * EE z axis dof generator
    def get_ee_sample_fn(roll_gen, pitch_gen, yaw_gen):
        def ee_sample_fn(ee_pose):
            # a finite generator
            for roll, pitch, yaw in product(roll_gen, pitch_gen, yaw_gen):
                yield multiply(ee_pose, Pose(euler=Euler(roll=roll, pitch=pitch, yaw=yaw)))
        return ee_sample_fn

    # by increasing this number we can see the cost go down
    roll_sample_size = 10
    pitch_sample_size = 10
    yaw_sample_size = 10
    delta_roll = np.pi/6
    delta_pitch = np.pi/6
    delta_yaw = np.pi/6
    roll_gen = np.linspace(-delta_roll, delta_roll, num=roll_sample_size)
    pitch_gen = np.linspace(-delta_pitch, delta_pitch, num=pitch_sample_size)
    yaw_gen = np.linspace(-delta_yaw, delta_yaw, num=yaw_sample_size)
    ee_sample_fn = get_ee_sample_fn(roll_gen, pitch_gen, yaw_gen)

    st_time = time.time()
    path, cost = plan_cartesian_motion_lg(robot, ik_joints, ee_poses, sample_ik_fn, collision_fn, sample_ee_fn=ee_sample_fn, \
        custom_vel_limits=vel_limits, ee_vel=ee_vel)
    print('Solving time: {}'.format(elapsed_time(st_time)))

    # TODO: plot ee z axis deviation plot

    # the ladder graph cost is just summation of all adjacent joint difference
    # so the following assertion should be true
    # conf_array = np.array(path)
    # conf_diff = np.abs(conf_array[:-1,:] - conf_array[1:,:])
    # np_cost = np.sum(conf_diff)
    # assert np.allclose(np_cost, cost), '{} - {}'.format(np_cost, cost)

    if path is None:
        cprint('ladder graph (releasing EE z dof) cartesian planning cannot find a plan!', 'red')
    else:
        cprint('ladder graph (releasing EE z dof) cartesian planning find a plan!', 'cyan')
        cprint('Cost: {}'.format(cost), 'yellow')
        time_step = 0.03
        for conf in path:
            # cprint('conf: {}'.format(conf))
            set_joint_positions(robot, ik_joints, conf)
            wait_for_duration(time_step)
        if draw_graph:
            print('drawing graph...')
            plot_joint_val(path, lower_limits, upper_limits, method='IKFast+LadderGraph+release z dof-res{}'.format(yaw_sample_size))
    print('='*20)

    # Now, let's do a scaling test:
    # if scaling_test:
        # res_sc = list(range(4,30))
        # cost_sc = []
        # for res in res_sc:
        #     yaw_sample_size = res
        #     yaw_gen = np.linspace(0.0, 2*np.pi, num=yaw_sample_size)
        #     ee_sample_fn = get_ee_sample_fn(yaw_gen)
        #     path, cost = plan_cartesian_motion_lg(robot, ik_joints, ee_poses, sample_ik_fn, collision_fn, sample_ee_fn=ee_sample_fn)
        #     assert path is not None
        #     cost_sc.append(cost)
        #     if res % 5 == 0:
        #         if draw_graph:
        #             plot_joint_val(path, lower_limits, upper_limits, method='IKFast+LadderGraph+release z dof-res{}'.format(yaw_sample_size))

        # fig, ax = plt.subplots()
        # ax.plot(res_sc, cost_sc)
        # ax.set(xlabel='yaw angle discr resolution', ylabel='total joint difference cost',
        #        title='Total joint diff scaling test')
        # ax.grid()
        # fig.savefig(os.path.join(HERE, 'images', 'tota_diff-scaling.png'))
        # # plt.show()

    wait_if_gui('Press enter to exit')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-nv', '--noviewer', action='store_true', help='Enables the viewer during planning, default True')
    parser.add_argument('-sc', '--scale_test', action='store_true', help='Scaling test')
    parser.add_argument('-dg', '--draw_graph', action='store_true', help='Draw joint evolution graph')
    # parser.add_argument('-d', '--demo', default='UR', choices=TUTORIALS, \
    #     help='The name of the demo')
    args = parser.parse_args()
    print('Arguments:', args)

    descartes_demo(viewer=not args.noviewer, scaling_test=args.scale_test, draw_graph=args.draw_graph)


if __name__ == '__main__':
    main()
