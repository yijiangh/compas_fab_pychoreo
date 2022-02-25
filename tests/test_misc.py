
# @pytest.mark.extend_fn
# def test_extend_fn(abb_irb4600_40_255_setup):
#     urdf_filename, semantics = abb_irb4600_40_255_setup

#     move_group = 'bare_arm'

#     attempts = 100
#     with PyChoreoClient(viewer=False) as client:
#         with LockRenderer():
#             robot = client.load_robot(urdf_filename)

#         ik_joint_names = robot.get_configurable_joint_names(group=move_group)

#         robot_uid = client.get_robot_pybullet_uid(robot)
#         joint_names = robot.get_configurable_joint_names(group=move_group)
#         conf_joints = joints_from_names(robot_uid, joint_names)

#         joint_resolutions = {jn : 0.3 for jn in ik_joint_names}
#         pb_joint_resolutions = None if len(joint_resolutions) == 0 else \
#             [joint_resolutions[joint_name] for joint_name in joint_names]

#         extend_fn = pp.get_extend_fn(robot_uid, conf_joints, resolutions=pb_joint_resolutions, norm=pp.INF)

#         # q0 = np.array([0.0, 0.0, 0.29, 0.0, 0.0, 0])
#         # q1 = np.array((0.0, 0.0, 0.0, 0.0, 0.0, 0))
#         # path = list(extend_fn(q0, q1))
#         # assert len(path) == 2

#         q0 = np.array([0.0, 0.0, 0.3, 0.0, 0.0, 0])
#         q1 = np.array((0.0, 0.0, 0.0, 0.0, 0.0, 0))
#         path = list(extend_fn(q0, q1))
#         assert len(path) == 2

#         q0 = np.array([0.0, 0.0, 0.31, 0.0, 0.0, 0])
#         q1 = np.array((0.0, 0.0, 0.0, 0.0, 0.0, 0))
#         path = list(extend_fn(q0, q1))
#         assert len(path) == 3

#         q0 = np.array([0.0, 0.0, 0.59, 0.0, 0.0, 1])
#         q1 = np.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.9))
#         path = list(extend_fn(q0, q1))
#         assert len(path) == 4

#         sample_fn = pp.get_sample_fn(robot_uid, conf_joints)

#         for _ in range(attempts):
#             q0 = sample_fn()
#             q1 = sample_fn()
#             path = list(extend_fn(q0, q1))
#             assert np.allclose(q0, path[0])
#             assert np.allclose(q1, path[-1])
#             for qt, qt1 in zip(path[:-1], path[1:]):
#                 diff = np.abs(np.array(qt1)-np.array(qt))
#                 assert pp.all_between(np.zeros(6), diff, pb_joint_resolutions)

