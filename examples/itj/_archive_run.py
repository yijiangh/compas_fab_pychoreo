    # yzarm_move_group = 'robot12_eaYZ'
    # yzarm_joint_names = robot.get_configurable_joint_names(group=yzarm_move_group)
    # yzarm_joint_types = robot.get_joint_types_by_names(yzarm_joint_names)
    # flange_link_name = robot.get_end_effector_link_name(group=yzarm_move_group)

    # gantry_x_joint_name = 'bridge1_joint_EA_X'
    # custom_x_limits = {gantry_x_joint_name : GANTRY_X_LIMIT}
    # custom_yz_limits = {}

    # gantry_joint_names = []
    # for jt_n, jt_t in zip(yzarm_joint_names, yzarm_joint_types):
    #     if 'Y' in jt_n:
    #         custom_yz_limits[jt_n] = GANTRY_Y_LIMIT
    #     elif 'Z' in jt_n:
    #         custom_yz_limits[jt_n] = GANTRY_Z_LIMIT
    #     if jt_t == 2:
    #         gantry_joint_names.append(jt_n)

    # gantry_x_joint = joint_from_name(robot_uid, gantry_x_joint_name)
    # gantry_joints = joints_from_names(robot_uid, gantry_joint_names)

    # # * custom limits
    # gantry_x_sample_fn = get_sample_fn(robot_uid, [gantry_x_joint],
    #     custom_limits={joint_from_name(robot_uid, jn) : limits for jn, limits in custom_x_limits.items()})
    # gantry_yz_sample_fn = get_sample_fn(robot_uid, gantry_joints,
    #     custom_limits={joint_from_name(robot_uid, jn) : limits for jn, limits in custom_yz_limits.items()})

    # yzarm_joints = joints_from_names(robot_uid, yzarm_joint_names)

    # # * attachments
    # gripper_type = assembly.get_beam_attribute(beam_id, 'gripper_type')
    # ee_touched_link_names = ['robot12_tool0', 'robot12_link_6']
    # ee_cms_fn = itj_TC_PG1000_cms if 'PG1000' in gripper_type else itj_TC_PG500_cms
    # ee_acms = [AttachedCollisionMesh(ee_cm, flange_link_name, ee_touched_link_names) for ee_cm in ee_cms_fn()]
    # cprint('Using gripper {}'.format(gripper_type), 'yellow')

    # for ee_acm in ee_acms:
    #     # ! note that options must contain 'robot' and 'mass' entries
    #     client.add_attached_collision_mesh(ee_acm, options={'robot': robot, 'mass': 1, 'color': YELLOW})

    # # * Individual process path planning
    # def flange_frame_at(subprocess_name):
    #     gripper_t0cp = process.get_gripper_t0cp_for_beam_at(beam_id, subprocess_name)
    #     toolchanger.set_current_frame_from_tcp(gripper_t0cp)
    #     return toolchanger.current_frame.copy()

    # if process.get_gripper_t0cp_for_beam_at(beam_id, 'assembly_wcf_inclampapproach'):
    #     assembly_wcf_inclampapproach =  flange_frame_at('assembly_wcf_inclampapproach')
    # else:
    #     assembly_wcf_inclampapproach =  flange_frame_at('assembly_wcf_inclamp')
    # assembly_wcf_inclamp =  flange_frame_at('assembly_wcf_inclamp')
    # assembly_wcf_final =  flange_frame_at('assembly_wcf_final')
    # assembly_wcf_finalretract =  flange_frame_at('assembly_wcf_finalretract')

    # # these tcp are all for flange frame
    # wcf_inclamp_approach_pose = multiply(WORLD_FROM_DESIGN_POSE, pose_from_frame(assembly_wcf_inclampapproach, MIL2M))
    # wcf_inclamp_pose = multiply(WORLD_FROM_DESIGN_POSE, pose_from_frame(assembly_wcf_inclamp, MIL2M))
    # wcf_final_detach = multiply(WORLD_FROM_DESIGN_POSE, pose_from_frame(assembly_wcf_final, MIL2M))
    # wcf_final_retract = multiply(WORLD_FROM_DESIGN_POSE, pose_from_frame(assembly_wcf_finalretract, MIL2M))

    # # draw target poses
    # set_camera_pose(wcf_inclamp_approach_pose[0] + np.array([0,-0.5,0]), wcf_inclamp_approach_pose[0])

    # # * add previously assembled beam to the scene
    # for prev_beam_id in assembly.get_already_built_beams(beam_id):
    #     prev_beam = assembly.beam(prev_beam_id)
    #     beam_cm = CollisionMesh(prev_beam.cached_mesh, prev_beam.name)
    #     beam_cm.scale(MIL2M)
    #     client.add_collision_mesh(beam_cm, {'color':PREV_BEAM_COLOR})
    #     beam_body = client.collision_objects[beam_cm.id][0]
    #     set_pose(beam_body, WORLD_FROM_DESIGN_POSE)

    # # * add current beam to be transferred
    # # Select the current beam
    # cur_beam = assembly.beam(beam_id)
    # gripper_type = assembly.get_beam_attribute(beam_id, 'gripper_type')
    # gripper = process.get_one_gripper_by_type(gripper_type)
    # gripper.current_frame = process.robot_toolchanger.tool_coordinate_frame.copy()
    # # Beam (In Gripper def pose)
    # gripper_t0cp = process.get_gripper_t0cp_for_beam_at(beam_id, 'assembly_wcf_final')
    # T = Transformation.from_frame_to_frame(gripper_t0cp, gripper.current_frame.copy())
    # cur_beam_cm = CollisionMesh(cur_beam.cached_mesh.transformed(T), cur_beam.name)
    # cur_beam_cm.scale(MIL2M)
    # cur_beam_acm = AttachedCollisionMesh(cur_beam_cm, flange_link_name, ee_touched_link_names)

    # if not disable_attachment:
    #     client.add_attached_collision_mesh(cur_beam_acm, {'robot': robot, 'color':CUR_BEAM_COLOR, 'mass':1})

    # retreat_vector = RETREAT_DISTANCE*np.array([0, 0, -1])
    # cart_key_poses = [multiply(wcf_inclamp_approach_pose, (retreat_vector, unit_quat())),
    #                   wcf_inclamp_pose, wcf_final_detach,
    #                   multiply(wcf_final_retract, (retreat_vector, unit_quat()))]
    # cart_pose_groups = []
    # cart_group_sizes = []
    # for p1, p2 in zip(cart_key_poses[:-1], cart_key_poses[1:]):
    #     c_interp_poses = list(interpolate_poses(p1, p2, pos_step_size=POS_STEP_SIZE))
    #     cart_pose_groups.extend(c_interp_poses)
    #     cart_group_sizes.append(len(c_interp_poses))
    #     # interpolate_poses(pose1, pose2, pos_step_size=0.01, ori_step_size=np.pi/16):
    # print('cart_group_size: ', cart_group_sizes)
    # for ckp in cart_key_poses:
    #     draw_pose(ckp)

    # transfer_plan_options = {
    #     'diagnosis' : False,
    #     # 'resolution' : 0.01,
    #     'resolution' : 0.001,
    #     'custom_limits' : custom_yz_limits,
    # }

    # x_attempts = 100
    # yz_attempts = 50
    # transfer_attempts = 2
    # solution_found = False
    # transfer_traj = None

    # ik_failures = 0
    # path_failures = 0
    # samples_cnt = 0
    # # wait_if_gui('Transfer planning starts')

    # # * sanity check, is beam colliding with the obstacles?
    # with WorldSaver():
    #     env_obstacles = values_as_list(client.collision_objects)
    #     for attachments in client.pychoreo_attachments.values():
    #         for attachment in attachments:
    #             body_collision_fn = get_floating_body_collision_fn(attachment.child, obstacles=env_obstacles)

    #             inclamp_approach_pose = body_from_end_effector(cart_key_poses[0], attachment.grasp_pose)
    #             if body_collision_fn(inclamp_approach_pose, diagnosis=True):
    #                 cprint('wcf_inclamp_approach_pose in collision!', 'red')
    #                 wait_for_user()
    #             # final_retract_pose = body_from_end_effector(cart_key_poses[3], attachment.grasp_pose)
    #             # if body_collision_fn(final_retract_pose, diagnosis=True):
    #             #     cprint('final_retract_pose in collision!', 'red')
    #             #     wait_for_user()
    #     # wait_if_gui('Check the inclamp approach pose.')

    # assert transfer_traj is not None, 'transfer plan failed after {}(X)x{}(YZ) samples (total {}) | {} due to IK, {} due to path'.format(
    #     x_attempts, yz_attempts, samples_cnt, ik_failures, path_failures)
    # # transfer_traj.start_configuration = full_start_conf.copy()
    # # transfer_traj.start_configuration.values[0] = gantry_x_val[0] * 1./MIL2M

    # # if debug:
    # notify('Transfer + Cartesian Planning done.')
    # wait_if_gui('Transfer + Cartesian Planning done. Start simulation...')

    # # * sim transfer motion
    # for traj_pt in transfer_traj.points:
    #     client.set_robot_configuration(robot, traj_pt) #, group=yzarm_move_group
    #     if debug:
    #         wait_for_duration(0.1)
    #         # wait_for_duration(time_step)
    #         # wait_if_gui('sim transfer.')

    # assert cart_conf_vals is not None, 'Cartesian planning failure'
    # confval_groups = divide_list_chunks(cart_conf_vals, cart_group_sizes)

    # # cart_process_start_conf = transfer_traj.start_configuration.copy()
    # # for i in range(9, 9+8):
    # #     cart_process_start_conf.values[i] = transfer_traj.points[-1].values[i-9]
    # # cart_gantry_yz_vals = [cart_process_start_conf.values[i] for i in range(9, 11)]

    # cart_process_start_conf = transfer_traj.points[-1].copy()
    # cart_gantry_yz_vals = [cart_process_start_conf.values[i] for i in range(0, 2)]

    # cart_process = {}
    # for k, conf_vals in enumerate(confval_groups):
    #     cprint('Cartesian phase: {} - {}'.format(k, CART_PROCESS_NAME_FROM_ID[k]), 'cyan')
    #     jt_traj_pts = []
    #     # * drop the beam before final_to_retreat
    #     if k==2:
    #         client.detach_attached_collision_mesh(cur_beam.name)
    #     for i, conf_val in enumerate(conf_vals):
    #         yzarm_conf = Configuration(values=cart_gantry_yz_vals+list(conf_val), types=yzarm_joint_types, joint_names=yzarm_joint_names)
    #         jt_traj_pt = JointTrajectoryPoint(values=yzarm_conf.values, types=yzarm_conf.types, \
    #             time_from_start=Duration(i*1,0))
    #         jt_traj_pt.joint_names = yzarm_joint_names
    #         jt_traj_pts.append(jt_traj_pt)
    #         client.set_robot_configuration(robot, jt_traj_pt) #, group=yzarm_move_group
    #         if debug:
    #             wait_if_gui('step Cartesian.')
    #     # if k == 0:
    #     #     assert_almost_equal(np.array(cart_process_start_conf.values)[9:17], np.array(jt_traj_pts[0].values), decimal=4)
    #     trajectory = JointTrajectory(trajectory_points=jt_traj_pts, \
    #         joint_names=yzarm_joint_names, start_configuration=jt_traj_pts[0], fraction=1.0)
    #     cart_process[CART_PROCESS_NAME_FROM_ID[k]] = trajectory

    # cur_beam_assembled_cm = CollisionMesh(assembly.beam(beam_id).cached_mesh, assembly.beam(beam_id).name)
    # cur_beam_assembled_cm.scale(MIL2M)
    # client.add_collision_mesh(cur_beam_assembled_cm, {'color':PREV_BEAM_COLOR})
    # placed_beam_body = client.collision_objects[cur_beam_assembled_cm.id][0]
    # set_pose(placed_beam_body, WORLD_FROM_DESIGN_POSE)

    # # * Transit planning
    # last_cart_conf = cart_process[CART_PROCESS_NAME_FROM_ID[2]].points[-1]
    # reset_conf = Configuration(R12_INTER_CONF_VALS, yzarm_joint_types, yzarm_joint_names)

    # transit_plan_options = {
    #     'diagnosis' : True,
    #     'resolution' : 0.1, # 0.01
    #     'custom_limits' : custom_yz_limits,
    # }
    # goal_constraints = robot.constraints_from_configuration(reset_conf, [0.01], [0.01], group=yzarm_move_group)

    # transit_attempts = 2
    # transit_traj = None
    # for _ in range(transit_attempts):
    #     with LockRenderer(True):
    #         set_joint_positions(robot_uid, yzarm_joints, last_cart_conf.values)
    #         with WorldSaver():
    #             transit_traj = client.plan_motion(robot, goal_constraints, start_configuration=last_cart_conf,
    #                 group=yzarm_move_group, options=transit_plan_options)
    #         if transit_traj is not None:
    #             break
    # else:
    #     cprint('transit plan failed after {} attempts!'.format(transit_attempts), 'red')

    # # assert transit_traj is not None, 'transfer plan failed!'
    # if transit_traj is not None:
    #     if debug:
    #         wait_if_gui('Transit Planning done. Start simulation...')
    #     # * sim transit motion
    #     for traj_pt in transit_traj.points:
    #         client.set_robot_configuration(robot, traj_pt)
    #         if debug:
    #             wait_if_gui('sim transit.')

    # assembly_plan_data = {k : v.data for k, v in cart_process.items()}
    # assembly_plan_data.update({'transfer' : transfer_traj.data})
    # if transit_traj is not None:
    #     assembly_plan_data.update({'transit' : transit_traj.data})
    # assembly_plan_data['generated_time'] = str(datetime.datetime.now())

    # # * Save Results
    # if write:
    #     output_data_path = os.path.join(JSON_OUT_DIR, 'trajectory_dict_s{}_{}_full.json'.format(seq_i, beam_id))
    #     with open(output_data_path, "w") as outfile:
    #         json.dump(assembly_plan_data, outfile)
    #     cprint('Computed plan trajectories saved to {}'.format(output_data_path), 'green')

    #     # with open(json_path_out, 'w') as f:
    #     #     json_str = jsonpickle.encode(process, keys=True)
    #     #     f.write(json_str)
    #     #     cprint ("Planned path saved to {}, out json_str len: {}".format(json_path_out, len(json_str)), 'green')

