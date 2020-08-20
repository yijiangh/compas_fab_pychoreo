        # # * start planning
        # if parse_transfer_motion:
        #     # if a transfer plan is given, set the x, y, z axis based on the parsed transfer motion
        #     full_start_conf = transfer_traj.start_configuration.copy()
        #     full_start_conf.scale(MIL2M)
        #     set_joint_positions(robot_uid, full_joints, full_start_conf.values)

        #     # * sim transfer motion
        #     traj_joints = joints_from_names(robot_uid, transfer_traj.joint_names)
        #     for traj_pt in transfer_traj.points:
        #         traj_pt_scaled = traj_pt.scaled(MIL2M)
        #         set_joint_positions(robot_uid, traj_joints, traj_pt_scaled.values)
        #         for _, attach in client.attachments.items():
        #             attach.assign()
        #         # if debug:
        #             # print(traj_pt_scaled)
        #             # wait_for_duration(0.1)
        #             # wait_if_gui()
        #             # yzarm_conf = Configuration(traj_pt_scaled.values, yzarm_joint_types, yzarm_joint_names)
        #             # if client.configuration_in_collision(yzarm_conf, group=yzarm_move_group, options={'diagnosis':True}):
        #             #     wait_if_gui('collision detected!!')

        #     print('last transfer conf: ', transfer_traj.points[-1])


        #####################################

        # * Sequentially copy movement target_frame to next movement source_frame
        # _source_frame = None
        # for ia, action in enumerate(process.actions):
        #     for im, movement in enumerate(action.movements):
        #         if isinstance(movement, RoboticMovement):
        #             movement.source_frame = _source_frame
        #             _source_frame = movement.target_frame

        # * Sequential path planning
        # last_configuration = pp.rfl_timber_start_configuration()

        # t = time.time()
        # for ia, action in enumerate(process.actions):
        #     # if ia not in list(range(0, 20)): continue
        #     seq_n = action.seq_n
        #     act_n = action.act_n

        #     for im, movement in enumerate(action.movements):
        #         cprint ("Seq(%s) Act (%s) Mov (%s) - %s" % (seq_n, act_n, im, movement), 'cyan')

        #         if isinstance(movement, RoboticMovement):
        #             # Add already built beams to planning scene
        #             # beam_id = assembly.sequence[seq_n]
        #             # already_built_beam_ids = assembly.get_already_built_beams(beam_id)
        #             # pp.remove_collision_mesh('already_built_beams')
        #             # for already_beam_id in already_built_beam_ids:
        #             #     pp.append_collision_mesh(assembly.beam(already_beam_id).cached_mesh, 'already_built_beams')

        #             # Attach Tool and Beam to robot

        #             # Prepare Starting Configuration
        #             if last_configuration is not None:
        #                 # retrive last planned trajectory's last config
        #                 start_configuration = last_configuration
        #             else:
        #                 # Previous planning failed, compute config based on source_frame
        #                 # Source frame is created in the beginning of this file.
        #                 start_configuration = pp.ik_solution(movement.source_frame, verbose=True)

        #             # Perform planning if start_configuration is not None.
        #             if start_configuration is not None:
        #                 trajectory = pp.plan_motion(
        #                     movement.target_frame,
        #                     start_configuration = start_configuration,
        #                     free_motion = True,
        #                     verbose = True)
        #             else:
        #                 trajectory = None

        #             # Post Planning
        #             if trajectory and trajectory.fraction == 1.0:
        #                 # Path planning succeeded
        #                 print("> > Motion Planned (%s pts, %.1f secs)" % (len(trajectory.points), trajectory.time_from_start))
        #                 print("> > Last Point: %s" % trajectory.points[-1])

        #                 # Assign Last frame of the path to next configuration.
        #                 last_configuration = Configuration(values=trajectory.points[-1].values, types=pp.joint_types, joint_names=pp.joint_names)

        #                 # Save trajectory to movement
        #                 movement.trajectory = trajectory
        #             else:
        #                 # Path planning failed.
        #                 print("> > Motion Plan Fail !!!")
        #                 last_configuration = None
        #         else:
        #             print("> > No Robotic Motion")
        #         print("")
        # print("> Total Path Plan Time: %.2fs" % (time.time() - t))


