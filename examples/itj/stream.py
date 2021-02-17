from termcolor import cprint
from copy import copy

from pybullet_planning import GREY

from compas.geometry import Frame
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh
from compas_fab_pychoreo.conversions import pose_from_frame

from .visualization import BEAM_COLOR, GRIPPER_COLOR, CLAMP_COLOR, TOOL_CHANGER_COLOR

##############################

def set_state(client, robot, process, state_from_object, initialize=False, scale=1e-3):
    for object_id, object_state in state_from_object.items():
        if object_id.startswith('robot'):
            if object_state.kinematic_config is not None:
                client.set_robot_configuration(robot, object_state.kinematic_config)
        else:
            if initialize:
                # * create each object in the state dictionary
                # create objects in pybullet, convert mm to m
                color = GREY
                if object_id.startswith('b'):
                    beam = process.assembly.beam(object_id)
                    # ! notice that the notch geometry will be convexified in pybullet
                    meshes = [beam.mesh]
                    color = BEAM_COLOR
                elif object_id.startswith('c') or object_id.startswith('g'):
                    tool = process.tool(object_id)
                    tool.current_frame = Frame.worldXY()
                    if object_state.kinematic_config is not None:
                        # this might be in millimeter, but that's not related to pybullet's business (if we use static meshes)
                        tool._set_kinematic_state(object_state.kinematic_config)
                    meshes = tool.draw_visual()
                    if object_id.startswith('c'):
                        color = CLAMP_COLOR
                    elif object_id.startswith('g'):
                        color = GRIPPER_COLOR
                elif object_id.startswith('t'):
                    tool = process.robot_toolchanger
                    tool.current_frame = Frame.worldXY()
                    meshes = tool.draw_visual()
                    color = TOOL_CHANGER_COLOR
                for i, m in enumerate(meshes):
                    cm = CollisionMesh(m, object_id + '_{}'.format(i))
                    cm.scale(scale)
                    # add mesh to environment at origin
                    client.add_collision_mesh(cm, {'color':color})
            # * set pose according to state
            if object_state.current_frame is not None:
                current_frame = copy(object_state.current_frame)
                # assert current_frame is not None, 'object id : {} , state : {}'.format(object_id, object_state)
                current_frame.point *= scale
                client.set_collision_mesh_frame(object_id, current_frame,
                    options={'wildcard' : '{}_*'.format(object_id)})

##############################
