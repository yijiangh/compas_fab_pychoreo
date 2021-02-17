import os
import json
import sys
from termcolor import cprint
from copy import copy

from compas.geometry import Frame
from compas.datastructures import Mesh
from compas.robots import RobotModel
from compas.robots import LocalPackageMeshLoader
from compas_fab.robots import Robot
from compas_fab.robots import RobotSemantics
from compas_fab.robots import Tool
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh
from compas.utilities import DataDecoder, DataEncoder

from compas_fab_pychoreo.conversions import pose_from_frame

HERE = os.path.dirname(__file__)

from pybullet_planning import GREY, BLUE, YELLOW, GREEN, draw_pose

BEAM_COLOR = GREY
GRIPPER_COLOR = BLUE
CLAMP_COLOR = YELLOW
TOOL_CHANGER_COLOR = GREEN

#######################################

# TODO made a GH script for auto-gen URDF for static collision objects
def parse_collision_mesh_from_path(dir_path, filename, scale=1e-3):
    file_path = os.path.join(dir_path, filename)
    obj_name = filename.split('.')[0]
    if filename.endswith('.obj'):
        mesh = Mesh.from_obj(file_path)
    elif filename.endswith('.stl'):
        mesh = Mesh.from_stl(file_path)
    else:
        return None
    cm = CollisionMesh(mesh, obj_name)
    cm.scale(scale)
    return cm

def parse_collision_meshes_from_dir(dir_path, scale=1e-3):
    cms = []
    for filename in sorted(os.listdir(dir_path)):
        cm = parse_collision_mesh_from_path(dir_path, filename, scale)
        if cm is not None:
            cms.append(cm)
    return cms

########################################
# ! DEPRECATED, these objects are parsed from Process json file
ARCHIVED_DATA_DIR = os.path.join(HERE, "data", "_archive")

def itj_TC_PG500_cms():
    tc_dir_path = os.path.abspath(os.path.join(ARCHIVED_DATA_DIR, 'itj_TC'))
    cms = parse_collision_meshes_from_dir(tc_dir_path)
    pg500_dir_path = os.path.abspath(os.path.join(ARCHIVED_DATA_DIR, 'itj_PG500'))
    cms.extend(parse_collision_meshes_from_dir(pg500_dir_path))
    return cms

def itj_TC_PG1000_cms():
    tc_dir_path = os.path.abspath(os.path.join(ARCHIVED_DATA_DIR, 'itj_TC'))
    cms = parse_collision_meshes_from_dir(tc_dir_path)
    pg1000_dir_path = os.path.abspath(os.path.join(ARCHIVED_DATA_DIR, 'itj_PG1000'))
    cms.extend(parse_collision_meshes_from_dir(pg1000_dir_path))
    return cms

def itj_rfl_pipe_cms():
    tc_dir_path = os.path.abspath(os.path.join(ARCHIVED_DATA_DIR, 'itj_rfl_pipe_acm'))
    return parse_collision_meshes_from_dir(tc_dir_path)

def itj_rfl_obstacle_cms():
    dir_path = os.path.abspath(os.path.join(ARCHIVED_DATA_DIR, 'itj_rfl_obstacles'))
    return parse_collision_meshes_from_dir(dir_path)

###########################################

def rfl_setup():
    data_dir = os.path.abspath(os.path.join(HERE, "..", "..", "data", 'robots'))
    # ! the original rfl.urdf use concave collision objects, over-conservative
    # urdf_filename = os.path.join(data_dir, 'rfl_description', 'rfl_description', "urdf", "rfl.urdf")
    urdf_filename = os.path.join(data_dir, 'rfl_description', 'rfl_description', "urdf", "rfl_pybullet.urdf")
    srdf_filename = os.path.join(data_dir, 'rfl_description', 'rfl_description', "urdf", "rfl.srdf")
    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    return urdf_filename, semantics

###########################################

DATA_DIR = os.path.join(HERE, 'data')

def get_process_path(assembly_name, file_dir=DATA_DIR):
    if assembly_name.endswith('.json'):
        filename = os.path.basename(assembly_name)
    else:
        filename = '{}.json'.format(assembly_name)
    model_path = os.path.abspath(os.path.join(file_dir, filename))
    if not os.path.exists(model_path):
        raise FileNotFoundError(model_path)
    return model_path

def parse_process(process):
    # * Load process from file
    with open(get_process_path(process), 'r') as f:
        process = json.load(f, cls=DataDecoder)  # type: RobotClampAssemblyProcess
    return process

def init_objects_from_process(client, process, scale=1e-3):
    # create objects in pybullet, convert mm to m
    for object_id, object_state in process.initial_state.items():
        # create each object in the state dictionary
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
        elif object_id.startswith('robot'):
            # ignore robot setup
            continue

        for i, m in enumerate(meshes):
            cm = CollisionMesh(m, object_id + '_{}'.format(i))
            cm.scale(scale)
            # add mesh to environment at origin
            client.add_collision_mesh(cm, {'color':color})
            # set pose according to state
        current_frame = copy(object_state.current_frame)
        assert current_frame is not None, 'object id : {} , state : {}'.format(object_id, object_state)
        current_frame.point *= scale
        client.set_collision_mesh_frame(object_id, current_frame,
            options={'wildcard' : '{}_*'.format(object_id)})

##########################################
