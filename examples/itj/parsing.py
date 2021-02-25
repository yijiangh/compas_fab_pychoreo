import os
import json
import sys

from compas.geometry import Frame
from compas.datastructures import Mesh
from compas.robots import RobotModel
from compas.robots import LocalPackageMeshLoader
from compas_fab.robots import Robot
from compas_fab.robots import RobotSemantics
from compas_fab.robots import Tool
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh
from compas.utilities import DataDecoder, DataEncoder

HERE = os.path.dirname(__file__)

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

# DATA_DIR = os.path.join(HERE, 'data')
DATA_DIR = r'C:\Users\harry\Dropbox (MIT)\code_ws_dropbox\itj_ws\itj_design_study\210128_RemodelFredPavilion'

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

##########################################
