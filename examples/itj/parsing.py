import os

from compas.geometry import Frame
from compas.datastructures import Mesh
from compas.robots import RobotModel
from compas.robots import LocalPackageMeshLoader
from compas_fab.robots import Robot
from compas_fab.robots import RobotSemantics
from compas_fab.robots import Tool
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh

HERE = os.path.dirname(__file__)

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

def rfl_setup():
    data_dir = os.path.abspath(os.path.join(HERE, "..", "..", "data", 'robots'))
    urdf_filename = os.path.join(data_dir, 'rfl_description', 'rfl_description', "urdf", "rfl.urdf")
    srdf_filename = os.path.join(data_dir, 'rfl_description', 'rfl_description', "urdf", "rfl.srdf")
    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    robot = Robot(model, semantics=semantics)
    return urdf_filename, robot

def itj_TC_PG500_cms():
    tc_dir_path = os.path.abspath(os.path.join(HERE, "data", 'itj_TC'))
    cms = parse_collision_meshes_from_dir(tc_dir_path)
    pg500_dir_path = os.path.abspath(os.path.join(HERE, "data", 'itj_PG500'))
    cms.extend(parse_collision_meshes_from_dir(pg500_dir_path))
    return cms

def itj_TC_PG1000_cms():
    tc_dir_path = os.path.abspath(os.path.join(HERE, "data", 'itj_TC'))
    cms = parse_collision_meshes_from_dir(tc_dir_path)
    pg1000_dir_path = os.path.abspath(os.path.join(HERE, "data", 'itj_PG1000'))
    cms.extend(parse_collision_meshes_from_dir(pg1000_dir_path))
    return cms

def itj_rfl_obstacle_cms():
    dir_path = os.path.abspath(os.path.join(HERE, "data", 'itj_rfl_obstacles'))
    return parse_collision_meshes_from_dir(dir_path)
