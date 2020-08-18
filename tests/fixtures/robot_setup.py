import os
import pytest

from compas.geometry import Frame
from compas.datastructures import Mesh
from compas.robots import RobotModel
from compas.robots import LocalPackageMeshLoader
from compas_fab.robots import Robot
from compas_fab.robots import RobotSemantics
from compas_fab.robots import Tool
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh

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

@pytest.fixture
def fixed_waam_setup():
    HERE = os.path.dirname(__file__)
    package_path = os.path.abspath(os.path.join(HERE, "..", "..", "data", "robots", "abb_fixed_waam"))
    urdf_filename = os.path.join(package_path, "urdf", "abb_fixed_waam.urdf")
    srdf_filename = os.path.join(package_path, "srdf", "abb_fixed_waam.srdf")

    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)

    # load_geometry = False
    # packages = ["abb_irb4600_40_255", "abb_linear_axis", "waam_setup"]
    # loaders = [LocalPackageMeshLoader(package_path, package) for package in packages]
    # if load_geometry:
    #     model.load_geometry(*loaders)

    robot = Robot(model, semantics=semantics)
    tool_frame_robotA = Frame.from_euler_angles([0.591366, -0.000922, 1.570177], static=True, axes='xyz', point=[-0.002241, -0.000202, 0.505922])
    tool_mesh_robotA = Mesh.from_obj(os.path.join(package_path, "meshes", "collision", "waam_tool.obj"))
    robotA_tool = Tool(tool_mesh_robotA, tool_frame_robotA, collision=tool_mesh_robotA)
    return urdf_filename, robot, robotA_tool

@pytest.fixture
def abb_irb4600_40_255_setup():
    HERE = os.path.dirname(__file__)
    data_dir = os.path.abspath(os.path.join(HERE, "..", "..", "data", 'robots'))
    urdf_filename = os.path.join(data_dir, 'abb_irb4600_40_255', "urdf", "abb_irb4600_40_255.urdf")
    srdf_filename = os.path.join(data_dir, 'abb_fixed_waam', "srdf", "abb_irb4600_40_255.srdf")
    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    robot = Robot(model, semantics=semantics)
    return urdf_filename, robot

@pytest.fixture
def rfl_setup():
    HERE = os.path.dirname(__file__)
    data_dir = os.path.abspath(os.path.join(HERE, "..", "..", "data", 'robots'))
    urdf_filename = os.path.join(data_dir, 'rfl_description', 'rfl_description', "urdf", "rfl.urdf")
    srdf_filename = os.path.join(data_dir, 'rfl_description', 'rfl_description', "urdf", "rfl.srdf")
    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    robot = Robot(model, semantics=semantics)
    return urdf_filename, robot

@pytest.fixture
def itj_TC_PG500_cms():
    HERE = os.path.dirname(__file__)
    tc_dir_path = os.path.abspath(os.path.join(HERE, "..", "data", 'itj_TC'))
    cms = parse_collision_meshes_from_dir(tc_dir_path)
    pg500_dir_path = os.path.abspath(os.path.join(HERE, "..", "data", 'itj_PG500'))
    cms.extend(parse_collision_meshes_from_dir(pg500_dir_path))
    return cms

@pytest.fixture
def itj_TC_PG1000_cms():
    HERE = os.path.dirname(__file__)
    tc_dir_path = os.path.abspath(os.path.join(HERE, "..", "data", 'itj_TC'))
    cms = parse_collision_meshes_from_dir(tc_dir_path)
    pg1000_dir_path = os.path.abspath(os.path.join(HERE, "..", "data", 'itj_PG1000'))
    cms.extend(parse_collision_meshes_from_dir(pg1000_dir_path))
    return cms

@pytest.fixture
def itj_beam_cm():
    HERE = os.path.dirname(__file__)
    data_dir = os.path.abspath(os.path.join(HERE, "..", "data"))
    return parse_collision_mesh_from_path(data_dir, "itj_beam1.obj")

@pytest.fixture
def base_plate_cm():
    HERE = os.path.dirname(__file__)
    data_dir = os.path.abspath(os.path.join(HERE, "..", "data"))
    return parse_collision_mesh_from_path(data_dir, "base_plate.obj", scale=1)

@pytest.fixture
def column_obstacle_cm():
    HERE = os.path.dirname(__file__)
    data_dir = os.path.abspath(os.path.join(HERE, "..", "data"))
    return parse_collision_mesh_from_path(data_dir, "column_obstacle.obj", scale=1)

@pytest.fixture
def itj_rfl_obstacle_cms():
    HERE = os.path.dirname(__file__)
    dir_path = os.path.abspath(os.path.join(HERE, "..", "data", 'itj_rfl_obstacles'))
    return parse_collision_meshes_from_dir(dir_path)

@pytest.fixture
def itj_process_file_path():
    HERE = os.path.dirname(__file__)
    file_path = os.path.abspath(os.path.join(HERE, "..", "data", 'rfl_assembly_process.json'))
    return file_path
