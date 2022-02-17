import os
import pytest
import numpy as np

from compas.geometry import Frame
from compas.datastructures import Mesh
from compas.robots import RobotModel
from compas_fab.robots import RobotSemantics
from compas_fab.robots import Tool
from compas_fab.robots import CollisionMesh
from compas_fab_pychoreo.conversions import transformation_from_pose, pose_from_transformation

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
    cm_from_name = {}
    for filename in sorted(os.listdir(dir_path)):
        cm = parse_collision_mesh_from_path(dir_path, filename, scale)
        if cm is not None:
            cm_from_name[cm.id] = cm
    return cm_from_name

########################################

@pytest.fixture
def fixed_waam_setup():
    HERE = os.path.dirname(__file__)
    package_path = os.path.abspath(os.path.join(HERE, "..", "..", "data", "robots", "abb_fixed_waam"))
    urdf_filename = os.path.join(package_path, "urdf", "abb_fixed_waam.urdf")
    srdf_filename = os.path.join(package_path, "srdf", "abb_fixed_waam.srdf")

    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)

    tool_frame_robotA = Frame.from_euler_angles([0.591366, -0.000922, 1.570177], static=True, axes='xyz', point=[-0.002241, -0.000202, 0.505922])
    tool_mesh_robotA = Mesh.from_obj(os.path.join(package_path, "meshes", "collision", "waam_tool.obj"))
    robotA_tool = Tool(tool_mesh_robotA, tool_frame_robotA, collision=tool_mesh_robotA)
    return urdf_filename, semantics, robotA_tool

@pytest.fixture
def abb_irb4600_40_255_setup():
    HERE = os.path.dirname(__file__)
    data_dir = os.path.abspath(os.path.join(HERE, "..", "..", "data", 'robots'))
    urdf_filename = os.path.join(data_dir, 'abb_irb4600_40_255', "urdf", "abb_irb4600_40_255.urdf")
    srdf_filename = os.path.join(data_dir, 'abb_fixed_waam', "srdf", "abb_irb4600_40_255.srdf")
    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    return urdf_filename, semantics

@pytest.fixture
def itj_TC_g1_cms():
    HERE = os.path.dirname(__file__)
    tc_dir_path = os.path.abspath(os.path.join(HERE, "..", "data", 'TC_static'))
    cm_from_name = parse_collision_meshes_from_dir(tc_dir_path)
    g1_dir_path = os.path.abspath(os.path.join(HERE, "..", "data", 'g1_static'))
    cm_from_name.update(parse_collision_meshes_from_dir(g1_dir_path))
    return cm_from_name

@pytest.fixture
def itj_tool_changer_urdf_path():
    HERE = os.path.dirname(__file__)
    urdf_path = os.path.abspath(os.path.join(HERE, "..", "data", 'TC3_Igus', 'urdf', 'TC3_Igus.urdf'))
    return urdf_path

@pytest.fixture
def itj_g1_urdf_path():
    HERE = os.path.dirname(__file__)
    urdf_path = os.path.abspath(os.path.join(HERE, "..", "data", 'g1', 'urdf', 'g1.urdf'))
    return urdf_path

@pytest.fixture
def itj_beam_cm():
    HERE = os.path.dirname(__file__)
    data_dir = os.path.abspath(os.path.join(HERE, "..", "data"))
    return parse_collision_mesh_from_path(data_dir, "itj_beam_b2.obj")

@pytest.fixture
def itj_s1_urdf_path():
    HERE = os.path.dirname(__file__)
    urdf_path = os.path.abspath(os.path.join(HERE, "..", "data", 's1', 'urdf', 's1.urdf'))
    return urdf_path

##########################################

@pytest.fixture
def itj_tool_changer_grasp_transf():
    # gripper_from_object
    grasp_pose = ((0, 0, 0), (0, 0, 0, 1))
    return transformation_from_pose(grasp_pose)

@pytest.fixture
def itj_gripper_grasp_transf():
    # gripper_from_object
    grasp_pose = ((0, 0, 0.0663), (0, 0, 0, 1))
    return transformation_from_pose(grasp_pose)

@pytest.fixture
def itj_beam_grasp_transf():
    # gripper_from_object
    grasp_pose = ((0, 0, 0.0), (0, 0, 0, 1))
    return transformation_from_pose(grasp_pose)

@pytest.fixture
def itj_s1_grasp_transf():
    # gripper_from_object
    grasp_pose = ((0, 0, 0), (0, 0, 0, 1))
    return transformation_from_pose(grasp_pose)

##########################################

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
def tube_cms():
    HERE = os.path.dirname(__file__)
    data_dir = os.path.abspath(os.path.join(HERE, "..", "data", "long_tube"))
    return [parse_collision_mesh_from_path(data_dir, "long_tube_{}.obj".format(i), scale=1) for i in range(4)]

@pytest.fixture
def thin_panel_cm():
    HERE = os.path.dirname(__file__)
    data_dir = os.path.abspath(os.path.join(HERE, "..", "data"))
    return parse_collision_mesh_from_path(data_dir, "thin_panel.obj", scale=1)

##########################################

@pytest.fixture
def abb_tolerances():
    joint_jump_tolerances = {}
    joint_compare_tolerances = {}
    joint_resolutions = {}
    joint_names = [f'joint_{i}' for i in range(1,7)]
    for jt_name in joint_names:
        joint_jump_tolerances[jt_name] = 10.0 * np.pi / 180.0 # 0.174 rad
        joint_compare_tolerances[jt_name] = 0.0025 # rad, try tightened to 0.001 if possible
        joint_resolutions[jt_name] = 10.0 * np.pi / 180.0 # 0.174 rad
    tolerances = {
        'joint_jump_tolerances' : joint_jump_tolerances,
        'joint_compare_tolerances' : joint_compare_tolerances,
        'joint_resolutions' : joint_resolutions,
        # 'joint_custom_limits' : get_gantry_robot_custom_limits(MAIN_ROBOT_ID),
    }
    return tolerances
