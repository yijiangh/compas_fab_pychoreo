import os

from compas.geometry import Frame
from compas.datastructures import Mesh
from compas.robots import RobotModel
from compas.robots import LocalPackageMeshLoader
from compas_fab.robots import Robot
from compas_fab.robots import RobotSemantics
from compas_fab.robots import Tool
import pytest


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
def itj_gripper_path():
    HERE = os.path.dirname(__file__)
    return os.path.abspath(os.path.join(HERE, "..", "data", "itj_gripper1.obj"))

@pytest.fixture
def itj_beam_path():
    HERE = os.path.dirname(__file__)
    return os.path.abspath(os.path.join(HERE, "..", "data", "itj_beam1.obj"))
