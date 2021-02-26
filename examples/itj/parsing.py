import os
import json
import sys
from termcolor import cprint

from compas.datastructures import Mesh
from compas.robots import RobotModel
from compas_fab.robots import RobotSemantics
from compas_fab.robots import CollisionMesh
from compas.utilities import DataDecoder, DataEncoder

from pybullet_planning import get_date

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticMovement

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

DESIGN_DIR = os.path.abspath(os.path.join(HERE, '..', '..', '..', 'itj_design_study', '210128_RemodelFredPavilion'))

def get_process_path(assembly_name, file_dir=DESIGN_DIR):
    if assembly_name.endswith('.json'):
        filename = os.path.basename(assembly_name)
    else:
        filename = '{}.json'.format(assembly_name)
    model_path = os.path.abspath(os.path.join(file_dir, filename))
    if not os.path.exists(model_path):
        raise FileNotFoundError(model_path)
    return model_path

def parse_process(process_name):
    # * Load process from file
    with open(get_process_path(process_name), 'r') as f:
        process = json.load(f, cls=DataDecoder)  # type: RobotClampAssemblyProcess
    return process

def save_process_and_movements(process_name, process, movements, overwrite=False, include_traj_in_process=False, indent=None):
    for m in movements:
        m_file_path = os.path.abspath(os.path.join(DESIGN_DIR, m.filepath))
        with open(m_file_path, 'w') as f:
            json.dump(m, f, cls=DataEncoder, indent=indent, sort_keys=True)
    cprint('#{} movements written to {}'.format(len(movements), os.path.abspath(DESIGN_DIR)), 'green')

    if not include_traj_in_process:
        for m in process.movements:
            if isinstance(m, RoboticMovement):
                m.trajectory = None

    process_file_path = get_process_path(process_name)
    if not overwrite:
        process_dir = os.path.dirname(process_file_path)
        process_fnames = os.path.basename(process_file_path).split('.json')
        process_file_path = os.path.join(process_dir, process_fnames[0] + ('' if overwrite else '_'+get_date()) + '.json')

    with open(process_file_path, 'w') as f:
        json.dump(process, f, cls=DataEncoder, indent=indent, sort_keys=True)
    cprint('Process written to {}'.format(process_file_path), 'green')



##########################################
