import numpy as np
from pybullet_planning import GREY, BLUE, YELLOW, GREEN, draw_pose

BEAM_COLOR = GREY
GRIPPER_COLOR = BLUE
CLAMP_COLOR = YELLOW
TOOL_CHANGER_COLOR = GREEN

################################################

def rfl_camera(scale=1e-3):
    camera = {
        'location': np.array([14830.746366, 17616.580504, 9461.594828])*scale,
        'target' : np.array([24470.185559, 7976.896428, 2694.413294])*scale,
        'lens' : 50.0*scale,
        'up_direction':np.array([0.314401,-0.314409,0.895712])*scale,
    }
    return camera
