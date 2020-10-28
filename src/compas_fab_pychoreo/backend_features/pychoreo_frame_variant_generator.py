from itertools import product
import numpy as np
from pybullet_planning import multiply, Pose, Euler

from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo.utils import is_valid_option
from compas_fab_pychoreo.backend_features.frame_variant_generator import FrameVariantGenerator

# See also: # practical example: https://github.com/gramaziokohler/algorithmic_details/blob/e1d5e24a34738822638a157ca29a98afe05beefd/src/algorithmic_details/accessibility/reachability_map.py#L30
class PyChoreoFiniteEulerAngleVariantGenerator(FrameVariantGenerator):
    def __init__(self, options=None):
        self._options = options

    def generate_frame_variant(self, frame, options=None):
        """....

        Parameters
        ----------
        frame : :class:`compas.geometry.Frame`
            TODO

        Yield
        -----
        frame
            frame variantion
        """
        # overwrite if options is provided in the function call
        options = options or self._options

        # * half range
        delta_roll = is_valid_option(options, 'delta_roll', 0.)
        delta_pitch = is_valid_option(options, 'delta_pitch', 0.)
        delta_yaw = is_valid_option(options, 'delta_yaw', 0.)

        # * discretization
        roll_sample_size = is_valid_option(options, 'roll_sample_size', 1)
        pitch_sample_size = is_valid_option(options, 'pitch_sample_size', 1)
        yaw_sample_size = is_valid_option(options, 'yaw_sample_size', 1)

        roll_gen = np.linspace(-delta_roll, delta_roll, num=roll_sample_size)
        pitch_gen = np.linspace(-delta_pitch, delta_pitch, num=pitch_sample_size)
        yaw_gen = np.linspace(-delta_yaw, delta_yaw, num=yaw_sample_size)
        pose = pose_from_frame(frame)
        # a finite generator
        for roll, pitch, yaw in product(roll_gen, pitch_gen, yaw_gen):
            yield frame_from_pose(multiply(pose, Pose(euler=Euler(roll=roll, pitch=pitch, yaw=yaw))))

