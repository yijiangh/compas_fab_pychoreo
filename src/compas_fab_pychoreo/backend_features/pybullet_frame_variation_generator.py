from abc import ABCMeta
from abc import abstractmethod

class FrameVariantionGenerator(object):
    """Interface for generating frame variantion.  Any implementation of
    ``FrameVariantionGenerator`` must define the method ``generate_frame_variantions``.
    The ``__call__`` magic method allows an instance of an implementation of
    ``FrameVariantionGenerator`` to be treated as its ``generate_frame_variantions`` method.
    See <https://docs.python.org/3/reference/datamodel.html#object.__call__> and
    <https://en.wikipedia.org/wiki/Function_object#In_Python>.
    """
    __metaclass__ = ABCMeta

    def __call__(self, frame, options=None):
        return self.generate_frame_variantions(frame, options)

    @abstractmethod
    def generate_frame_variantions(self, frame, options=None):
        """....

        Parameters
        ----------
        frame : :class:`compas.geometry.Frame`
            TODO
        options : dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

        Returns
        -------
        bool
            True if a collision is detected.
        """
        pass

# practical example: https://github.com/gramaziokohler/algorithmic_details/blob/e1d5e24a34738822638a157ca29a98afe05beefd/src/algorithmic_details/accessibility/reachability_map.py#L30
