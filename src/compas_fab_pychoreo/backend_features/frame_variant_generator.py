from abc import ABCMeta
from abc import abstractmethod

class FrameVariantGenerator(object):
    """Interface for a Planner's frame variation generation feature.  Any implementation of
    ``FrameVariantGenerator`` must define the method ``generate_frame_variant``.

    The ``__call__`` magic method allows an instance of an implementation of
    ``FrameVariantGenerator`` to be treated as its ``generate_frame_variant`` method.
    See <https://docs.python.org/3/reference/datamodel.html#object.__call__> and
    <https://en.wikipedia.org/wiki/Function_object#In_Python>.
    """
    __metaclass__ = ABCMeta

    def __call__(self, frame, options=None):
        return self.generate_frame_variant(frame)

    @abstractmethod
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
        raise NotImplementedError()
