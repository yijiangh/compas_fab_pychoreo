from abc import ABCMeta
from abc import abstractmethod

class TrajectorySmoother(object):
    """Interface for smoothing a given robot trajecotry.  Any implementation of
    ``TrajectorySmoother`` must define the method ``smooth_trajectory``.
    The ``__call__`` magic method allows an instance of an implementation of

    ``TrajectorySmoother`` to be treated as its ``smooth_trajectory`` method.
    See <https://docs.python.org/3/reference/datamodel.html#object.__call__> and
    <https://en.wikipedia.org/wiki/Function_object#In_Python>.
    """
    __metaclass__ = ABCMeta

    def __call__(self, robot, trajectory, options=None):
        return self.step_in_collision(robot, trajectory, options)

    @abstractmethod
    def smooth_trajectory(self, robot, trajectory, options=None):
        pass
