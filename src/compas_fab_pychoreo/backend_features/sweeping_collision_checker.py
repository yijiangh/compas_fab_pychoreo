from abc import ABCMeta
from abc import abstractmethod

class SweepingCollisionChecker(object):
    """Interface for a Planner's sweeping collision checker feature.  Any implementation of
    ``SweepingCollisionChecker`` must define the method ``check_sweeping_collisions``.
    The ``__call__`` magic method allows an instance of an implementation of

    ``SweepingCollisionChecker`` to be treated as its ``check_sweeping_collisions`` method.
    See <https://docs.python.org/3/reference/datamodel.html#object.__call__> and
    <https://en.wikipedia.org/wiki/Function_object#In_Python>.
    """
    __metaclass__ = ABCMeta

    def __call__(self, robot, configuration_1=None, configuration_2=None, options=None):
        return self.check_sweeping_collisions(robot, configuration_1, configuration_2, options)

    @abstractmethod
    def check_sweeping_collisions(self, robot, configuration_1=None, configuration_2=None, options=None):
        """....

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            Robot whose configuration may be in collision.
        configuration_1 : :class:`compas_fab.robots.Configuration`
            TODO
        configuration_2 : :class:`compas_fab.robots.Configuration`
            TODO
        options : dict
            TODO

        Returns
        -------
        bool
            True if a collision is detected.
        """
        raise NotImplementedError()

