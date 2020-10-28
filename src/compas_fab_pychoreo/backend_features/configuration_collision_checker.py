from abc import ABCMeta
from abc import abstractmethod

class ConfigurationCollisionChecker(object):
    """Interface for a Planner's robot collision checker feature.  Any implementation of
    ``ConfigurationCollisionChecker`` must define the method ``configuration_in_collision``.
    The ``__call__`` magic method allows an instance of an implementation of
    ``ConfigurationCollisionChecker`` to be treated as its ``configuration_in_collision`` method.
    See <https://docs.python.org/3/reference/datamodel.html#object.__call__> and
    <https://en.wikipedia.org/wiki/Function_object#In_Python>.
    """
    __metaclass__ = ABCMeta

    def __call__(self, robot, configuration=None, options=None):
        return self.check_collisions(robot, configuration, options)

    @abstractmethod
    def check_collisions(self, robot, configuration=None, options=None):
        """....

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            Robot whose configuration may be in collision.
        configuration : :class:`compas_fab.robots.Configuration`
            Configuration to be checked for collisions.  If ``None`` is given, the current
            configuration will be checked.  Defaults to ``None``.

        Returns
        -------
        bool
            True if a collision is detected.
        """
        pass

