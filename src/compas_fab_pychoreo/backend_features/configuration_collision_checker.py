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

    def __call__(self, configuration, group=None, options=None):
        return self.configuration_in_collision(configuration, group, options)

    @abstractmethod
    def configuration_in_collision(self, configuration, group=None, options=None):
        """....

        Parameters
        ----------
        configuration : :class:`compas_fab.robots.Configuration`
            TODO
        group : str, optional
            The name of the group to be used in the calculation.
        options : dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

        Returns
        -------
        bool
            True if a collision is detected.
        """
        pass

