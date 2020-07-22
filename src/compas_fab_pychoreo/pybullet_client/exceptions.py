from compas_fab.backends.exceptions import BackendError


class PyBulletError(BackendError):
    def __init__(self, message):
        super(PyBulletError, self).__init__(message)


class CollisionError(PyBulletError):
    def __init__(self, name1, name2):
        super(CollisionError, self).__init__("Collision between '{}' and '{}'".format(name1, name2))


class InverseKinematicsError(PyBulletError):
    def __init__(self):
        super(InverseKinematicsError, self).__init__("No inverse kinematics solution found.")
