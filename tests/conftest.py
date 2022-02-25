from email.policy import default
import pytest
from fixtures.robot_setup import *

def pytest_addoption(parser):
    parser.addoption('--viewer', action='store_true', help='Enables the pybullet viewer')
    parser.addoption('--write', action='store_true', help='Export results')
    parser.addoption('--diagnosis', action='store_true', help='enable diagnosis for collision checking')
    parser.addoption('--attempt_iters', default=100, type=int, help='attempt iterations')

@pytest.fixture
def viewer(request):
    return request.config.getoption("--viewer")

@pytest.fixture
def write(request):
    return request.config.getoption("--write")

@pytest.fixture
def diagnosis(request):
    return request.config.getoption("--diagnosis")

@pytest.fixture
def attempt_iters(request):
    return request.config.getoption("--attempt_iters")
