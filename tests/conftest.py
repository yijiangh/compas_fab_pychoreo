import pytest
from fixtures.robot_setup import fixed_waam_setup

def pytest_addoption(parser):
    parser.addoption('--viewer', action='store_true', help='Enables the pybullet viewer')
    parser.addoption('--write', action='store_true', help='Export results')
    parser.addoption('--collision', action='store_false', help='disable collision checking')
    # parser.addoption('--watch', action='store_true', help='watch trajectories')

@pytest.fixture
def viewer(request):
    return request.config.getoption("--viewer")

@pytest.fixture
def write(request):
    return request.config.getoption("--write")

@pytest.fixture
def collision(request):
    return request.config.getoption("--collision")
