import pytest

def pytest_addoption(parser):
    parser.addoption('--viewer', action='store_true', help='Enables the pybullet viewer')
    parser.addoption('--write', action='store_true', help='Export results')
    parser.addoption('--collision', action='store_false', help='disable collision checking')
    parser.addoption('--watch', action='store_true', help='watch trajectories')
    parser.addoption('--problem', default='rfl_assembly_process.json')
    parser.addoption('--debug_mode', action='store_true', help='debug verbose mode')

@pytest.fixture
def viewer(request):
    return request.config.getoption("--viewer")

@pytest.fixture
def collision(request):
    return request.config.getoption("--collision")

@pytest.fixture
def watch(request):
    return request.config.getoption("--watch")

@pytest.fixture
def problem(request):
    return request.config.getoption("--problem")

@pytest.fixture
def debug_mode(request):
    return request.config.getoption("--debug_mode")
