import pytest

from sitl_helpers import Sitl


def pytest_configure(config):
    config.addinivalue_line("markers", "sitl_options(**kwargs): options passed to the Sitl fixture")


@pytest.fixture
def sitl(tmp_path, request):
    marker = request.node.get_closest_marker("sitl_options")
    instance = Sitl(str(tmp_path), **(marker.kwargs if marker else {}))
    yield instance
    instance.close()
