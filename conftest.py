"""Pytest configuration for the project."""

import pytest


def pytest_collection_modifyitems(config, items):
    """Skip tests that require ROS 2 ament packages if they're not installed."""
    try:
        import ament_copyright  # noqa: F401
        import ament_flake8  # noqa: F401
        import ament_pep257  # noqa: F401
    except ImportError:
        # Skip linter tests if ament packages are not available
        skip_ament = pytest.mark.skip(reason="ROS 2 ament packages not installed")
        for item in items:
            if "test_copyright" in item.nodeid or "test_flake8" in item.nodeid or "test_pep257" in item.nodeid:
                item.add_marker(skip_ament)
