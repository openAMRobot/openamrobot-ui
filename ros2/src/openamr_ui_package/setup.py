from setuptools import setup
from glob import glob
import os

package_name = "openamr_ui_package"


def data_files_from_dir(source_dir: str, dest_prefix: str):
    """Walk source_dir and produce data_files entries preserving subdirectory structure."""
    result = []
    for dirpath, _, filenames in os.walk(source_dir):
        if not filenames:
            continue
        rel = os.path.relpath(dirpath, start=os.path.dirname(source_dir))
        dest = os.path.join(dest_prefix, rel)
        files = [os.path.join(dirpath, f) for f in filenames]
        result.append((dest, files))
    return result


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        # ament index + package manifest
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        ("share/" + package_name, ["package.xml"]),

        # launch files
        ("share/" + package_name + "/launch", glob("launch/*.py")),

        # Runtime data used by the UI nodes. Preserve nested folders so paths
        # returned by get_package_share_directory(...) work after install.
        *data_files_from_dir("param", os.path.join("share", package_name)),
        *data_files_from_dir("maps", os.path.join("share", package_name)),
        *data_files_from_dir("paths", os.path.join("share", package_name)),

        # React build → share/openamr_ui_package/static/app/** (colcon installs/symlinks these)
        *data_files_from_dir(
            os.path.join(package_name, "static", "app"),
            os.path.join("share", package_name),
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    test_suite="test",
    zip_safe=True,
    maintainer="darkadius",
    maintainer_email="tobbalya@gmail.com",
    description="UI for AMRs",
    license="MIT",
    entry_points={
        "console_scripts": [
            "flask = openamr_ui_package.flask_app:main",
            "handler = openamr_ui_package.folders_handler:main",
            "nav = openamr_ui_package.waypoint_nav:main",
            "map_relay = openamr_ui_package.map_relay:main",
            "nav_relay = openamr_ui_package.nav_relays:main",
        ],
    },
)
