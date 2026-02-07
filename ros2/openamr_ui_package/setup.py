from setuptools import setup
from glob import glob
import os

package_name = "openamr_ui_package"


def package_files(directory: str):
    """
    Collect all files under 'directory' to install them as package data.
    Returns a list of file paths (relative to the package root).
    """
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths


# React build is copied into:
#   openamr_ui_package/static/app/
react_files = package_files(os.path.join(package_name, "static", "app"))

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

        # install param files (ui_launch.py expects param/config.yaml)
        ("share/" + package_name + "/param", glob("param/*.yaml")),
    ],
    package_data={
        # Install the React build as package data (static/app/**)
        package_name: react_files,
    },
    include_package_data=True,
    install_requires=["setuptools"],
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
    ],
},

)

