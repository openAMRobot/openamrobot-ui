from glob import glob
from setuptools import setup

package_name = "openamr_ui_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Raj",
    maintainer_email="raj@example.com",
    description="Standalone OpenAMR UI bringup launch wrapper",
    license="MIT",
)
