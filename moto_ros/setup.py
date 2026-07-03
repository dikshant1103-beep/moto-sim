import os
from glob import glob
from setuptools import setup

package_name = "moto_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dikshant",
    maintainer_email="261631710+dikshant1103-beep@users.noreply.github.com",
    description="moto_sim → ROS 2 telemetry bridge",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ws_bridge = moto_ros.ws_bridge:main",
        ],
    },
)
