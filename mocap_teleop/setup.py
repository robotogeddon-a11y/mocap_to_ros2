from glob import glob

from setuptools import setup

package_name = "mocap_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/urdf/g1", glob("urdf/g1/*")),
        ("share/" + package_name + "/meshes/g1", glob("meshes/g1/*")),
        ("share/" + package_name + "/config", glob("config/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robotogeddon",
    maintainer_email="robotogeddon@gmail.com",
    description="Drive a ROS2 robot model from video motion capture",
    license="MIT",
    entry_points={
        "console_scripts": [
            "robot_player = mocap_teleop.robot_player:main",
        ],
    },
)
