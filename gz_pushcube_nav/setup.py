from setuptools import setup

package_name = "gz_pushcube_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/pushcube.launch.py",
            "launch/collect.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Botong Qu",
    maintainer_email="botong_qu@outlook.com",
    description="Teleop, success check, and episode recording for the Gazebo push-cube task",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "keyboard_teleop = gz_pushcube_nav.keyboard_teleop:main",
            "success_checker = gz_pushcube_nav.success_checker:main",
            "episode_recorder = gz_pushcube_nav.episode_recorder:main",
        ],
    },
)
