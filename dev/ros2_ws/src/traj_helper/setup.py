from setuptools import find_packages, setup

package_name = "traj_helper"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/cfg/fastlio", ["cfg/fastlio/scan_go2w.yaml"]),
        ("share/" + package_name + "/launch", [
            "launch/system.launch.py",
        ]),
        
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jlcucumber",
    maintainer_email="jasonli1207@foxmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "trajectory_follower = traj_helper.trajectory_follower:main"
        ],
    },
)
