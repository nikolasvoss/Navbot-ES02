from setuptools import find_packages, setup

package_name = "navbot_link_bridge"
# The ROS-free link library is shared with the host tools in scripts/ and lives
# outside the package; it is installed next to the bridge.
shared_link_dir = "../../../../navbot_link"

setup(
    name=package_name,
    version="0.1.0",
    python_requires=">=3.10",
    packages=find_packages(exclude=["test"]) + ["navbot_link"],
    package_dir={"navbot_link": shared_link_dir},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial", "protobuf"],
    zip_safe=True,
    entry_points={"console_scripts": ["navbot_link_bridge = navbot_link_bridge.node:main"]},
)
