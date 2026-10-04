from setuptools import find_packages, setup

package_name = "hmmd_radar"

setup(
    name=package_name,
    version="0.1.0",
    python_requires=">=3.10",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/sensor.launch.py"]),
    ],
    install_requires=["setuptools", "pyserial", "numpy", "matplotlib"],
    zip_safe=True,
    license="Unspecified",
    entry_points={
        "console_scripts": [
            "hmmd_sensor = hmmd_radar.sensor_node:main",
            "hmmd_heatmap = hmmd_radar.viewer_node:main",
        ],
    },
)
