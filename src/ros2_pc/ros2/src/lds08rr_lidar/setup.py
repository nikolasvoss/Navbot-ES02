from setuptools import find_packages, setup

package_name = "lds08rr_lidar"

setup(
    name=package_name,
    version="0.1.0",
    python_requires=">=3.10",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial", "lds2d[viz]==0.7.0"],
    package_data={package_name: ["templates/*.html"]},
    zip_safe=True,
    entry_points={"console_scripts": ["lds08rr_lidar = lds08rr_lidar.service:main"]},
)
