from glob import glob
import os
from setuptools import find_packages, setup

package_name = "rk3576_footbath_safety"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sky",
    maintainer_email="sky@example.com",
    description="Safety diagnostics and automatic cmd_vel hard limiter.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "auto_cmd_vel_limiter = rk3576_footbath_safety.velocity_limiter:main",
            "footbath_command_mux = rk3576_footbath_safety.command_mux:main",
            "glass_suspect_monitor = rk3576_footbath_safety.glass_monitor:main",
        ],
    },
)
