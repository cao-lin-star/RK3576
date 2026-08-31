from glob import glob
import os
from setuptools import find_packages, setup

package_name = "rk3576_footbath_exploration"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sky",
    maintainer_email="sky@example.com",
    description="Frontier exploration supervisor and map-save interface for the footbath robot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "exploration_supervisor = rk3576_footbath_exploration.supervisor:main",
        ],
    },
)
