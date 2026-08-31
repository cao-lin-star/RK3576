from glob import glob
import os
from setuptools import find_packages, setup
package_name = "rk3576_footbath_mobile"
setup(name=package_name, version="0.1.0", packages=find_packages(exclude=["test"]),
 data_files=[
  ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
  ("share/" + package_name, ["package.xml", "README.md"]),
  (os.path.join("share", package_name, "web"), glob("web/*")),
  (os.path.join("share", package_name, "config"), glob("config/*")),
  (os.path.join("share", package_name, "scripts"), glob("scripts/*")),
  (os.path.join("share", package_name, "deploy", "systemd"), glob("deploy/systemd/*"))],
 install_requires=["setuptools"], zip_safe=True, maintainer="sky",
 maintainer_email="sky@example.com",
 description="Fail-closed local Wi-Fi mobile mapping and navigation gateway.",
 license="Apache-2.0", tests_require=["pytest"],
 entry_points={"console_scripts": ["footbath_mobile_gateway = rk3576_footbath_mobile.gateway:main"]})
