import os
from glob import glob
from setuptools import setup

package_name = "rosbot_rviz"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dotX Automation s.r.l",
    maintainer_email="info@dotxautomation.com",
    description="RViz 2 visualization package for the Roboworks ROSbot platform.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # No console scripts for this package
        ],
    },
)
