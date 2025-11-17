import os
from glob import glob
from setuptools import setup

package_name = 'rosbot_joystick'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dotX Automation s.r.l',
    maintainer_email='info@dotxautomation.com',
    description='Joystick teleoperation node for Roboworks ROSbot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_node = rosbot_joystick.joystick_node:main'
        ],
    },
)
