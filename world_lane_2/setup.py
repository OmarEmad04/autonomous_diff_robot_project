from setuptools import setup
import os
from glob import glob

package_name = 'world_lane_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        # Install world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omar',
    maintainer_email='omar@example.com',
    description='Lane keeping world',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={},
)
