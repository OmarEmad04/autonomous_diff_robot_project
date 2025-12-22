from setuptools import find_packages, setup

package_name = 'lane_keeper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omar',
    maintainer_email='omar@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lane_follow_node = lane_keeper.lane_follow_node:main',
            'lane_vision_node = lane_keeper.lane_vision_node:main',
            'path_node = lane_keeper.path_node:main',
            'controller_node = lane_keeper.controller_node:main',
        ],
    },
)
