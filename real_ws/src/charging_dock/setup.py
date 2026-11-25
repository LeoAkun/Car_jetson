from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'charging_dock'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    scripts=['tools/stage3_camera_check.py'],
    zip_safe=True,
    maintainer='akun',
    maintainer_email='2370344139@qq.com',
    description='Rear lidar, camera, and charging dock support tools.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'relative_docking_controller = charging_dock.relative_docking_node:main',
            'dock_tag_adapter = charging_dock.dock_tag_adapter:main',
            'stage3_camera_check = charging_dock.stage3_camera_check:main',
            'command_mux = charging_dock.command_mux:main',
            'active_docking_controller = '
            'charging_dock.active_docking_controller:main',
        ],
    },
)
