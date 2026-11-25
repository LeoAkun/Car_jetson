import os
from glob import glob

from setuptools import setup

package_name = 'ground'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'rviz'), glob('config/rviz/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    maintainer='akun',
    maintainer_email='2370344139@qq.com',
    description='Pothole detector using depth image and height map',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'height_map_pothole_detector = ground.height_map_pothole_detector:main',
        ],
    },
)
