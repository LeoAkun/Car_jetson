from setuptools import find_packages, setup
import os

package_name = 'multi_map_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name + '/launch'), ['launch/' + 'multi_map_navigation.launch.py']),
        (os.path.join('share', package_name + '/config'), ['config/' + 'mqtt_config.yaml']),
        (os.path.join('share', package_name + '/config'), ['config/' + 'navigation_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akun',
    maintainer_email='2370344139@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mqtt_task_receiver = multi_map_navigation.mqtt_task_receiver:main',
            'status_reporter = multi_map_navigation.status_reporter:main',
            'navigation_manager = multi_map_navigation.navigation_manager:main',
            'map_switch_controller = multi_map_navigation.map_switch_controller:main',
            'process_manager = multi_map_navigation.process_manager:main',
        ],
    },
)
