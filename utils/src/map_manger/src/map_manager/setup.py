from setuptools import find_packages, setup

package_name = 'map_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/generate_gird_config.yaml']),
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
            'map_manager = map_manager.map_manager:main',
            'generate_gird = map_manager.generate_gird:main',
        ],
    },
)
