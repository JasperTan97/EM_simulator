import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'em_sim_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sdf_to_occupancy_grid = em_sim_gazebo.sdf_to_occupancy_grid:main',
            'map_subscriber = em_sim_gazebo.map_subscriber:main',
        ],
    },
)
