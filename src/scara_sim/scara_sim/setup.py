from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'scara_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'description'), glob('description/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='banana-killer',
    maintainer_email='sashagrachev2005@gmail.com',
    description='SCARA robot simulation with analytical IK/ID in Ignition Fortress',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'trajectory_generator = scara_sim.trajectory_generator:main',
            'effort_publisher = scara_sim.effort_publisher:main',
            'trajectory_visualizer = scara_sim.trajectory_visualizer:main',
            'real_trajectory_visualizer = scara_sim.real_trajectory_visualizer:main',  
            'data_logger = scara_sim.data_logger:main',  
        ],
    },
)
