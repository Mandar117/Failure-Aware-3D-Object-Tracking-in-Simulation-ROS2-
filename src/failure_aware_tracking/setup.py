from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'failure_aware_tracking'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mandar',
    maintainer_email='your_email@example.com',
    description='Failure-aware multi-object 3D tracking',
    license='MIT',
    entry_points={
        'console_scripts': [
            'object_mover = failure_aware_tracking.object_mover:main',
            'color_detector = failure_aware_tracking.color_detector:main',
        ],
    },
)