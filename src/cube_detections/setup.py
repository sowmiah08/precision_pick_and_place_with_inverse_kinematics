import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cube_detections'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[

        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zozo',
    maintainer_email='snknitheesh@gmail.com',
    description='Cube detection and visualization nodes',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'cube_detector = cube_detections.cube_detector:main',
            'pointcloud_view = cube_detections.pointcloud_view:main',
            'tf_node = cube_detections.tf_node:main',
            'cam_view = cube_detections.cam_view:main',
            'move_to_cube = cube_detections.move_to_cube:main',
            'grasp = cube_detections.grasp:main'
        ],
    },
)