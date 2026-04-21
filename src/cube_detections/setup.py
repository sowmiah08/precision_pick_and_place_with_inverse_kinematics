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
        ('share/' + package_name +'/launch', ['launch/cube_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zozo',
    maintainer_email='snknitheesh@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [ 
            'cube_detector = cube_detections.cube_detector:main',
            'pointcloud_view = cube_detections.pointcloud_view:main',
            'detector02 = cube_detections.detector02:main',
            'tag_to_tf = cube_detections.tag_to_tf:main',
            'target_tranform = cube_detections.target_transform:main'
        ],
    },
)
