from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'arm_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        (os.path.join('share', package_name, 'assets'),
            glob('assets/*')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zozo',
    maintainer_email='sowmiah.jerom@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
