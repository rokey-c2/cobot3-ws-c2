import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cobot3'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='injae',
    maintainer_email='il1282113@gmail.com',
    description='Subscribes to a camera image topic and publishes a detected cube color (blue=1, green=2)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'm0609_color_detector = cobot3.m0609_color_detector:main',
        ],
    },
)
