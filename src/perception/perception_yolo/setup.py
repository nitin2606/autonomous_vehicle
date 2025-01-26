import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'perception_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='knight',
    maintainer_email='nitinmaurya2606@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frame_subscriber = perception_yolo.frame_subscriber:main',
        ],
    },
)
