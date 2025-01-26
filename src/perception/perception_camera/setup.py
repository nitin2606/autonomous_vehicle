import os
from glob import glob
from setuptools import find_packages, setup


package_name = 'perception_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all files in the launch folder
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all the config files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
   
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nitin',
    maintainer_email='nitinmaurya2606@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frame_publisher = perception_camera.frame_publisher:main', 
        ],
    },
)
