from setuptools import setup
import os
from glob import glob

package_name = 'ros2_arduino_serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.perspective'))),    
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))         
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ana',
    maintainer_email='ana@traclabs.com',
    description='TODO: Package description',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'car_comm = ros2_arduino_serial.car_comm:main',
          'camera_feed = ros2_arduino_serial.camera_feed:main'
        ],
    },
)
