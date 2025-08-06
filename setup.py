from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'hand_gesture'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sallu',
    maintainer_email='166565@toyota-boshoku.co.jp',
    description='This package is for detecting hand gesture using Go2 camera and performing appropriate action',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_gesture_node = hand_gesture.hand_gesture_node:main',   #get code for hand gesture detection from "hand_gesture_node.py" inside "hand_gesture" package 
        ],
    },
)
