from glob import glob
import os
from setuptools import setup

package_name = 'chair'

setup(
    name=package_name,
    version='3.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('icons/*')),
        (os.path.join('share', package_name), glob('worlds/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jenkin',
    maintainer_email='jenkin@yorku.ca',
    description='Air Chair',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_target = chair.aruco_target:main',
            'view_camera = chair.view_camera:main',
            'opencv_camera = chair.opencv_camera:main',
            'chair_controller = chair.chair_controller:main',
            'aruco_board_detect = chair.aruco_board_detect:main',
            'chair_ui = chair.chair_ui:main',
            'chair_follower = chair.chair_follower:main',
            'tester = chair.tester:main',
        ],
    },
)
