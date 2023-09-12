import os
from glob import glob
from setuptools import setup

package_name = 'chair_pose'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('models/*')),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='walleed',
    maintainer_email='walleedk@yorku.ca',
    description='Air Chair',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_node = chair_pose.pose_node:main',
            'camera = chair_pose.camera:main',
        ],
    },
)
