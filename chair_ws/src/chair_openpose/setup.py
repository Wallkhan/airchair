import os
from glob import glob
from setuptools import setup

package_name = 'chair_openpose'

setup(
    name=package_name,
    version='3.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='walleed',
    maintainer_email='walleedk@my.yorku.ca',
    description='Air Chair',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'openpose_node = chair_openpose.openpose_node:main'
            'openpose_view = chair_openpose.openpose_view:main'
        ],
    },
)
