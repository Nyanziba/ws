from setuptools import find_packages, setup
import os
# globパッケージからglob関数を使用可能にする
from glob import glob

package_name = 'python_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('./launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koki',
    maintainer_email='koki@todo.todo',
    description='Python tutorial package for ROS 2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish = python_tutorial.publish:main',
            'image_sub = python_tutorial.image_sub:main',
            'image_realsense = python_tutorial.image_realsense:main',
            'simple_realsense = python_tutorial.simple_realsense:main',
            'simple_yolo = python_tutorial.simple_yolo:main',
            'cv_qrcode = python_tutorial.cv_qrcode:main',
            'stop_node = python_tutorial.stop:main',
            'realsense_track = python_tutorial.realsense_track:main',
            'center_controller_lugguge = python_tutorial.center_controller_lugguge:main',
            'person_detection_receptionist = python_tutorial.person_detection_receptionist:main',
            'chair_detection = python_tutorial.chair_detection:main',
            'test = python_tutorial.test:main',
        ],
    },
)
