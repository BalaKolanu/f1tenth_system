from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_stack'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hongrui Zheng',
    maintainer_email='billyzheng.bz@gmail.com',
    description='Onboard drivers for vesc and sensors for F1TENTH vehicles.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'throttle_interpolator = f1tenth_stack.throttle_interpolator:main',
            'speed_clipper = f1tenth_stack.speed_clipper:main',
            'tf_publisher = f1tenth_stack.tf_publisher:main',
            'scanmatching_tf_publisher = f1tenth_stack.scanmatching_tf_publisher:main',
            'tf_pose_reference_publisher = f1tenth_stack.tf_pose_reference_publisher:main',
            'imu_odom_fusion_node = f1tenth_stack.imu_odom_fusion_node:main',
            'linear_odom_calibrator = f1tenth_stack.linear_odom_calibrator:main',
            'joy_gated_localization = f1tenth_stack.joy_gated_localization:main',
            'usb_imu_serial_node = f1tenth_stack.usb_imu_serial_node:main',
            'imu_odom_fusion_node = f1tenth_stack.imu_odom_fusion_node:main',
        ],
    },
)
