from setuptools import setup
import os
from glob import glob

package_name = 'hamals_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],

    data_files=[
        # ROS 2 package index
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        # package.xml
        (
            'share/' + package_name,
            ['package.xml']
        ),

        # launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),

        # config files (odom_tf.yaml, slam.yaml, vb.)
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='m_gnr',
    maintainer_email='m_gnr@icloud.com',

    description=(
        'hamals_localization: '
        'Publishes TF (odom -> base_link) from MCU-provided raw odometry. '
        'Minimal localization layer, SLAM/Nav2 compatible.'
    ),

    license='MIT',

    entry_points={
        'console_scripts': [
            'odom_tf_node = hamals_localization.odom_tf_node:main',
        ],
    },
)