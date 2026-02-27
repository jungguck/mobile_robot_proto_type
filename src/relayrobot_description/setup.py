from setuptools import setup
import os
from glob import glob


package_name = 'relayrobot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
	(os.path.join('share', package_name, 'config'), glob('config/*.yaml')), # confing에 지금 ekf.yaml이 있어요

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        
        'odom_listener = relayrobot_description.odom_listener:main',
        'real_driver = relayrobot_description.real_robot_driver:main',

        ],
    },
)
