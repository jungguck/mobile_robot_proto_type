from setuptools import find_packages, setup

package_name = 'gui_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/gui_py']),
        ('share/gui_py', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='relayrobot',
    maintainer_email='todo@todo.com',
    description='ROS2 GUI package for teleoperation and SLAM control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_py = gui_py.main:main',
        ],
    },
)
