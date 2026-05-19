from setuptools import find_packages, setup

package_name = 'mpc_tubempc_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='relayrobot',
    maintainer_email='todo@todo.com',
    description='ROS2 bridge package for mpc_tubempc',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_tubempc_bridge = mpc_tubempc_bridge.bridge_node:main',
            'mpc_tubempc_path_planner = mpc_tubempc_bridge.path_planner:main',
        ],
    },
)
