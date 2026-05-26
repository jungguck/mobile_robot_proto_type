from setuptools import find_packages, setup

package_name = 'relayrobot_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frmaster',
    maintainer_email='frmaster@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
         # 실행할 이름 = 패키지명.파일이름:main함수
	'main_driver = relayrobot_driver.motor_node_1:main',
	'odom_sub = relayrobot_driver.odom_subscriber:main', # 이 줄을 추가!
        ],
    },
)
