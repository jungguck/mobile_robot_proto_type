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
	# main_driver(motor_node_1) 는 2026-09-06 에 제거했다.
	# 8/22 확정 부호 규약(DIR_R/DIR_L)과 9/3 확정 기구학 상수가 반영되지 않은
	# 레거시라 그대로 돌리면 로봇이 반대로 움직인다. 실제 드라이버는
	# relayrobot_description 의 real_robot_driver_260519 다.
	'odom_sub = relayrobot_driver.odom_subscriber:main', # 이 줄을 추가!
        ],
    },
)
