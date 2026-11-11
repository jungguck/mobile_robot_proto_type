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
        # 중복이라 주석 처리 (2026-11-01). 바로 위 glob('config/*') 가 이미
        # config/ 안의 모든 파일을 설치한다 — ekf.yaml, my_cartographer.lua 포함.
        # 이 줄이 있으면 같은 yaml 을 두 번 복사하게 된다.
        # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # ──────────────────────────────────────────────────────────────────────
    # entry_points — 터미널에서 'ros2 run' 으로 칠 수 있는 이름을 정의한다.
    #
    #   'ㅇㅇㅇ = 패키지.모듈:함수'
    #    └ 칠 이름   └ 어느 파일   └ 그 안의 어느 함수 (보통 main)
    #
    #   예:  ros2 run relayrobot_description real_robot_driver_260519
    #            └ 패키지 이름          └ 아래 왼쪽 이름
    #
    # ★ 이름을 추가·변경하면 'colcon build' 를 다시 해야 ros2 run 이 알아본다.
    #   install/ 안에 실행 스크립트가 새로 만들어지는 방식이기 때문이다.
    # ──────────────────────────────────────────────────────────────────────
    entry_points={
        'console_scripts': [

        # 모터 시리얼 통신 + 바퀴 엔코더 오도메트리. 이 패키지의 핵심 노드다.
        #   발행: /odom_raw (바퀴 적분 위치), /joint_states (RViz 바퀴 시각화)
        #   구독: /cmd_vel  (MPC 또는 teleop 의 속도 명령)
        #   ※ TF 는 발행하지 않는다 — odom→base_link 는 ekf_filter_node 담당.
        #      (이중 발행을 막으려고 일부러 그렇게 뒀다)
        #   보통은 단독으로 돌리지 않고 launch 로 띄운다:
        #      ros2 launch relayrobot_description real_robot_260519.launch.py
        'real_robot_driver_260519 = relayrobot_description.real_robot_driver_260519:main',

        # /odom 을 구독해서 x, y, yaw 를 터미널에 찍어 주는 확인용 노드.
        # 로봇을 굴려 보며 "값이 들어오고 있나" 를 눈으로 볼 때 쓴다.
        #   ※ PC 에서 원격으로 볼 때는 tools/odom_check.py 가 더 낫다
        #     (명령 vs 바퀴 vs EKF 를 나란히 비교해 준다).
        'odom_listener = relayrobot_description.odom_listener:main',

        # 기구학 상수 캘리브레이션. 로봇을 정해진 대로 굴린 뒤 줄자/실측값과
        # 비교해서 wheel_radius / wheel_base 보정치를 계산한다.
        #   직진 → wheel_radius:
        #     ros2 run relayrobot_description odom_calibrate --ros-args \
        #       -p mode:=straight -p speed:=0.15 -p duration:=10.0 -p measured:=<실측m>
        #   제자리 회전 → wheel_base:
        #     ros2 run relayrobot_description odom_calibrate --ros-args \
        #       -p mode:=rotate -p speed:=0.6 -p duration:=20.0 -p measured:=<실측deg>
        #
        #   ★ IMU 를 뺀 뒤(2026-11-01) 회전 캘리브레이션 수단이 이것뿐이다.
        #     odom_check.py 의 '회전 스케일' 자동 계산은 IMU 가 기준이었어서 사라졌다.
        #   ⚠️ 이 파일 docstring 의 "STAGE 2 / STAGE 3" 은 자체 번호다.
        #     프로젝트 STAGE 번호(RUNBOOK 기준)로는 둘 다 STAGE 4 에 해당한다.
        'odom_calibrate = relayrobot_description.odom_calibrate:main',

        ],
    },
)
