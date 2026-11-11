import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    """실기 기동 런치 — 모터 + 라이다 + 바퀴 오도메트리.

    2026-11-01: IMU(ebimu_pkg)를 파이프라인에서 제거했다. 구성은
    "바퀴 엔코더 오도메트리 + 라이다" 두 개로 SLAM 까지 간다.

    노드 구성과 데이터 흐름:
        real_robot_driver_260519  --/odom_raw-->  ekf_filter_node --/odom--> (SLAM/MPC)
                                                         └--TF: odom→base_link
        sllidar_node              --/scan---->  (SLAM)
        robot_state_publisher     --TF: base_link→각 링크

    SLAM 은 이 런치에 들어있지 않다. 따로 띄운다:
        ros2 launch relayrobot_description cartographer.launch.py
    """

    # 0. 런치 인자
    #    오도메트리만 검증할 때는 라이다가 필요 없다. 라이다를 안 꽂은 상태로
    #    실행하면 /dev/rplidar 를 못 열어 에러 로그만 계속 쌓이므로 끌 수 있게 한다.
    #      ros2 launch relayrobot_description real_robot_260519.launch.py use_lidar:=false
    use_lidar = LaunchConfiguration('use_lidar')
    declare_use_lidar = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='LiDAR(sllidar_node) 실행 여부. 오도메트리만 볼 때는 false.'
    )

    # 1. URDF 설정
    share_dir = get_package_share_directory('relayrobot_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'relayrobot.xacro')
    doc = xacro.process_file(xacro_file)
    robot_urdf = doc.toxml()

    # 2. Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}],
        output='screen'
    )

    # 3. 로봇 드라이버 — 모터 시리얼 + 바퀴 오도메트리(/odom_raw)
    #    주의: 이 노드는 TF 를 발행하지 않는다. odom→base_link 는 아래 EKF 담당이다.
    my_robot_driver = Node(
        package='relayrobot_description',
        executable='real_robot_driver_260519',
        name='real_robot_driver_260519',
        output='screen'
    )

    # 4. LiDAR 드라이버
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        condition=IfCondition(use_lidar),
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 1000000,
            'frame_id': 'lidar_v1_1',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'DenseBoost'
        }]
    )

    # 5. EKF — /odom_raw 를 받아 /odom 과 odom→base_link TF 를 낸다.
    #    센서가 바퀴 하나뿐이라 실질적으로 패스스루다. 그래도 빼면 안 된다:
    #    TF(odom→base_link)를 내는 노드가 이것뿐이라 빼면 Cartographer 가 못 돈다.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(share_dir, 'config', 'ekf.yaml')],
        remappings=[('odometry/filtered', 'odom')]
    )

    return LaunchDescription([
        declare_use_lidar,
        rsp_node,
        my_robot_driver,
        lidar_node,
        ekf_node,
    ])
