import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    # 0. 런치 인자
    #    오도메트리 검증(STAGE 4)에는 라이다가 필요 없다. 라이다를 안 꽂은 상태로
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

    # 3. [수정] 새로운 로봇 드라이버 (IMU 통합 버전)
    my_robot_driver = Node(
        package='relayrobot_description', # relayrobot_description 패키지 안에 있음
        executable='real_robot_driver_260519',
        name='real_robot_driver_260519',
        output='screen'
    )

    # 4. LiDAR 드라이버
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name = 'sllidar_node',
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

    # 5. IMU 드라이버 (항상 켜도록 설정)
    imu_node = Node(
        package='ebimu_pkg',
        executable='ebimu_publisher',
        name='ebimu_publisher',
        output='screen',
        parameters=[{
            'port': '/dev/ttyimu',
            'frame_id': 'base_link'
        }]
    )

    # 6. EKF 노드 (센서 융합)
    ekf_config_path = os.path.join(share_dir, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', 'odom')]
    )

    return LaunchDescription([
        declare_use_lidar,
        rsp_node,
        imu_node,
        my_robot_driver,
        lidar_node,
        ekf_node,
    ])
