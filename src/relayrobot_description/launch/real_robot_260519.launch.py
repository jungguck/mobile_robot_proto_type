import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

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
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/lidar',
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

    # 6. EKF 노드는 드라이버 자체에서 합치므로 선택사항입니다. 
    # 일단 꺼두고, 드라이버가 쏘는 odom만으로도 충분히 정확할 것입니다.

    return LaunchDescription([
        rsp_node,
        imu_node,
        my_robot_driver,
        lidar_node,
    ])
