import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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

    #    IMU/EKF 사용 여부. 기본은 false — 2D SLAM 은 라이다 scan matching 이
    #    yaw 를 잡아주므로 IMU 없이 동작한다. 바퀴 오도메트리의 드리프트는
    #    cartographer 가 map→odom 으로 흡수한다.
    #      ros2 launch relayrobot_description real_robot_260519.launch.py use_imu:=true
    use_imu = LaunchConfiguration('use_imu')
    declare_use_imu = DeclareLaunchArgument(
        'use_imu',
        default_value='false',
        description='IMU(ebimu) + EKF 융합 사용 여부. false면 드라이버가 /odom 과 TF 를 직접 낸다.'
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

    # 3. 로봇 드라이버 (모터 + 바퀴 오도메트리)
    #
    #    TF(odom→base_link) 발행자는 시스템에 하나뿐이어야 한다. use_imu 에 따라
    #    그 책임이 드라이버와 EKF 사이를 오가므로, 파라미터만 다른 두 벌을 두고
    #    조건으로 하나만 띄운다.
    driver_common = dict(
        package='relayrobot_description',  # relayrobot_description 패키지 안에 있음
        executable='real_robot_driver_260519',
        name='real_robot_driver_260519',
        output='screen',
    )

    # IMU 없음(기본): 드라이버가 /odom 과 odom→base_link TF 를 직접 낸다
    driver_standalone = Node(
        condition=UnlessCondition(use_imu),
        parameters=[{'odom_topic': 'odom', 'publish_tf': True}],
        **driver_common
    )

    # IMU 사용: /odom_raw 만 내고, /odom 과 TF 는 EKF 가 만든다
    driver_for_ekf = Node(
        condition=IfCondition(use_imu),
        parameters=[{'odom_topic': 'odom_raw', 'publish_tf': False}],
        **driver_common
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

    # 5. IMU 드라이버 (use_imu:=true 일 때만)
    imu_node = Node(
        package='ebimu_pkg',
        executable='ebimu_publisher',
        name='ebimu_publisher',
        output='screen',
        condition=IfCondition(use_imu),
        parameters=[{
            'port': '/dev/ttyimu',
            'frame_id': 'base_link'
        }]
    )

    # 6. EKF 노드 (센서 융합) — use_imu:=true 일 때만.
    #    ekf.yaml 은 yaw 를 IMU 에 전적으로 맡기는 설정이라 IMU 없이 띄우면
    #    방향 소스가 사라진다. 그래서 IMU 와 항상 같이 켜고 같이 끈다.
    ekf_config_path = os.path.join(share_dir, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(use_imu),
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', 'odom')]
    )

    return LaunchDescription([
        declare_use_lidar,
        declare_use_imu,
        rsp_node,
        imu_node,
        driver_standalone,
        driver_for_ekf,
        lidar_node,
        ekf_node,
    ])
