import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    # 1. URDF 파일 읽기 (이건 시뮬레이션과 똑같음! 중요!)
    # -> 로봇의 사이즈, 센서 위치(Static TF)를 알려주기 위함
    share_dir = get_package_share_directory('relayrobot_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'relayrobot.xacro')
    doc = xacro.process_file(xacro_file)
    robot_urdf = doc.toxml()

    use_imu = LaunchConfiguration('use_imu', default='false')

    # 2. Robot State Publisher (필수)
    # -> URDF를 보고 base_link -> laser, base_link -> imu 같은 고정 TF를 방송함
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}],
        output='screen'
    )

    # 3. [주인공] 작성하신 로봇 구동 노드 (Motor Driver & Odom Pub)
    # -> 모터 시리얼 통신하고, odom -> base_link TF를 방송하는 바로 그 코드!
    my_robot_driver = Node(
        package='relayrobot_driver',  # 작성하신 패키지 이름
        executable='main_driver', # 작성하신 노드 실행 파일 이름
        output='screen'
    )

    # 4. 라이다(LiDAR) 드라이버 노드
    # -> 실제 라이다 장비를 켜서 /scan 토픽을 쏴주는 노드 (예: rplidar, ld08 등)
    lidar_node = Node(
        package='sllidar_ros2',    # 설치된 패키지 이름 
        executable='sllidar_node',    # 실행 파일 이름 
        name = 'sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/lidar',  # 포트 확인 필수! (ls -l /dev/ttyUSB*)
            'serial_baudrate': 1000000,      # 다른거면 매뉴얼 확인
            'frame_id': 'lidar_v1_1',      # URDF랑 이름 꼭 맞추기/ 보통 laser_frame 인데 내가 lidar_v1_1 으로 지정 
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'DenseBoost'        # [중요] S3 모델 필수 모드 추가
        }]
    )

    # 5. IMU 드라이버 노드 (옵션)
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

    # 6. odom 데이터를 받는 노드
    odom_node = Node(
        package='relayrobot_driver',
        executable='odom_sub',
        name='odom_sub',
        output='screen'
    )
           

	# 7. EKF 필터 노드 추가 (바퀴 Odom + IMU 합치기)
	# 설정 파일(.yaml)은 [relayrobot_description] 패키지에 들어있습니다.
    ekf_config_path = os.path.join(
        get_package_share_directory('relayrobot_description'),
        'config',
        'ekf.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('/odometry/filtered', '/odom')]
    )


    return LaunchDescription([
        rsp_node,
        my_robot_driver,
        lidar_node,
        imu_node,
        odom_node,
        ekf_node
    ])
