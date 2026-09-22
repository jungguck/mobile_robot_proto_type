import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # 1. 패키지 경로 얻기
    share_dir = get_package_share_directory('relayrobot_description')
    hospital_pkg = get_package_share_directory('aws_robomaker_hospital_world')
    
    # 2. 월드 파일 경로 설정/ ㅐ
    world_file_path = os.path.join(hospital_pkg, 'worlds', 'hospital.world')

    # 3. 모델 경로 및 환경변수 설정
    model_path = os.path.join(hospital_pkg, 'models')

    # (A) 모델 경로 추가 (이게 아까 삭제하라고 한 터미널 명령어를 파이썬식으로 바꾼 것)
    add_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=model_path
    )

    # (B) [필수] 인터넷 연결 끊기 (병원 맵 무한 로딩 방지용)
    disable_fuel = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )
    
    # 4. Xacro(로봇 모델) 파일 처리
    xacro_file = os.path.join(share_dir, 'urdf', 'relayrobot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # 5. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}]
    )

    # 6. Gazebo Server (시뮬레이션 엔진)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'true',         # 랙 방지를 위해 일시정지 상태로 시작
            'world': world_file_path # 병원 맵 로드
        }.items()
    )
    
    # 7. Gazebo Client (화면 GUI)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # 8. RViz2 실행 노드 (선택 사항: 필요하면 주석 해제)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # 9. 로봇 스폰 (Spawn) 노드
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'relayrobot',
            '-topic', 'robot_description',
            '-x', '0.0', 
            '-y', '0.0', 
            '-z', '0.2'
        ],
        output='screen'
    )

    return LaunchDescription([
        add_model_path,  # 모델 경로 설정
        disable_fuel,    # 인터넷 차단 (필수!)
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        # rviz_node,     # RViz도 같이 켜고 싶으면 주석 해제하세요
        urdf_spawn_node,
    ])
