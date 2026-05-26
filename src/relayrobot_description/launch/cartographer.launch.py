import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 패키지 및 설정 파일 경로 잡기
    pkg_name = 'relayrobot_description'
    
    # config 폴더 경로 (여기에 my_cartographer.lua가 있어야 함)
    config_dir = os.path.join(get_package_share_directory(pkg_name), 'config')
    config_file = 'my_cartographer.lua'

    # 2. 시뮬레이션 시간 설정 (매우 중요)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 3. 카토그래퍼 노드 설정
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', config_dir,
                   '-configuration_basename', config_file]
    )

    # 4. 그리드맵 노드 설정 (지도를 2D 그림으로 바꿔줌)
    grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': 0.05,
            'publish_period_sec': 1.0
        }]
    )

    # 5. 실행 목록 리턴
    return LaunchDescription([
        cartographer_node,
        grid_node
    ])