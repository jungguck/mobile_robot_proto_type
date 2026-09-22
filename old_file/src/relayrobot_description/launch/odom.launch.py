from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='relayrobot_description',
            executable='odom_listener',  # setup.py에서 정한 이름
            name='my_odom_listener',
            output='screen'              # 이걸 해야 터미널에 print 된 게 보입니다
        )
    ])