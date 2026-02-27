안돼면 당황하지말고 바로 chatgpt 떤지셈
260112_ 시뮬레이션 xacro 파일에 diff_drive plug_in 넣음 -> 속도 명령어를 각 바퀴에 회전 속도로 변환함

## 📚 시뮬레이션 작동 순서
 
 # 빌드
 colcon build --symlink-install
 # 환경 
 source install/setup.bash
 # 가제보 환경에서 불러오기
 ros2 launch relayrobot_description gazebo.launch.py

 # slam on
 ros2 launch relayrobot_description gazebo.launch.py use_sim_time:=true 
 
  ->이렇게 gazebo.launch.py를 실행해주고 맵도 띄워주고 그런다 . 근데 단 use_sim_time:=true는 필요함
 
 ros2 run teleop_twist_keyboard teleop_twist_keyboard  
 ros2 launch relayrobot_description cartographer.launch.py
  -> my_cartographer.lua (in relayrobot_description/confing 폴더에 저장 )이 파일을 만들기 
  
 # rviz 에서 한번 보자~ 
 rviz2
  add → bytopic → map 을 누르면서 현재 로봇이 움직이는 거에 따라서 어떻게 맵이 구성되는지확인가능함




## 📚 References (참고 자료)
* [ROS2 Humble 기본 세팅 튜토리얼]
  https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
* [DDSM]
  https://github.com/waveshareteam/ddsm_example
* [AWS Robomaker Hospital World]
   https://github.com/aws-robotics/aws-robomaker-hospital-world
