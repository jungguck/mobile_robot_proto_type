안되면 당황하지말고 바로 chatgpt 떤지셈  
260112_ 시뮬레이션 xacro 파일에 diff_drive plug_in 넣음 -> 속도 명령어를 각 바퀴에 회전 속도로 변환함

---

## 📚 시뮬레이션 작동 순서
 
### 1. 빌드 및 환경 설정
```bash
# 빌드
colcon build --symlink-install

# 환경 설정 적용
source install/setup.bash
```

### 2. 가제보(Gazebo) 실행
가제보 환경에서 맵과 로봇을 불러옴. SLAM 구동을 위해 `use_sim_time` 옵션 필요.
```bash
# 가제보 실행 (use_sim_time:=true 필수)
ros2 launch relayrobot_description gazebo.launch.py use_sim_time:=true 
```
> **Note:** gazebo.launch.py를 실행해주고 맵도 띄워줌.

### 3. 조종 및 매핑(SLAM) 실행
로봇을 움직여 실시간으로 지도를 생성.
```bash
# 키보드 조종 노드 실행
ros2 run teleop_twist_keyboard teleop_twist_keyboard  

# Cartographer 실행
ros2 launch relayrobot_description cartographer.launch.py
```
> **Note:** `my_cartographer.lua` 파일이 `relayrobot_description/config` 폴더에 저장되어 있어야 함.

### 4. RViz2 시각화 및 확인
```bash
rviz2
```
* **방법:** `add` → `bytopic` → `map` 클릭
* 로봇이 움직이는 경로에 따라 실시간으로 맵이 구성되는지 확인 가능.
---

## 📚 References (참고 자료)
* [ROS2 Humble 기본 세팅 튜토리얼]
  https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
* [DDSM]
  https://github.com/waveshareteam/ddsm_example
* [AWS Robomaker Hospital World]
   https://github.com/aws-robotics/aws-robomaker-hospital-world
