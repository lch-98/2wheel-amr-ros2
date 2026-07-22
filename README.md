# 2-Wheel Differential Drive AMR (ROS2 Humble)

2륜 차동구동(differential drive) 자율이동로봇을 직접 설계·제작한 프로젝트입니다.
**시뮬레이션에서 지도를 그리고 자율주행 → 실물 로봇에서 지도를 그리고 자율주행**까지의 전체 과정을 다룹니다.

```
                 [ 로봇의 몸체 역할 ]                [ 그 위에 얹히는 두뇌 ]
시뮬레이션 :  Gazebo (가상 물리엔진)        →   slam_toolbox 기본 파라미터 (지도 그리기)
              + robot_state_publisher          my_nav2_params.yaml (자율주행)
              + teleop_twist_keyboard (표준 패키지)

실물 로봇  :  base_controller.py (모터 구동)    →   slam_params.yaml (지도 그리기, 커스텀)
              + ydlidar_node (라이다)                my_nav2_params_real.yaml (자율주행)
              + Arduino 펌웨어
              + robot_state.launch.py
              + keyboard_teleop.py (커스텀)
```

---

## 목차

1. [리포지토리 구조](#1-리포지토리-구조)
2. [사전 준비](#2-사전-준비)
3. [빌드](#3-빌드)
4. [A. 시뮬레이션 — 지도 그리기 (SLAM)](#4-a-시뮬레이션--지도-그리기-slam)
5. [B. 시뮬레이션 — Nav2 자율주행](#5-b-시뮬레이션--nav2-자율주행)
6. [C. 실물 — Arduino 펌웨어 업로드](#6-c-실물--arduino-펌웨어-업로드)
7. [D. 실물 — 지도 그리기 (SLAM)](#7-d-실물--지도-그리기-slam)
8. [E. 실물 — Nav2 자율주행](#8-e-실물--nav2-자율주행)
9. [하드웨어 스펙 & 파라미터](#9-하드웨어-스펙--파라미터)
10. [트러블슈팅](#10-트러블슈팅)

---

## 1. 리포지토리 구조

```
2wheel-amr-ros2/                 (= ~/robot_ws/src 그 자체)
├── my_robot/                    # ROS2 패키지 (ament_cmake + ament_cmake_python)
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── urdf/
│   │   └── my_robot.urdf.xacro
│   ├── launch/
│   │   ├── gazebo.launch.py        # Gazebo 실행 + 로봇 스폰 (시뮬 전용)
│   │   └── robot_state.launch.py   # URDF → TF 발행 (공통)
│   ├── config/
│   │   ├── slam_params.yaml           # 실물 전용 커스텀 SLAM 설정
│   │   ├── my_nav2_params.yaml        # Nav2 설정 (시뮬용)
│   │   └── my_nav2_params_real.yaml   # Nav2 설정 (실물용)
│   ├── my_robot/                   # 파이썬 노드
│   │   ├── base_controller.py         # 모터 구동 + odom 계산 (실물 전용)
│   │   └── keyboard_teleop.py         # 커스텀 teleop (실물 전용)
│   └── src/
│       └── ydlidar_node.cpp        # YDLIDAR 드라이버, 공식 SDK 링크 (실물 전용)
│
├── firmware/                    # Arduino 코드 (COLCON_IGNORE — colcon 빌드 대상 아님)
│   ├── EncoderTest_JGB37520/
│   ├── MotorTest_JGB37520/
│   ├── MotorEncoderTest_JGB37520/
│   └── MotorJGB37520_Firmware/     # 실사용 최종 펌웨어
│
├── maps/                         # COLCON_IGNORE — SLAM으로 그린 지도 저장소
│   ├── sim/
│   └── real/
│
└── docs/                          # COLCON_IGNORE — 발표자료, 사진 등
```

## 2. 사전 준비

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox ros-humble-gazebo-ros-pkgs \
                 ros-humble-teleop-twist-keyboard
```

**⚠️ YDLIDAR 공식 SDK는 별도로 미리 빌드해두셔야 합니다** (`ydlidar_node`가 이걸 링크합니다):
```bash
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build && cd build
cmake ..
make
sudo make install
# /usr/local/lib/libydlidar_sdk.a, /usr/local/include 에 설치되는지 확인
ls /usr/local/lib/ | grep ydlidar
```

## 3. 빌드

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/lch-98/2wheel-amr-ros2.git .

cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`~/.bashrc`에 추가 (매번 새 터미널마다 필요):
```bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source ~/robot_ws/install/setup.bash
```

---

## 4. A. 시뮬레이션 — 지도 그리기 (SLAM)

**터미널 1 — 로봇 몸체 켜기 (Gazebo)**
```bash
ros2 launch my_robot gazebo.launch.py
```
- 로봇은 `x=-2.0, y=-0.5, z=0.05`에 스폰됩니다.

**터미널 2 — 지도 그리기 시작**
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```
- 파라미터 파일 없이 `slam_toolbox` 기본값으로 실행합니다. Gazebo의 가상 라이다/odom은 노이즈가 거의 없어서 기본값으로 충분합니다.

**터미널 3 — 로봇 조종해서 맵 채우기**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**터미널 4 — RViz로 확인**
```bash
rviz2
```
- `Fixed Frame`을 `map`으로, `/map` 토픽 추가
- `Map`추가 `/map` 토픽 추가
- `RobotModel`추가 `/robot_description` 토픽 추가
- `LaserScan`추가 `/scan` 토픽 추가

**지도 저장**
```bash
mkdir -p ~/robot_ws/src/maps/sim
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/src/maps/sim/my_robot_map
```

---

## 5. B. 시뮬레이션 — Nav2 자율주행

**터미널 1 — 로봇 몸체 켜기**
```bash
ros2 launch my_robot gazebo.launch.py
```

**터미널 2 — Nav2 실행**
```bash
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=true \
    map:=$HOME/robot_ws/src/maps/sim/my_robot_map.yaml \
    params_file:=$(ros2 pkg prefix my_robot)/share/my_robot/config/my_nav2_params.yaml
```
- AMCL 초기 위치는 `my_nav2_params.yaml` 안에 `initial_pose.x=-2.0`, `initial_pose.y=-0.5`, `initial_pose.yaw=0.0`로 스폰 위치와 맞춰져 있습니다.

  ⚠️ 파라미터 키는 `initial_pose_x`가 아니라 **점(`.`) 표기**인 `initial_pose.x`여야 합니다.

**터미널 3 — RViz에서 목표 지점 클릭**
```bash
rviz2
```
- "`2D Goal Pose`로 지도 위 원하는 지점 클릭 → 자율주행 시작"
- `Fixed Frame`을 `map`으로, `/map` 토픽 추가
- `Map`추가 `global_costmap/costmap` 토픽 추가
- `Map`추가 `/local_costmap/costmap` 토픽 추가
- `RobotModel`추가 `/robot_description` 토픽 추가
- `LaserScan`추가 `/scan` 토픽 추가
---

## 6. C. 실물 — Arduino 펌웨어 업로드

Nav2, SLAM 전에 모터/엔코더부터 검증하세요. **반드시 이 순서대로** 업로드합니다.

1. `firmware/EncoderTest_JGB37520` 업로드
→ 시리얼 모니터에서 엔코더 카운트 증가/감소 확인
2. `firmware/MotorTest_JGB37520` 업로드
→ PWM으로 모터 방향/속도 확인
3. `firmware/MotorEncoderTest_JGB37520` 업로드
→ PID(`Kp=150, Ki=300, Kd=0`)로 속도 추종 확인
4. `firmware/MotorJGB37520_Firmware` 업로드
→ **최종 펌웨어.** Pi와 `"v/e"` 시리얼 프로토콜로 통신 (50Hz, watchdog 300ms)

---

## 7. D. 실물 — 지도 그리기 (SLAM)

**[Pi] 터미널 1~3 — 로봇 몸체 켜기**
```bash
ros2 launch my_robot robot_state.launch.py
ros2 run my_robot base_controller.py --ros-args -p port:=/dev/ttyUSB0
ros2 run my_robot ydlidar_node --ros-args -p port:=/dev/ttyUSB1
```
- 시작 전에 `ls -l /dev/ttyUSB*`로 Arduino(`ttyUSB0`, CH340)와 YDLIDAR(`ttyUSB1`, CP2102) 인식 확인

**[Pi] 터미널 4 — 지도 그리기 시작**
```bash
ros2 run slam_toolbox async_slam_toolbox_node \
    --ros-args --params-file $(ros2 pkg prefix my_robot)/share/my_robot/config/slam_params.yaml
```
- 실물은 라이다 노이즈/오도메트리 오차 때문에 **커스텀** `slam_params.yaml`을 반드시 지정합니다.

**[PC] 터미널 1 — 로봇 조종해서 맵 채우기**
```bash
ros2 run my_robot keyboard_teleop.py
```
- 시뮬에서 쓴 표준 패키지가 아니라, 실물 속도 특성에 맞춘 **커스텀** teleop입니다.

**[PC] 터미널 2 — RViz로 확인**
```bash
rviz2
```

**지도 저장**
```bash
mkdir -p ~/robot_ws/src/maps/real
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/src/maps/real/my_real_map
```
---

## 8. E. 실물 — Nav2 자율주행

**[Pi] 터미널 1~3 — 로봇 몸체 켜기**
```bash
ros2 launch my_robot robot_state.launch.py
ros2 run my_robot base_controller.py --ros-args -p port:=/dev/ttyUSB0
ros2 run my_robot ydlidar_node --ros-args -p port:=/dev/ttyUSB1
```

**[Pi] 터미널 4 — Nav2 실행**
```bash
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=false \
    map:=$HOME/robot_ws/src/maps/real/my_real_map.yaml \
    params_file:=$(ros2 pkg prefix my_robot)/share/my_robot/config/my_nav2_params_real.yaml
```

**[PC] 터미널 1 — RViz에서 목표 지점 클릭**
```bash
ros2 launch nav2_bringup rviz_launch.py
```
- 로봇 실제 위치와 AMCL 초기 포즈가 다르면 `2D Pose Estimate`로 먼저 보정하세요.
- `2D Goal Pose`로 목표 지점 클릭 → 자율주행 시작

---

## 9. 하드웨어 스펙 & 파라미터

| 항목 | 값 |
|---|---|
| 엔코더 CPR | 1320 (11 PPR × 4 quadrature × 감속비 30) |
| 바퀴 직경 | 65 mm |
| 바퀴 간격 | 0.185 m |
| 캐스터 반지름 | 15 mm |
| 라이다 높이(바닥 기준) | 200 mm |
| 라이다 x-offset | +35 mm |
| 섀시 크기 | 160 × 160 mm |
| 모터 핀 (Arduino) | `L_DIR=4, L_PWM=5, R_PWM=6, R_DIR=7` |
| 엔코더 핀 (Arduino) | `ENC_L_A=18, ENC_L_B=19, ENC_R_A=2, ENC_R_B=3` |
| PID | `Kp=150, Ki=300, Kd=0` |
| 피드포워드 | `PWM = 222.7 × speed + 3.4` |
| 통신 | `ROS_DOMAIN_ID=30`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` |

---

## 10. 트러블슈팅

- **cmd_vel이 실물에서만 최대 0.815초 지연** → 원인은 시리얼이 아니라 무선 환경 RELIABLE QoS의 재전송 큐잉. `/odom` 주기(49.3Hz, 지터 1.6ms)로 시리얼 결백을 데이터로 먼저 확인.
- **Arduino 속도값 이상** → `sscanf(%f)` 대신 `atof` 사용 (AVR에서 `sscanf %f` 오동작).
- **AMCL 초기 위치 미반영** → `initial_pose_x`가 아니라 `initial_pose.x`(점 표기) 확인.
- **좌/우 바퀴 반대로 도는 느낌** → `EncoderTest_JGB37520`로 먼저 방향 확인 후 배선/핀 교차 수정.
- **colcon build에서 0 packages 인식** → `my_robot/` 폴더 바로 밑에 `package.xml`, `CMakeLists.txt`가 있는지, `my_robot/my_robot/`에 `__init__.py`가 있는지 확인.
- **ydlidar_node 링크 에러** → `/usr/local/lib/libydlidar_sdk.a`가 실제로 존재하는지(`ls /usr/local/lib/ | grep ydlidar`), YDLidar SDK를 먼저 빌드·설치했는지 확인.

---

## License

MIT License
