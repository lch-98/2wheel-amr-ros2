# 2-Wheel Differential Drive AMR (ROS2 Humble)

2륜 차동구동(differential drive) 자율이동로봇을 직접 설계·제작한 프로젝트입니다.<br>
**시뮬레이션에서 지도를 그리고 자율주행 → 실물 로봇에서 지도를 그리고 자율주행**까지의 전체 과정을 다룹니다.

```
                 [ 로봇의 몸체 역할 ]                      [ 그 위에 얹히는 두뇌 ]
시뮬레이션 :  Gazebo (가상 물리엔진)              →      slam_toolbox 기본 파라미터 (지도 그리기)
              + robot_state_publisher               my_nav2_params.yaml (자율주행)
              + teleop_twist_keyboard (표준 패키지)

실물 로봇  :  base_controller.py (모터 구동)     →      slam_params.yaml (지도 그리기, 커스텀)
              + ydlidar_node (라이다)                 my_nav2_params_real.yaml (자율주행)
              + Arduino 펌웨어
              + robot_state.launch.py
              + keyboard_teleop.py (커스텀)
              + 조이스틱 teleop (데드맨 스위치)
```

---

## 목차

0. [활용 플랫폼, 제어 구조 및 실행 영상](#0-활용-플랫폼-제어-구조-및-실행-영상)
1. [리포지토리 구조](#1-리포지토리-구조)
2. [사전 준비](#2-사전-준비)
3. [빌드](#3-빌드)
4. [A. 시뮬레이션 — 지도 그리기 (SLAM)](#4-a-시뮬레이션--지도-그리기-slam)
5. [B. 시뮬레이션 — Nav2 자율주행](#5-b-시뮬레이션--nav2-자율주행)
6. [C. 실물 — Arduino 펌웨어 업로드](#6-c-실물--arduino-펌웨어-업로드)
7. [D. 실물 — 조이스틱 Teleop 설정](#7-d-실물--조이스틱-teleop-설정)
8. [E. 실물 — 지도 그리기 (SLAM)](#8-e-실물--지도-그리기-slam)
9. [F. 실물 — Nav2 자율주행](#9-f-실물--nav2-자율주행)
10. [하드웨어 스펙 & 파라미터](#10-하드웨어-스펙--파라미터)
11. [트러블슈팅](#11-트러블슈팅)

---
## 0. 활용 플랫폼, 제어 구조 및 실행 영상

### 활용 플랫폼 (자체 제작)
<img width="966" height="1173" alt="로봇실사진" src="https://github.com/user-attachments/assets/a0a22a08-7569-4268-97eb-6ad8b9c7d071" />

- 메인제어기:   Raspberry pi4 8gb
- 하위제어기:   Arduino Mega 2560
- 라이다센서:   YDLidar X4
- 엔코더모터:   JGB37-520
- 모터드라이버:  MDD10A
- 보조배터리:   VOVA PD 22.5W
- 모터전원:     리튬이온배터리 3구
- 조이스틱:     8BitDo Ultimate 2 Wireless (2.4GHz, X-input)

### 제어 구조
<img width="1360" height="867" alt="1784736149446" src="https://github.com/user-attachments/assets/58b8942c-9393-4e6b-9252-99fa08f88767" />

### 실행 영상 (이미지를 클릭하면 시연 영상으로 이동합니다.)
<a href="https://blog.naver.com/dlcndgusgnss/224353754456">
  <img src="https://github.com/user-attachments/assets/bad077b1-2e96-4435-87e2-94bde6b8863d" width="700">
</a>


---

## 1. 리포지토리 구조

```
2wheel-amr-ros2/                       (= ~/robot_ws/src 그 자체)
├── my_robot/                          # ROS2 패키지 (ament_cmake + ament_cmake_python)
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── urdf/
│   │   └── my_robot.urdf.xacro
│   ├── launch/
│   │   ├── gazebo.launch.py           # Gazebo 실행 + 로봇 스폰 (시뮬 전용)
│   │   ├── robot_state.launch.py      # URDF → TF 발행 (공통)
│   │   └── joy_teleop.launch.py       # 조이스틱 teleop (실물 전용)
│   ├── config/
│   │   ├── slam_params.yaml           # 실물 전용 커스텀 SLAM 설정
│   │   ├── my_nav2_params.yaml        # Nav2 설정 (시뮬용)
│   │   ├── my_nav2_params_real.yaml   # Nav2 설정 (실물용)
│   │   ├── joy_teleop.yaml            # 조이스틱 축·버튼 매핑 및 속도 스케일
│   │   └── twist_mux.yaml             # cmd_vel 소스 우선순위
│   ├── my_robot/                      # 파이썬 노드
│   │   ├── base_controller.py         # 모터 구동 + odom 계산 (실물 전용)
│   │   └── keyboard_teleop.py         # 커스텀 teleop (실물 전용)
│   └── src/
│       └── ydlidar_node.cpp           # YDLIDAR 드라이버, 공식 SDK 링크 (실물 전용)
│
├── firmware/                          # Arduino 코드 (COLCON_IGNORE — colcon 빌드 대상 아님)
│   ├── EncoderTest_JGB37520/
│   ├── MotorTest_JGB37520/
│   ├── MotorEncoderTest_JGB37520/
│   └── MotorJGB37520_Firmware/        # 실사용 최종 펌웨어
│
├── maps/                              # COLCON_IGNORE — SLAM으로 그린 지도 저장소
│   ├── sim/
│   └── real/
│
└── docs/                              # COLCON_IGNORE — 발표자료, 사진 등
    └── xpad-8bitdo.service            # 조이스틱 xpad 자동 등록 (systemd)
```
---

## 2. 사전 준비

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox ros-humble-gazebo-ros-pkgs \
                 ros-humble-teleop-twist-keyboard \
                 ros-humble-joy ros-humble-teleop-twist-joy ros-humble-twist-mux
```

**YDLIDAR 공식 SDK는 별도로 미리 빌드해두셔야 합니다** (`ydlidar_node`가 이걸 링크합니다)
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
---

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
- 파라미터 파일 없이 `slam_toolbox` 기본값으로 실행합니다.<br>Gazebo의 가상 라이다/odom은 노이즈가 거의 없어서 기본값으로 충분합니다.

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
- AMCL 초기 위치는 `my_nav2_params.yaml` 안에 `initial_pose.x=-2.0`, `initial_pose.y=-0.5`, `initial_pose.yaw=0.0`로 스폰 위치와 맞춰져 있습니다.<br> 파라미터 키는 `initial_pose_x`가 아니라 **점(`.`) 표기**인 `initial_pose.x`여야 합니다.

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

## 7. D. 실물 — 조이스틱 Teleop 설정

SLAM으로 지도를 그릴 때 로봇을 직접 몰고 다녀야 하는데, 키보드보다 아날로그 스틱이 훨씬 섬세합니다.<br>
무엇보다 **데드맨 스위치(누르고 있을 때만 동작)** 를 걸 수 있어서, 펌웨어 개발 중 모터가 폭주해도 버튼만 놓으면 즉시 정지시킬 수 있습니다.

> 사용 컨트롤러: **8BitDo Ultimate 2 Wireless (2.4GHz 동글, X-input 모드)**<br>
> 헤드리스 로봇에서는 블루투스 페어링이 번거로우므로 **USB 동글 방식**을 권장합니다.

### D-1. 커널이 조이스틱을 인식하는지 확인

동글을 **라즈베리파이**에 꽂고 (PC가 아닙니다 — 로봇을 따라다녀야 하므로), 컨트롤러 전원을 켠 뒤:

```bash
ls -l /dev/input/js0
sudo dmesg | tail -20
```

**`Generic X-Box pad` 로그와 `/dev/input/js0`이 보이면 정상**입니다.<br>
대신 `Keyboard`/`Mouse`만 보이고 드라이버가 `hid-generic`이면 아래 D-2가 필요합니다.

### D-2. xpad 드라이버에 컨트롤러 ID 등록 (커널 6.15 미만인 경우)

8BitDo Ultimate 2 Wireless(`2dc8:310b`)는 **커널 6.15부터** xpad 드라이버가 기본 지원합니다.<br>
Ubuntu 22.04(커널 5.15)에서는 xpad가 이 ID를 몰라서 X-input 인터페이스에 드라이버가 안 붙습니다.

```bash
uname -r        # 6.15 미만이면 아래 등록 필요
sudo modprobe xpad
echo "2dc8 310b" | sudo tee /sys/bus/usb/drivers/xpad/new_id
ls -l /dev/input/js0        # 생성되면 성공
```

**재부팅 후에도 유지되도록 systemd 서비스 등록:**

```bash
sudo cp docs/xpad-8bitdo.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now xpad-8bitdo.service
```

`docs/xpad-8bitdo.service` 내용:
```ini
[Unit]
Description=Register 8BitDo Ultimate 2 Wireless with xpad driver
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'modprobe xpad && echo "2dc8 310b" > /sys/bus/usb/drivers/xpad/new_id'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

### D-3. 권한 설정 (필수)

`joy_node`는 SDL을 통해 `/dev/input/event*`를 읽는데, 이 장치는 **`input` 그룹 권한**이 필요합니다.<br>
(`jstest`가 읽는 `/dev/input/js0`은 others 읽기가 열려 있어서, jstest는 되는데 joy_node만 안 되는 상황이 흔합니다.)

```bash
groups                      # input 이 목록에 있는지 확인
sudo usermod -aG input $USER
# 로그아웃 후 재로그인 필요
```

### D-4. 입력 확인 및 매핑

```bash
sudo apt install joystick
jstest /dev/input/js0
```

스틱과 버튼을 하나씩 움직이며 인덱스를 확인합니다. 본 프로젝트의 확정값:

| 조작 | 인덱스 | 방향 |
|---|---|---|
| 전진/후진 (왼쪽 스틱 상하) | `axes[1]` | 앞 = 양수 |
| 회전 (오른쪽 스틱 좌우) | `axes[3]` | 왼쪽 = 양수 |
| 데드맨 스위치 (LB) | `buttons[4]` | 누르는 동안만 동작 |
| 부스트 (RB) | `buttons[5]` | — |

축·버튼 인덱스는 컨트롤러마다 다르므로 **반드시 직접 측정**하세요.<br>
부호가 ROS 규약(REP-103: 앞 = +x, 좌회전 = +yaw)과 반대면 `scale_*`에 음수를 주면 됩니다.

### D-5. 실행

```bash
ros2 launch my_robot joy_teleop.launch.py
```

노드 구성:
```
joy_node → /joy → teleop_twist_joy → /cmd_vel_joy ─┐
                                                    ├─ twist_mux → /cmd_vel
             (향후) Nav2 → /cmd_vel_nav ────────────┘
```

`twist_mux`는 조이스틱(priority 100)을 Nav2(priority 10)보다 우선하도록 설정되어 있어,<br>
자율주행 중에도 사람이 개입하면 즉시 수동 제어로 넘어갑니다.

### D-6. 검증 (안전 필수)

**반드시 바퀴를 공중에 띄운 상태에서 시작하세요.**

```bash
ros2 topic echo /cmd_vel
```

| 순서 | 확인 | 기대 결과 |
|---|---|---|
| 1 | LB **안 누르고** 스틱 조작 | 아무것도 발행되지 않음 |
| 2 | LB 누르고 스틱 끝까지 밀기 | `linear.x`가 설정 스케일(0.22)과 일치 |
| 3 | LB 누른 채 앞으로 | `linear.x` 양수 |
| 4 | LB 누른 채 오른쪽 스틱 왼쪽 | `angular.z` 양수 |
| 5 | **LB 놓는 순간** | 즉시 0 ← **가장 중요** |

`base_controller`를 켠 상태에서 `/odom` 주기가 유지되는지도 확인하면 좋습니다.
```bash
ros2 topic hz /odom --window 50   # 49Hz 근처, 지터 1~2ms 유지되면 정상
```

---

## 8. E. 실물 — 지도 그리기 (SLAM)

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

**[Pi] 터미널 5 — 로봇 조종해서 맵 채우기 (조이스틱)**
```bash
ros2 launch my_robot joy_teleop.launch.py
```
- LB를 누른 채 스틱으로 조종합니다. 아날로그 입력이라 키보드보다 섬세하게 맵을 채울 수 있습니다.
- 조이스틱 없이 진행하려면 PC에서 커스텀 키보드 teleop을 대신 실행하세요:
  ```bash
  ros2 run my_robot keyboard_teleop.py
  ```

**[PC] 터미널 1 — RViz로 확인**
```bash
rviz2
```

**지도 저장**
```bash
mkdir -p ~/robot_ws/src/maps/real
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/src/maps/real/my_real_map
```
---

## 9. F. 실물 — Nav2 자율주행

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

> **참고 — 조이스틱과 Nav2를 함께 쓰려면**<br>
> Nav2는 기본적으로 `/cmd_vel`에 직접 발행하므로 twist_mux를 우회합니다.<br>
> 자율주행 중 조이스틱 개입을 원한다면 Nav2 출력을 `/cmd_vel_nav`로 리맵해야 합니다 (`SetRemap` 사용).

---

## 10. 하드웨어 스펙 & 파라미터

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
| 조이스틱 | 8BitDo Ultimate 2 Wireless, VID:PID `2dc8:310b` |
| 조이스틱 속도 스케일 | linear 0.22 m/s, angular 1.0 rad/s |

---

## 11. 트러블슈팅

- **cmd_vel이 실물에서만 최대 0.815초 지연** → 원인은 시리얼이 아니라 무선 환경 RELIABLE QoS의 재전송 큐잉. `/odom` 주기(49.3Hz, 지터 1.6ms)로 시리얼 결백을 데이터로 먼저 확인.
- **Arduino 속도값 이상** → `sscanf(%f)` 대신 `atof` 사용 (AVR에서 `sscanf %f` 오동작).
- **AMCL 초기 위치 미반영** → `initial_pose_x`가 아니라 `initial_pose.x`(점 표기) 확인.
- **좌/우 바퀴 반대로 도는 느낌** → `EncoderTest_JGB37520`로 먼저 방향 확인 후 배선/핀 교차 수정.
- **colcon build에서 0 packages 인식** → `my_robot/` 폴더 바로 밑에 `package.xml`, `CMakeLists.txt`가 있는지, `my_robot/my_robot/`에 `__init__.py`가 있는지 확인.
- **ydlidar_node 링크 에러** → `/usr/local/lib/libydlidar_sdk.a`가 실제로 존재하는지(`ls /usr/local/lib/ | grep ydlidar`), YDLidar SDK를 먼저 빌드·설치했는지 확인.
- **조이스틱이 Keyboard/Mouse로만 잡힘 (`/dev/input/js0` 없음)** → 커널 6.15 미만에서 xpad가 `2dc8:310b`를 모르는 것. `echo "2dc8 310b" > /sys/bus/usb/drivers/xpad/new_id`로 등록 (D-2 참고).
- **`jstest`는 되는데 `joy_node`만 입력이 안 옴** → `joy_node`는 `/dev/input/event*`를 읽으므로 `input` 그룹 권한 필요. `sudo usermod -aG input $USER` 후 재로그인.
- **조이스틱을 꽂았는데 아무 반응 없음** → 8BitDo는 일정 시간 후 자동 절전. 전원 버튼을 눌러 깨운 뒤 launch를 재실행하세요 (joy_node는 시작 시점에 장치를 열기 때문).
- **`/joy`가 1500Hz로 발행됨** → 컨트롤러 폴링레이트가 1000Hz라 정상 동작. `base_controller`가 자체 타이머로 시리얼을 보내므로 제어에는 영향 없음(`/odom` 49Hz 유지 확인). 줄이려면 8BitDo Ultimate Software에서 폴링레이트를 낮추거나 `topic_tools throttle` 사용.

---

## License

MIT License
