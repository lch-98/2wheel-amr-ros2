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
              + imu_node.py (MPU6050) + EKF 융합
              + robot_state.launch.py
              + keyboard_teleop.py (커스텀)
              + 조이스틱 teleop (데드맨 스위치)
```

> **오도메트리 개선(EKF):** 바퀴 엔코더만 쓰던 오도메트리에 MPU6050 IMU의 자이로(yaw rate)를
> `robot_localization`의 EKF로 융합했습니다. 제자리 360° 회전 시 바퀴만 쓴 오도메트리는 약 27.8°의
> 오차가 났지만, IMU 융합 후 약 1.4°로 **회전 오차가 약 20배 개선**되었습니다.

---

## 목차

0. [활용 플랫폼, 제어 구조 및 실행 영상](#0-활용-플랫폼-제어-구조-및-실행-영상)
1. [리포지토리 구조](#1-리포지토리-구조)
2. [사전 준비](#2-사전-준비)
3. [빌드](#3-빌드)
4. [A. 시뮬레이션 — 지도 그리기 (SLAM)](#4-a-시뮬레이션--지도-그리기-slam)
5. [B. 시뮬레이션 — Nav2 자율주행](#5-b-시뮬레이션--nav2-자율주행)
6. [C. 실물 — Arduino 펌웨어 업로드](#6-c-실물--arduino-펌웨어-업로드)
7. [D. 실물 — USB 포트 고정 (udev)](#7-d-실물--usb-포트-고정-udev)
8. [E. 실물 — 조이스틱 Teleop 설정](#8-e-실물--조이스틱-teleop-설정)
9. [F. 실물 — IMU(MPU6050) + EKF 오도메트리 융합](#9-f-실물--imumpu6050--ekf-오도메트리-융합)
10. [G. 실물 — 통합 Launch 파일 & RViz 설정](#10-g-실물--통합-launch-파일--rviz-설정)
11. [H. 실물 — 지도 그리기 (SLAM)](#11-h-실물--지도-그리기-slam)
12. [I. 실물 — Nav2 자율주행](#12-i-실물--nav2-자율주행)
13. [하드웨어 스펙 & 파라미터](#13-하드웨어-스펙--파라미터)
14. [트러블슈팅](#14-트러블슈팅)

---
## 0. 활용 플랫폼, 제어 구조 및 실행 영상

### 활용 플랫폼 (자체 제작)
<img width="966" height="1173" alt="로봇실사진" src="https://github.com/user-attachments/assets/a0a22a08-7569-4268-97eb-6ad8b9c7d071" />

- 메인제어기:   Raspberry pi4 8gb
- 하위제어기:   Arduino Mega 2560
- 라이다센서:   YDLidar X4
- IMU 센서:    MPU6050 (6축, I2C)
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
│   │   ├── robot_state.launch.py      # URDF → TF 발행 (실물에서 bringup이 호출; 시뮬은 gazebo.launch.py에 포함)
│   │   ├── joy_teleop.launch.py       # 조이스틱 teleop (실물 전용)
│   │   ├── bringup_real.launch.py     # 실물 통합: 하드웨어 + IMU + EKF + 조이스틱
│   │   ├── mapping.launch.py          # bringup + SLAM (실물 지도 그리기)
│   │   └── navigation.launch.py       # bringup + Nav2 (실물 자율주행)
│   ├── config/
│   │   ├── slam_params.yaml           # 실물 전용 커스텀 SLAM 설정
│   │   ├── my_nav2_params.yaml        # Nav2 설정 (시뮬용)
│   │   ├── my_nav2_params_real.yaml   # Nav2 설정 (실물용)
│   │   ├── joy_teleop.yaml            # 조이스틱 축·버튼 매핑 및 속도 스케일
│   │   ├── twist_mux.yaml             # cmd_vel 소스 우선순위 (joystick 100 / navigation 10 → /cmd_vel_out)
│   │   └── ekf.yaml                   # robot_localization EKF 설정 (바퀴+IMU 융합)
│   ├── rviz/
│   │   ├── mapping.rviz               # 지도 그리기용 RViz 구성
│   │   └── nav.rviz                   # 자율주행 확인용 RViz 구성 (QoS·색상 저장)
│   ├── my_robot/                      # 파이썬 노드
│   │   ├── base_controller.py         # 모터 구동 + odom 계산 (실물 전용)
│   │   ├── imu_node.py                # MPU6050 IMU 드라이버 (실물 전용)
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
    ├── xpad-8bitdo.service            # 조이스틱 xpad 자동 등록 (systemd)
    └── 99-amr-usb.rules               # USB 포트 고정 udev 규칙
```
---

## 2. 사전 준비

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox ros-humble-gazebo-ros-pkgs \
                 ros-humble-teleop-twist-keyboard \
                 ros-humble-joy ros-humble-teleop-twist-joy ros-humble-twist-mux \
                 ros-humble-robot-localization \
                 i2c-tools python3-smbus2
```

**IMU(MPU6050)를 쓰려면 라즈베리파이에서 I2C를 활성화하세요:**
```bash
sudo raspi-config    # Interface Options → I2C → Enable → 재부팅
i2cdetect -y 1       # 0x68 이 보이면 MPU6050 인식 성공
```

**YDLIDAR 공식 SDK는 별도로 미리 빌드해두셔야 합니다** (`ydlidar_node`가 이걸 링크합니다).
이 작업은 **[Pi]** 에서 하며, 클론 위치는 어디든 상관없습니다 (예: 홈 디렉터리).
```bash
cd ~                 # 홈 디렉터리(또는 원하는 작업 폴더)에서
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

> 이 저장소는 **[PC](시뮬레이션용)와 [Pi](실물용) 양쪽에 각각 클론·빌드**합니다.
> 아래 과정을 두 기기에서 동일하게 수행하세요. (단, YDLIDAR SDK와 IMU I2C 설정은 [Pi]에서만 필요합니다.)

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/lch-98/2wheel-amr-ros2.git .

cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

> `git clone ... .` 의 **마지막 점(`.`)** 은 "현재 폴더에 바로 클론"하라는 뜻입니다.
> 이 저장소는 루트가 곧 `~/robot_ws/src`가 되도록 구성되어 있어, 클론 후 바로 `colcon build`가 됩니다.

`~/.bashrc`에 추가 (매번 새 터미널마다 필요):
```bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source ~/robot_ws/install/setup.bash
```

> **📁 이 문서의 명령 실행 경로 규칙 (중요)**
>
> 이 README의 모든 명령은 아래 두 가지만 지키면 어느 폴더에서 실행하든 동작합니다.
>
> 1. **새 터미널을 열면 항상 먼저 아래를 실행**하세요 (위 `.bashrc` 설정을 안 했다면):
>    ```bash
>    cd ~/robot_ws
>    source install/setup.bash
>    ```
> 2. **`ros2 run` / `ros2 launch` 명령**은 위 `source`만 되어 있으면 **어느 폴더에서든** 동작합니다
>    (ROS2가 설치된 패키지를 자동으로 찾기 때문). 굳이 특정 폴더로 이동할 필요가 없습니다.
> 3. **`sudo cp docs/...` 처럼 파일을 직접 복사하는 명령**은 리포지토리를 클론한 폴더
>    (`~/robot_ws/src`) 안에 있어야 합니다. 이런 명령 앞에는 항상 `cd ~/robot_ws/src`를 함께 적어두었습니다.
>
> **[Pi]** 는 라즈베리파이에서, **[PC]** 는 로컬 PC에서 실행하는 명령입니다.
> 라즈베리파이는 SSH로 접속해 여러 터미널을 열고, RViz 같은 무거운 GUI는 PC에서 실행합니다.

---

## 4. A. 시뮬레이션 — 지도 그리기 (SLAM)

시뮬레이션은 전부 **[PC]** 에서 실행합니다. 아래 터미널 1~4는 **각각 새 터미널을 열고**,
매번 먼저 `cd ~/robot_ws && source install/setup.bash`를 실행한 뒤 명령을 입력하세요.

**[PC] 터미널 1 — 로봇 몸체 켜기 (Gazebo)**
```bash
ros2 launch my_robot gazebo.launch.py
```
- 로봇은 `x=-2.0, y=-0.5, z=0.05`에 스폰됩니다.

**[PC] 터미널 2 — 지도 그리기 시작**
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```
- 파라미터 파일 없이 `slam_toolbox` 기본값으로 실행합니다.<br>Gazebo의 가상 라이다/odom은 노이즈가 거의 없어서 기본값으로 충분합니다.

**[PC] 터미널 3 — 로봇 조종해서 맵 채우기**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**[PC] 터미널 4 — RViz로 확인**
```bash
rviz2
```
- `Fixed Frame`을 `map`으로, `/map` 토픽 추가
- `Map`추가 `/map` 토픽 추가
- `RobotModel`추가 `/robot_description` 토픽 추가
- `LaserScan`추가 `/scan` 토픽 추가

**[PC] 지도 저장** (새 터미널에서)
```bash
mkdir -p ~/robot_ws/src/maps/sim
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/src/maps/sim/my_robot_map
```
- 저장 경로는 절대경로라 어느 폴더에서 실행해도 `~/robot_ws/src/maps/sim/`에 저장됩니다.

---

## 5. B. 시뮬레이션 — Nav2 자율주행

시뮬레이션은 전부 **[PC]** 에서 실행합니다. 각 터미널마다 새로 열고
`cd ~/robot_ws && source install/setup.bash`를 먼저 실행하세요.

**[PC] 터미널 1 — 로봇 몸체 켜기**
```bash
ros2 launch my_robot gazebo.launch.py
```

**[PC] 터미널 2 — Nav2 실행**
```bash
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=true \
    map:=$HOME/robot_ws/src/maps/sim/my_robot_map.yaml \
    params_file:=$(ros2 pkg prefix my_robot)/share/my_robot/config/my_nav2_params.yaml
```
- `$HOME`, `$(ros2 pkg prefix ...)`를 쓰므로 **어느 폴더에서 실행해도** 경로가 자동으로 맞춰집니다.
- AMCL 초기 위치는 `my_nav2_params.yaml` 안에 `initial_pose.x=-2.0`, `initial_pose.y=-0.5`, `initial_pose.yaw=0.0`로 스폰 위치와 맞춰져 있습니다.<br> 파라미터 키는 `initial_pose_x`가 아니라 **점(`.`) 표기**인 `initial_pose.x`여야 합니다.

**[PC] 터미널 3 — RViz에서 목표 지점 클릭**
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

Arduino IDE로 업로드합니다. 펌웨어 소스는 리포지토리의 **`~/robot_ws/src/firmware/`** 폴더에 있습니다
(각 폴더의 `.ino` 파일을 Arduino IDE로 열어 업로드).<br>
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

## 7. D. 실물 — USB 포트 고정 (udev)

USB 포트 번호(`/dev/ttyUSB0`, `ttyUSB1`)는 재부팅·재연결 시 **아두이노와 라이다가 뒤바뀔 수 있습니다.**
udev 규칙으로 VID:PID 기반 고정 이름을 만들어야, 이후 모든 단계에서 `/dev/arduino`, `/dev/ydlidar`를
안정적으로 쓸 수 있습니다.

> **이 단계는 이후 모든 실물 작업(조이스틱·IMU·SLAM·Nav2)의 전제조건입니다.** 반드시 먼저 수행하세요.
> 아래 launch 파일과 실행 명령들이 전부 `/dev/arduino`, `/dev/ydlidar`를 기본값으로 사용합니다.

```bash
# [Pi] 리포지토리를 클론한 폴더로 이동 (docs 폴더가 여기 있음)
cd ~/robot_ws/src

sudo cp docs/99-amr-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/arduino /dev/ydlidar     # 심볼릭 링크 확인
```

`docs/99-amr-usb.rules` 내용:
```
# YDLIDAR X4 (CP2102) → /dev/ydlidar
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ydlidar"
# Arduino Mega (CH340) → /dev/arduino
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="arduino"
```

- 링크가 안 생기면 USB를 뽑았다 다시 꽂거나 재부팅하세요 (udev는 장치 연결 시점에 규칙을 적용).
- **VID:PID를 확인하려면** (다른 아두이노/라이다를 쓸 경우): `udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct" | head -2`

이제 USB를 어느 순서로 꽂아도 **`/dev/arduino`, `/dev/ydlidar`가 항상 올바른 장치**를 가리킵니다.

---

## 8. E. 실물 — 조이스틱 Teleop 설정

SLAM으로 지도를 그릴 때 로봇을 직접 몰고 다녀야 하는데, 키보드보다 아날로그 스틱이 훨씬 섬세합니다.<br>
무엇보다 **데드맨 스위치(누르고 있을 때만 동작)** 를 걸 수 있어서, 펌웨어 개발 중 모터가 폭주해도 버튼만 놓으면 즉시 정지시킬 수 있습니다.

> 사용 컨트롤러: **8BitDo Ultimate 2 Wireless (2.4GHz 동글, X-input 모드)**<br>
> 헤드리스 로봇에서는 블루투스 페어링이 번거로우므로 **USB 동글 방식**을 권장합니다.

> **이 섹션(E-1 ~ E-6)의 모든 명령은 [Pi]** 에서 실행합니다 (동글이 Pi에 꽂혀 있으므로).
> `ros2 run`/`ros2 launch`가 포함된 명령 전에는 `cd ~/robot_ws && source install/setup.bash`를 먼저 실행하세요.

### E-1. 커널이 조이스틱을 인식하는지 확인

동글을 **라즈베리파이**에 꽂고 (PC가 아닙니다 — 로봇을 따라다녀야 하므로), 컨트롤러 전원을 켠 뒤:

```bash
ls -l /dev/input/js0
sudo dmesg | tail -20
```

**`Generic X-Box pad` 로그와 `/dev/input/js0`이 보이면 정상**입니다.<br>
대신 `Keyboard`/`Mouse`만 보이고 드라이버가 `hid-generic`이면 아래 E-2가 필요합니다.

### E-2. xpad 드라이버에 컨트롤러 ID 등록 (커널 6.15 미만인 경우)

8BitDo Ultimate 2 Wireless(`2dc8:310b`)는 **커널 6.15부터** xpad 드라이버가 기본 지원합니다.<br>
Ubuntu 22.04(커널 5.15)에서는 xpad가 이 ID를 몰라서 X-input 인터페이스에 드라이버가 안 붙습니다.

```bash
uname -r                    # 6.15 미만이면 아래 등록 필요
sudo modprobe xpad
echo "2dc8 310b" | sudo tee /sys/bus/usb/drivers/xpad/new_id
ls -l /dev/input/js0        # 생성되면 성공
```

**재부팅 후에도 유지되도록 systemd 서비스 등록:**

```bash
# [Pi] 리포지토리를 클론한 폴더로 이동 (docs 폴더가 여기 있음)
cd ~/robot_ws/src

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

### E-3. 권한 설정 (필수)

`joy_node`는 SDL을 통해 `/dev/input/event*`를 읽는데, 이 장치는 **`input` 그룹 권한**이 필요합니다.<br>
(`jstest`가 읽는 `/dev/input/js0`은 others 읽기가 열려 있어서, jstest는 되는데 joy_node만 안 되는 상황이 흔합니다.)

```bash
groups                        # input 이 목록에 있는지 확인
sudo usermod -aG input $USER
# 로그아웃 후 재로그인 필요
```

### E-4. 입력 확인 및 매핑

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

### E-5. 실행

```bash
ros2 launch my_robot joy_teleop.launch.py
```

노드 구성 (조이스틱만 켰을 때):
```
joy_node → /joy → teleop_twist_joy → /cmd_vel_joy → twist_mux → /cmd_vel_out → base_controller
```

`twist_mux`는 조이스틱(priority 100)을 navigation(priority 10)보다 우선하도록 설정되어 있어,<br>
자율주행 중에도 사람이 개입하면 즉시 수동 제어로 넘어갑니다. (Nav2 연동 배선은 I단계 참고)

> **twist_mux 출력은 `/cmd_vel`이 아니라 `/cmd_vel_out`입니다.** Nav2의 velocity_smoother가 `/cmd_vel`을
> 점유하기 때문에, 최종 명령 토픽을 `/cmd_vel_out`으로 분리하고 base_controller가 이를 구독합니다 (I단계에서 상술).

### E-6. 검증 (안전 필수)

**반드시 바퀴를 공중에 띄운 상태에서 시작하세요.**

```bash
ros2 topic echo /cmd_vel_out    # twist_mux의 최종 출력 (base_controller가 받는 토픽)
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

## 9. F. 실물 — IMU(MPU6050) + EKF 오도메트리 융합

바퀴 엔코더 오도메트리는 **미끄러짐(slip)** 에 취약합니다. 바퀴는 돌았는데 로봇은 안 움직인 경우
odom은 "이동했다"고 계산하죠. 특히 제자리 회전에서 오차가 큽니다.<br>
MPU6050의 **자이로 Z축(yaw rate)** 은 바퀴와 무관하게 실제 회전을 측정하므로, 이를 융합하면 회전 추정이 크게 개선됩니다.

> **설계 원칙:** EKF에는 IMU의 **자이로 Z(yaw rate)만** 융합합니다. 가속도계는 로봇 진동 노이즈가 심해 제외합니다.
> yaw 드리프트는 AMCL(라이다+지도)이 절대 위치로 보정하므로, 지자기 센서가 없는 MPU6050으로 충분합니다.

> **이 섹션(F)의 모든 명령은 [Pi]** 에서 실행합니다 (IMU가 Pi에 연결되어 있으므로).
> `ros2 run`이 포함된 명령 전에는 `cd ~/robot_ws && source install/setup.bash`를 먼저 실행하세요.

### F-1. 배선 및 인식

| MPU6050 | Raspberry Pi 물리 핀 |
|---|---|
| VCC | 3.3V (1번 핀) |
| GND | GND (6번 핀) |
| SDA | GPIO2 / SDA (3번 핀) |
| SCL | GPIO3 / SCL (5번 핀) |

```bash
i2cdetect -y 1     # 0x68 이 보이면 성공
```

### F-2. IMU 노드 실행 및 검증

`imu_node.py`는 smbus2로 MPU6050 레지스터를 직접 읽어 `/imu/data_raw`(50Hz)를 발행합니다.
DLPF(44Hz)로 진동을 칩 레벨에서 필터링하고, 시작 시 정지 상태에서 **자이로 바이어스를 자동 캘리브레이션**합니다.

```bash
# [Pi] 새 터미널에서
ros2 run my_robot imu_node.py

# [Pi] 또 다른 새 터미널에서 (값 확인용)
ros2 topic echo /imu/data_raw --field angular_velocity.z
```
- 정지 시 `angular_velocity.z`가 **0 근처(±0.001)** → 바이어스 보정 정상
- 로봇을 **왼쪽(반시계)으로 돌릴 때 양수** → REP-103 준수 (부호 보정 불필요)

### F-3. URDF에 imu_link

`my_robot.urdf.xacro`에 실제 장착 위치를 반영한 `imu_link`가 정의되어 있습니다
(base_link 기준 `x=0.025, y=-0.055, z=0.0975`, 방향 `rpy=0 0 0`).<br>
자이로만 융합하므로 위치(xyz)는 회전 측정에 영향이 없고, **방향(rpy)만 정확하면** 됩니다.

### F-4. TF 트리 정리 (가장 중요)

**`odom → base_footprint` TF는 오직 하나의 노드만 발행해야 합니다.**<br>
EKF 도입 후에는 **EKF가 이 TF를 발행**하고, `base_controller`는 `/odom` 토픽만 발행(TF 발행 끔)합니다.
`base_controller`에 `publish_tf:=false` 파라미터를 주면 됩니다.

```
[EKF 도입 후 — 입력 2개가 EKF로 모이고, EKF가 두 가지를 별도로 출력]

  base_controller ──(/odom, TF 발행 안 함)────┐
                                            ├──▶ EKF ──┬──▶ /odometry/filtered  (융합 오도메트리 토픽)
  imu_node ────────(/imu/data_raw)──────────┘          └──▶ odom → base_footprint (TF)
```

> EKF의 출력은 **두 개**입니다. `/odometry/filtered`(토픽)와 `odom → base_footprint`(TF)는 같은 융합 결과를
> 서로 다른 형식으로 표현한 것으로, 하나에 다른 하나가 포함되는 관계가 아닙니다.
> - **TF (`odom → base_footprint`)**: 좌표계 관계가 필요한 노드가 사용 — RViz(로봇 그리기), Nav2(AMCL·costmap이 위치 조회). **우리 시스템의 주 사용처.**
> - **토픽 (`/odometry/filtered`)**: 같은 정보를 데이터 메시지로 — 검증·디버깅(360° 회전 비교 시 `echo`로 확인), 향후 GPS 등 2차 융합 입력용.

### F-5. EKF 실행 및 검증

아래 세 명령은 **각각 별도의 [Pi] 터미널**에서 실행합니다 (매 터미널마다 `source` 먼저).

```bash
# [Pi] 터미널 1 — base_controller (TF 발행 끔!)
ros2 run my_robot base_controller.py --ros-args -p port:=/dev/arduino -p publish_tf:=false

# [Pi] 터미널 2 — IMU
ros2 run my_robot imu_node.py

# [Pi] 터미널 3 — EKF
ros2 run robot_localization ekf_node --ros-args \
    --params-file $(ros2 pkg prefix my_robot)/share/my_robot/config/ekf.yaml
```

`ekf.yaml` 핵심: `two_d_mode: true`(지상 로봇), `base_link_frame: base_footprint`(base_controller와 일치),
바퀴에서 **vx(전진속도)만**, IMU에서 **vyaw(자이로 z)만** 융합.

**TF 트리 확인** — `odom → base_footprint`를 EKF 하나만 발행하는지:
```bash
ros2 run tf2_tools view_frames    # odom 프레임이 30Hz(=EKF frequency)면 정상
```

### F-6. 정량 검증 — 제자리 360° 회전

바닥에 시작 방향을 표시하고, 조이스틱으로 정확히 한 바퀴 회전시킨 뒤 `/odom`과 `/odometry/filtered`의
회전각을 비교합니다.

| | 실제 | 측정 오차 |
|---|---|---|
| 바퀴만 (`/odom`) | 0° (제자리 복귀) | **약 27.8° 오차** |
| 융합 (`/odometry/filtered`) | 0° (제자리 복귀) | **약 1.4° 오차** |

**→ IMU 융합으로 회전 오차 약 20배 개선.** RViz에서 두 Odometry를 겹쳐 보면
바퀴만(빨강)은 회전 시 방향이 사방으로 벌어지고, 융합(초록/흰색)은 실제에 가깝게 모입니다.

> RViz에서 `/odom`(BEST_EFFORT)은 Reliability를 **Best Effort**로, `/odometry/filtered`(RELIABLE)는 **Reliable**로
> 맞춰야 화살표가 보입니다.

---

## 10. G. 실물 — 통합 Launch 파일 & RViz 설정

여기까지 각 노드를 개별 터미널로 실행하며 검증했습니다. 이제 이를 **하나의 launch 파일로 묶어**
매번 여러 터미널을 여는 수고를 없앱니다.

### G-1. 통합 Launch 파일 구조

공통 부분(하드웨어 + IMU + EKF + 조이스틱)을 `bringup_real.launch.py`로 묶고,
그 위에 SLAM 또는 Nav2를 얹는 계층 구조입니다. **`publish_tf:=false`와 포트 기본값(`/dev/arduino`,
`/dev/ydlidar`)이 이미 반영**되어 있어, D단계(udev)만 되어 있으면 한 줄로 실행됩니다.

```
bringup_real.launch.py  = robot_state + base_controller(publish_tf=false) + imu + ekf + ydlidar + joy_teleop
mapping.launch.py       = bringup_real + slam_toolbox
navigation.launch.py    = bringup_real + nav2 (map 기본값: maps/real/my_real_map.yaml)
```

- 실제 실행은 다음 H(SLAM)·I(Nav2) 단계에서 `ros2 launch my_robot mapping.launch.py` 처럼 한 줄로 합니다.

### G-2. RViz 설정 저장

RViz 디스플레이 구성(색상·QoS 포함)을 `.rviz` 파일로 저장해두면 매번 수동 설정이 필요 없습니다.
RViz는 **[PC]** 에서 실행합니다. 이 저장소에는 `rviz/mapping.rviz`, `rviz/nav.rviz`가 포함되어 있습니다.

```bash
# [PC] 저장소에 포함된 설정으로 바로 실행:
rviz2 -d $(ros2 pkg prefix my_robot)/share/my_robot/rviz/nav.rviz

# 직접 구성을 바꾸고 싶으면 RViz에서 원하는 대로 설정 후
#   File → Save Config As → ~/robot_ws/src/my_robot/rviz/nav.rviz 로 저장
#   (저장 후 [PC]에서 colcon build 해야 install 공간에 반영됨)
```
- `$(ros2 pkg prefix ...)`를 쓰므로 **어느 폴더에서 실행해도** 경로가 맞춰집니다.
- `/map` 표시가 안 되면 Map 디스플레이의 **Durability Policy를 `Transient Local`**로 (map은 latched 발행)
- `/map` Color Scheme을 **`map`**으로 하면 흰색(빈 공간)/검정(벽)으로 깔끔하게 표시

---

## 11. H. 실물 — 지도 그리기 (SLAM)

> 시작 전 [Pi]에서 `cd ~/robot_ws && source install/setup.bash`를 실행하세요.
> D단계(udev 포트 고정)가 되어 있어야 launch 기본값(`/dev/arduino`, `/dev/ydlidar`)이 동작합니다.

**[Pi] — 한 줄로 실행 (하드웨어 + IMU + EKF + 조이스틱 + SLAM)**
```bash
ros2 launch my_robot mapping.launch.py
```
- IMU 융합 오도메트리로 지도를 그리므로 회전 구간이 더 정확합니다.
- 조이스틱 LB를 누른 채 스틱으로 로봇을 몰아 맵을 채웁니다. 천천히 이동·회전할수록 지도가 선명합니다.

**[PC] — RViz로 확인** (새 터미널에서 `source` 후)
```bash
rviz2 -d $(ros2 pkg prefix my_robot)/share/my_robot/rviz/mapping.rviz
```

**[Pi] 지도 저장** (새 터미널에서 `source` 후)
```bash
mkdir -p ~/robot_ws/src/maps/real
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/src/maps/real/my_real_map
```
- 절대경로로 저장하므로 어느 폴더에서 실행해도 `~/robot_ws/src/maps/real/`에 저장됩니다.

---

## 12. I. 실물 — Nav2 자율주행

> 시작 전 [Pi]에서 `cd ~/robot_ws && source install/setup.bash`를 실행하세요.

**[Pi] — 한 줄로 실행 (하드웨어 + IMU + EKF + 조이스틱 + Nav2)**
```bash
ros2 launch my_robot navigation.launch.py
# 다른 지도를 쓰려면: ros2 launch my_robot navigation.launch.py map:=/절대경로/다른맵.yaml
```
- 지도 기본값은 `~/robot_ws/src/maps/real/my_real_map.yaml`입니다 (G 단계에서 저장한 지도).

**[PC] — RViz에서 목표 지점 클릭** (새 터미널에서 `source` 후)
```bash
rviz2 -d $(ros2 pkg prefix my_robot)/share/my_robot/rviz/nav.rviz
```
- 로봇 실제 위치와 AMCL 초기 포즈가 다르면 `2D Pose Estimate`로 먼저 보정하세요.
- `2D Goal Pose`로 목표 지점 클릭 → 자율주행 시작
- 라이다 스캔(빨강)이 지도의 벽(검정)에 딱 붙으면 위치추정이 정확한 것입니다.

### 자율주행 중 조이스틱 수동 개입 (twist_mux + velocity_smoother 배선)

자율주행 중에도 LB를 누르면 즉시 수동 조종으로 전환되고, 놓으면 자율주행으로 복귀합니다.
이를 위해 Nav2 명령과 조이스틱 명령을 `twist_mux`로 통합합니다.

**핵심 문제:** Nav2의 `velocity_smoother` 노드가 최종 속도를 **`/cmd_vel`에 직접 발행**하는데,
이 출력 토픽은 파라미터로 바꿀 수 없어 `SetRemap`으로도 우회됩니다. 그래서 twist_mux 출력을
`/cmd_vel`로 두면 velocity_smoother와 충돌해 "가다 멈추다"가 반복됩니다.

**해결:** twist_mux의 최종 출력을 `/cmd_vel`이 아닌 **`/cmd_vel_out`** 으로 분리하고,
velocity_smoother가 내는 `/cmd_vel`을 twist_mux의 navigation 입력으로 받습니다.

```
Nav2 controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel ─┐ (navigation, priority 10)
                                                                      ├─ twist_mux → /cmd_vel_out → base_controller
조이스틱 teleop ─────────────────────────────────→ /cmd_vel_joy ────────┘ (joystick, priority 100)
```

이 배선을 위한 설정:
- `twist_mux.yaml` — navigation 소스의 topic을 `/cmd_vel`로 (velocity_smoother 출력을 받음)
- `joy_teleop.launch.py` — twist_mux 출력 리맵을 제거해 기본값 `/cmd_vel_out` 사용
- `bringup_real.launch.py` — base_controller가 `/cmd_vel_out`을 구독하도록 리맵
- `navigation.launch.py` — Nav2는 기본 구성 그대로 (SetRemap 불필요)

**검증:** 각 토픽의 발행자/구독자가 정확히 연결됐는지 확인합니다.
```bash
ros2 topic info /cmd_vel_out --verbose
#   Publisher: twist_mux 1개 / Subscription: base_controller 1개  → 최종 명령 경로 정상
```
- `/cmd_vel`에는 velocity_smoother와 behavior_server(복구 동작용)가 발행자로 등록되지만,
  둘은 동시에 발행하지 않으므로(정상 주행=velocity_smoother, 복구 상황=behavior_server) 충돌하지 않습니다.
- 최종적으로 base_controller가 받는 `/cmd_vel_out`은 발행자가 twist_mux 하나뿐이라 명령이 깔끔합니다.

---

## 13. 하드웨어 스펙 & 파라미터

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
| IMU | MPU6050 (I2C 0x68), DLPF 44Hz, 자이로 Z만 EKF 융합 |
| IMU 장착 위치 | base_link 기준 `x=0.025, y=-0.055, z=0.0975 m` |
| EKF | `robot_localization`, two_d_mode, odom→base_footprint TF 발행 |
| 360° 회전 오차 | 바퀴만 27.8° → 융합 1.4° (약 20배 개선) |
| USB 포트 고정 | udev: YDLIDAR(10c4:ea60)→`/dev/ydlidar`, Arduino(1a86:7523)→`/dev/arduino` |
| TF 트리 | `map → odom → base_footprint → base_link → …` |
| 명령 배선 | `joy→/cmd_vel_joy`, `nav2→/cmd_vel_nav→velocity_smoother→/cmd_vel` → `twist_mux` → `/cmd_vel_out` → base_controller |

---

## 14. 트러블슈팅

**제어 · 통신**
- **cmd_vel이 실물에서만 최대 0.815초 지연** → 원인은 시리얼이 아니라 무선 환경 RELIABLE QoS의 재전송 큐잉. `/odom` 주기(49.3Hz, 지터 1.6ms)로 시리얼 결백을 데이터로 먼저 확인.
- **Arduino 속도값 이상** → `sscanf(%f)` 대신 `atof` 사용 (AVR에서 `sscanf %f` 오동작).
- **좌/우 바퀴 반대로 도는 느낌** → `EncoderTest_JGB37520`로 먼저 방향 확인 후 배선/핀 교차 수정.
- **자율주행 중 조이스틱으로 개입하면 "가다 멈추다" 반복** → Nav2의 `velocity_smoother`와 twist_mux가 둘 다 `/cmd_vel`에 발행해 충돌. velocity_smoother 출력은 파라미터로 못 바꾸므로, twist_mux 출력을 `/cmd_vel_out`으로 분리하고 velocity_smoother의 `/cmd_vel`을 twist_mux navigation 입력으로 받도록 배선 (I단계 참고). `ros2 topic info /cmd_vel_out --verbose`로 발행자가 twist_mux 하나인지 확인.

**빌드 · 드라이버**
- **colcon build에서 0 packages 인식** → `my_robot/` 폴더 바로 밑에 `package.xml`, `CMakeLists.txt`가 있는지, `my_robot/my_robot/`에 `__init__.py`가 있는지 확인.
- **ydlidar_node 링크 에러** → `/usr/local/lib/libydlidar_sdk.a`가 실제로 존재하는지(`ls /usr/local/lib/ | grep ydlidar`), YDLidar SDK를 먼저 빌드·설치했는지 확인.
- **USB 포트(ttyUSB0/1)가 재부팅마다 뒤바뀜** → udev 규칙(`99-amr-usb.rules`)으로 `/dev/arduino`, `/dev/ydlidar` 고정 (F-1 참고).

**조이스틱**
- **조이스틱이 Keyboard/Mouse로만 잡힘 (`/dev/input/js0` 없음)** → 커널 6.15 미만에서 xpad가 `2dc8:310b`를 모르는 것. `echo "2dc8 310b" > /sys/bus/usb/drivers/xpad/new_id`로 등록 (E-2 참고).
- **`jstest`는 되는데 `joy_node`만 입력이 안 옴** → `joy_node`는 `/dev/input/event*`를 읽으므로 `input` 그룹 권한 필요. `sudo usermod -aG input $USER` 후 재로그인.
- **조이스틱을 꽂았는데 아무 반응 없음** → 8BitDo는 일정 시간 후 자동 절전. 전원 버튼을 눌러 깨운 뒤 launch를 재실행하세요 (joy_node는 시작 시점에 장치를 열기 때문).
- **`/joy`가 1500Hz로 발행됨** → 컨트롤러 폴링레이트가 1000Hz라 정상 동작. `base_controller`가 자체 타이머로 시리얼을 보내므로 제어에는 영향 없음(`/odom` 49Hz 유지 확인). 줄이려면 8BitDo Ultimate Software에서 폴링레이트를 낮추거나 `topic_tools throttle` 사용.

**IMU · EKF · TF**
- **정지 상태인데 로봇이 계속 회전한다고 인식** → 자이로 바이어스 미보정. `imu_node.py`가 시작 시 정지 상태에서 캘리브레이션하므로, 시작 시 로봇을 움직이지 말 것.
- **RViz에서 로봇이 덜덜 떨거나 순간이동** → `odom → base_footprint` TF를 base_controller와 EKF가 동시 발행 중. base_controller에 `publish_tf:=false` 확인.
- **EKF `/odometry/filtered`가 회전은 반영 안 하고 직진만 반영** → IMU 데이터가 EKF에 도달하지 않은 것. imu_node·EKF·base_controller가 같은 기기/네트워크에서 실제로 토픽이 흐르는지 확인.
- **EKF가 아무것도 발행 안 함** → 입력 토픽 이름(`/odom`, `/imu/data_raw`) 확인, `base_link_frame`이 `base_footprint`인지 확인.

**RViz · 시뮬레이션**
- **`/map`이 RViz에 안 보임** → Map 디스플레이 Durability Policy를 `Transient Local`로 (map은 latched 발행).
- **`/map`이 노란색으로만 보임** → Map 디스플레이 Color Scheme을 `map`으로 변경 (costmap용이 기본값).
- **`/odom` Odometry 화살표가 RViz에 안 보임** → base_controller가 BEST_EFFORT로 발행하므로 RViz Reliability를 `Best Effort`로.
- **AMCL 초기 위치 미반영** → `initial_pose_x`가 아니라 `initial_pose.x`(점 표기) 확인.
- **(시뮬) 라이다가 로봇 뒤 빈 공간에 유령 물체 감지** → `base_link`의 collision 박스가 라이다 레이저 높이에 걸리거나, 로봇이 미세하게 기울어 뒤쪽 광선이 캐스터를 훑는 것. collision 높이를 낮추고, 라이다 `range_min`을 0.165로, 캐스터 마찰(`mu`)을 조정해 해결. `/scan`의 `ranges`에서 비정상적으로 짧은 값의 인덱스로 방향을 특정.

---

## License

MIT License