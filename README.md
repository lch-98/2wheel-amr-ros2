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
              + Arduino 펌웨어 (STM32 가능: CANopen)
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
13. [J. 실물 — Arduino Serial → STM32 + CANopen 마이그레이션](#13-j-실물--arduino-serial--stm32--canopen-마이그레이션)
14. [하드웨어 스펙 & 파라미터](#14-하드웨어-스펙--파라미터)
15. [트러블슈팅](#15-트러블슈팅)

---
## 0. 활용 플랫폼, 제어 구조 및 실행 영상

### 활용 플랫폼 (자체 제작)
<img width="966" height="1173" alt="로봇실사진" src="https://github.com/user-attachments/assets/a0a22a08-7569-4268-97eb-6ad8b9c7d071" />

- 메인제어기:   Raspberry pi4 8gb
- 하위제어기:   Arduino Mega 2560 (STM32 가능: CANopen)
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
2wheel-amr-ros2/                        (= ~/robot_ws/src 그 자체)
├── my_robot/                            # ROS2 패키지 (ament_cmake + ament_cmake_python)
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── urdf/
│   │   ├── my_robot.urdf.xacro
│   │   └── my_robot_ros2_control.xacro  # <ros2_control> 태그 (use_ros2_control:=true 일 때)
│   ├── launch/
│   │   ├── gazebo.launch.py               # Gazebo 실행 + 로봇 스폰 (시뮬 전용)
│   │   ├── robot_state.launch.py          # URDF → TF 발행 (실물에서 arduino bringup이 호출 / 시뮬은 gazebo.launch.py에 포함)
│   │   ├── joy_teleop.launch.py           # 조이스틱 teleop (실물 전용)
│   │   ├── robot_control.launch.py        # ros2_control + diff_drive_controller (J단계)
│   │   ├── bringup_real_stm32.launch.py   # 실물 통합 (STM32+CANopen): robot_control + IMU + EKF + 라이다 + 조이스틱
│   │   ├── bringup_real_arduino.launch.py # 실물 통합 (Arduino Serial): robot_state + base_controller + …
│   │   ├── mapping.launch.py              # bringup + SLAM     (base:=stm32|arduino)
│   │   └── navigation.launch.py           # bringup + Nav2     (base:=stm32|arduino)
│   ├── config/
│   │   ├── slam_params.yaml           # 실물 전용 커스텀 SLAM 설정
│   │   ├── my_nav2_params.yaml        # Nav2 설정 (시뮬용)
│   │   ├── my_nav2_params_real.yaml   # Nav2 설정 (실물용)
│   │   ├── joy_teleop.yaml            # 조이스틱 축·버튼 매핑 및 속도 스케일
│   │   ├── twist_mux.yaml             # cmd_vel 소스 우선순위 (joystick 100 / navigation 10 → /cmd_vel_out)
│   │   ├── ekf.yaml                   # robot_localization EKF 설정 (바퀴+IMU 융합)
│   │   └── my_robot_controllers.yaml  # ros2_control 컨트롤러 설정 (J단계)
│   ├── rviz/
│   │   ├── mapping.rviz               # 지도 그리기용 RViz 구성
│   │   └── nav.rviz                   # 자율주행 확인용 RViz 구성 (QoS·색상 저장)
│   ├── my_robot/                      # 파이썬 노드
│   │   ├── base_controller.py         # 모터 구동 + odom 계산 (실물 전용 - arduino 전용)
│   │   ├── imu_node.py                # MPU6050 IMU 드라이버 (실물 전용)
│   │   └── keyboard_teleop.py         # 커스텀 teleop (실물 전용 - arduino 전용)
│   └── src/
│       └── ydlidar_node.cpp           # YDLIDAR 드라이버, 공식 SDK 링크 (실물 전용)
│
├── my_robot_canopen/                  # ROS2 패키지 — CANopen 버스 설정
│   ├── config/my_robot/
│   │   ├── bus.yml                    # 노드·PDO 매핑·채널·단위 환산 정의 (dcfgen 입력)
│   │   ├── my_robot.eds               # STM32 OD 명세 (rww / $NODEID 후처리 완료)
│   │   └── master.dcf, motor.bin      # dcfgen -r 산출물
│   └── launch/
│       └── my_robot_canopen.launch.py # 드라이버 단독 실행 (ros2_control 없이 검증용)
│
├── firmware/                          # 펌웨어 (COLCON_IGNORE — colcon 빌드 대상 아님)
│   ├── EncoderTest_JGB37520/          # [Arduino] 엔코더 방향 확인
│   ├── MotorTest_JGB37520/            # [Arduino] 모터 구동 확인
│   ├── MotorEncoderTest_JGB37520/     # [Arduino] 통합 확인
│   ├── MotorJGB37520_Firmware/        # [Arduino] 실사용 최종 펌웨어 (구버전)
│   └── STM32_CANopen/
│       └── main.c                     # [STM32] CANopenNode + CiA402 2축 펌웨어 + 안전 로직 (현행)
│
├── maps/                              # COLCON_IGNORE — SLAM으로 그린 지도 저장소
│   ├── sim/
│   └── real/
│
└── docs/                              # COLCON_IGNORE — 발표자료, 사진 등
    ├── xpad-8bitdo.service            # 조이스틱 xpad 자동 등록 (systemd)
    └── 99-amr-usb.rules               # USB 포트 고정 udev 규칙
```

**이 저장소 바깥에 따로 두는 것 (J단계 전용)**

```
~/canopen_ws/                          # ros2_canopen 백포트 전용 워크스페이스 (robot_ws와 별개)
└── src/ros2_canopen/                  # lch-98/ros2_canopen 의 humble-multichannel-backport 브랜치
```

> CANopen 드라이버는 upstream [`ros2_canopen`](https://github.com/ros-industrial/ros2_canopen)의 다축 기능(#404)을
> Humble로 백포트한 버전을 씁니다. 이 저장소에 코드를 복사하지 않고 별도 워크스페이스에서 빌드합니다 (2단계·J-6 참고).
> 예전에 직접 작성했던 `my_dual_axis_driver`는 태그
> [`custom-dual-axis-driver`](https://github.com/lch-98/2wheel-amr-ros2/tree/custom-dual-axis-driver)에 남아 있습니다.

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

**J단계(STM32 + CANopen)를 사용하는 경우 추가로 설치하세요.**
Arduino 구성(A~I단계)만 재현한다면 이 블록은 건너뛰어도 됩니다.

```bash
# ros2_control + 차동구동 컨트롤러
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
                 ros-humble-diff-drive-controller ros-humble-ros2controlcli

# CAN 진단 도구 (candump / cansend)
sudo apt install can-utils

# 무선 원격 시각화 시 권장 — Pi와 PC 양쪽에 설치·설정
sudo apt install ros-humble-rmw-cyclonedds-cpp
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
```

> CycloneDDS는 **Pi와 PC 양쪽 모두** 같은 설정이어야 통신됩니다(한쪽만 바꾸면 연결이 끊깁니다).

**확인**
```bash
ros2 control list_controller_types | grep diff_drive
# diff_drive_controller/DiffDriveController 가 보이면 정상
```

#### CANopen 드라이버 (ros2_canopen 다축 백포트) 빌드

`apt`의 `ros-humble-canopen`은 **한 장치에 모터 하나만** 지원합니다. 이 로봇은 STM32 하나(노드 1개)에
바퀴 두 개(채널 2개)가 붙어 있으므로, upstream `master`의 다축 기능(#404)을 Humble로 옮긴
**백포트 버전**을 직접 빌드해 씁니다.

- 백포트 브랜치: [`lch-98/ros2_canopen` · `humble-multichannel-backport`](https://github.com/lch-98/ros2_canopen/tree/humble-multichannel-backport)
- upstream 제안: [ros-industrial/ros2_canopen#449](https://github.com/ros-industrial/ros2_canopen/issues/449)

> 아래는 **[Pi]** 기준입니다. [PC]에서도 빌드를 확인하고 싶다면 같은 순서로 하되, ① 6·7번(라이브러리 등록,
> 권한)은 실물 CAN을 쓰는 [Pi]에서만 필요합니다. ② [PC]는 `--parallel-workers 1`을 빼도 됩니다.

**1. 빌드 도구 준비** (처음 한 번)
```bash
# [Pi] 아무 폴더에서
sudo apt install python3-rosdep python3-colcon-common-extensions
sudo rosdep init        # "already exists" 가 나오면 이미 된 것이므로 무시
rosdep update           # sudo 없이 실행
```

**2. 워크스페이스를 만들고 백포트 브랜치 받기**
```bash
# [Pi]
mkdir -p ~/canopen_ws/src
cd ~/canopen_ws/src
git clone -b humble-multichannel-backport https://github.com/lch-98/ros2_canopen.git
```
`-b humble-multichannel-backport`를 빠뜨리면 Jazzy용 `master`가 받아져 Humble에서 빌드되지 않습니다.

확인:
```bash
cd ~/canopen_ws/src/ros2_canopen
git log --oneline -2
# Reorder constructor member initializer list ...
# Backport CiA 402-2 multi-channel support (#404) to humble   ← 두 줄이 보이면 정상
```

**3. 의존성 설치**
```bash
# [Pi]
cd ~/canopen_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
# 마지막에 "#All required rosdeps installed successfully" 가 나오면 정상
```

**4. 빌드**
```bash
# [Pi]
cd ~/canopen_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to canopen_ros2_control canopen_master_driver \
             --parallel-workers 1 --event-handlers console_cohesion+
```
- 필요한 패키지만 빌드합니다. `canopen_master_driver`를 **반드시 함께** 넣어야 합니다
  (빠뜨리면 마스터만 apt 버전이 로드되어 두 버전이 섞입니다).
- 처음에는 CANopen 라이브러리(lely-core)를 내려받아 컴파일하므로 [Pi]에서 **10분 이상** 걸리고,
  중간에 한동안 멈춘 것처럼 보일 수 있습니다. 끄지 말고 기다리세요.
- 끝에 `Summary: 8 packages finished`가 나오고 `failed`가 없으면 성공입니다 (`stderr output`은 경고라 무시해도 됩니다).

**5. 설치 확인**
```bash
# [Pi]
source ~/canopen_ws/install/setup.bash
ros2 pkg prefix canopen_402_driver
# /home/<사용자>/canopen_ws/install/canopen_402_driver  ← /opt/ros/humble 이면 잘못된 것
```

**6. 라이브러리 경로 등록 (필수, [Pi])**

J-2에서 `ros2_control_node`에 `setcap` 권한을 주는데, 권한이 걸린 프로그램은 `source`로 잡은 경로를 **무시**하고
시스템에 등록된 경로에서만 라이브러리를 찾습니다. 등록하지 않으면 플러그인만 백포트 버전이고 그 안의 본체는
`/opt/ros/humble`(apt) 버전이 로드되어, 채널 기능이 동작하지 않습니다(트러블슈팅 참고).

```bash
# [Pi] 백포트 라이브러리 폴더 목록을 "00-" 로 시작하는 설정 파일로 저장 → 다른 설정보다 먼저 읽힘
ls -d ~/canopen_ws/install/*/lib | sudo tee /etc/ld.so.conf.d/00-canopen-backport.conf
sudo ldconfig
```
확인:
```bash
ldconfig -p | grep -E "libnode_canopen_cia402_driver|libnode_canopen_basic_master"
# 같은 이름이 두 줄씩 나오는데, 각 이름의 **첫 줄**이 /home/<사용자>/canopen_ws/... 이면 정상
```

> **되돌릴 때** (예: apt 버전으로 돌아갈 때) 반드시 이 등록을 먼저 지우세요.
> ```bash
> sudo rm /etc/ld.so.conf.d/00-canopen-backport.conf
> sudo ldconfig
> ```

**7. CAN 권한** — J-2의 "권한 설정"을 그대로 하면 됩니다.
`canopen_ws`를 **다시 빌드할 때마다** `device_container_node`가 새 파일로 바뀌어 권한이 사라지므로 재부여가 필요합니다.

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

> **J단계(STM32 + CANopen)를 쓰는 [Pi]는 source 순서가 다릅니다.**
> `robot_ws`를 빌드하기 **전에** 백포트 워크스페이스를 먼저 source해야, 로봇 패키지가 apt 버전이 아니라
> 백포트 버전 위에 빌드됩니다. 빌드와 실행 모두 아래 순서를 지키세요.
> ```bash
> source /opt/ros/humble/setup.bash      # 1) ROS2 기본
> source ~/canopen_ws/install/setup.bash # 2) ros2_canopen 백포트
> cd ~/robot_ws
> colcon build --symlink-install         # (빌드할 때만)
> source ~/robot_ws/install/setup.bash   # 3) 이 저장소
> ```
> `.bashrc`에도 같은 순서로 넣어 두면 새 터미널마다 입력하지 않아도 됩니다.
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/canopen_ws/install/setup.bash
> source ~/robot_ws/install/setup.bash
> ```

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

공통 부분(하드웨어 + IMU + EKF + 조이스틱)을 `bringup_real.launch.py`(STM32 or Arduino)로 묶고,
그 위에 SLAM 또는 Nav2를 얹는 계층 구조입니다. **`publish_tf:=false`와 포트 기본값(`/dev/arduino`,
`/dev/ydlidar`)이 이미 반영**되어 있어, D단계(udev)만 되어 있으면 한 줄로 실행됩니다.

```
bringup_real.launch.py  = robot_state + base_controller(publish_tf=false) + imu + ekf + ydlidar + joy_teleop
(STM32 or Arduino)
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

> ### 구동 계층 선택 (`base:=`)
>
> `mapping.launch.py`·`navigation.launch.py`는 하부 구동 계층을 인자로 선택합니다.
>
> | 값 | 하부 계층 | 실행되는 bringup |
> |---|---|---|
> | `stm32` (기본값) | STM32 + CANopen (J단계) | `bringup_real_stm32.launch.py` |
> | `arduino` | Arduino Serial (C단계) | `bringup_real_arduino.launch.py` |
>
> ```bash
> ros2 launch my_robot mapping.launch.py                 # STM32 (기본)
> ros2 launch my_robot mapping.launch.py base:=arduino   # Arduino
> ```
>
> 두 bringup은 각자 필요한 노드를 온전히 포함하므로 추가 인자가 필요 없습니다.
> - **Arduino**: `robot_state.launch.py`(`robot_state_publisher` + `joint_state_publisher`) + `base_controller.py`
> - **STM32**: `robot_control.launch.py`(`robot_state_publisher` + `joint_state_broadcaster` + `diff_drive_controller`)
>
> 잘못된 값(예: `base:=foo`)을 주면 두 조건 모두 거짓이 되어 **하드웨어가 아무것도 뜨지 않습니다.**
> 하드웨어 로그가 보이지 않으면 오타를 먼저 의심하세요.

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
- 지도 기본값은 `~/robot_ws/src/maps/real/my_real_map.yaml`입니다 (H 단계에서 저장한 지도).

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

## 13. J. 실물 — Arduino Serial → STM32 + CANopen 마이그레이션

> **이 단계는 A~I가 모두 동작하는 상태에서 시작합니다.** 상위 스택(IMU·EKF·라이다·SLAM·Nav2·조이스틱)은
> 그대로 두고, **하부 구동 계층만** Arduino 시리얼에서 STM32 CAN(CANopen CiA 402)으로 교체합니다.
> `twist_mux → /cmd_vel_out`과 `/odom → EKF` 인터페이스가 동일하게 유지되므로 상위 스택은 수정이 없습니다.

### J-0. 왜 CAN인가

| | Arduino Serial | STM32 + CANopen |
|---|---|---|
| 물리 계층 | USB 시리얼 (1:1) | CAN 버스 (멀티드롭, 최대 127노드) |
| 프로토콜 | 자체 텍스트 (`"v 0.1 0.1\n"`) | **CiA 402 표준** (산업용 모터 드라이브 프로파일) |
| 노이즈 내성 | 낮음 | 차동 신호로 높음 |
| 확장성 | 장치마다 USB 포트 필요 | 버스에 노드 추가만 |
| 진단 | 없음 | NMT 하트비트, EMCY, SDO abort 코드 |

축을 늘릴 때(예: 3축 매니퓰레이터) 배선이 늘지 않고, 표준 프로파일이라 상용 드라이브와도 호환됩니다.

### J-1. 하드웨어 구성 변경

```
[변경 전]  Pi ──USB Serial── Arduino Mega ── MDD10A ── 모터 ×2
[변경 후]  Pi ──USB-CAN(gs_usb)── CAN Bus ── STM32 NUCLEO-F446RE ── MDD10A ── 모터 ×2
```

**추가 부품**
- STM32 NUCLEO-F446RE
- USB-CAN 어댑터 (candleLight, `gs_usb` 드라이버)
- CAN 트랜시버 (TJA1050 등) ×1 (STM32 측)
- 종단 저항 120Ω ×2 (버스 양 끝)

**배선 시 필수 확인**
- CAN_H ↔ CAN_L 저항이 **60Ω**(120Ω 두 개 병렬)일 것
- **모든 장치의 GND 공유** — CAN은 차동 신호지만 공통 기준점(common mode reference)이 필요합니다
- 모터 단자에 0.22µF 세라믹 커패시터 (PWM 노이즈의 전도 결합 차단)

> **GND는 브레드보드 점퍼로 하지 마세요.** 접촉 저항이 조금만 생겨도 bus-off가 폭증합니다.
> 실제로 이 프로젝트에서 GND 접촉 불량 하나로 `bus-off` 카운터가 10만 회를 넘긴 적이 있습니다 (트러블슈팅 참고).

### J-2. CAN 인터페이스 설정

```bash
# [Pi] 인터페이스 확인
ip link show can0

# 500kbps로 up, bus-off 자동 복구 100ms
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000 restart-ms 100

# 확인
ip -details link show can0        # state UP, restart-ms 100
candump can0                      # 하트비트 701 수신 확인
```

> **`restart-ms 100`은 선택이 아니라 필수입니다.** 기본값 0이면 bus-off에 한 번 빠졌을 때
> 수동으로 down/up 하기 전까지 영구 정지합니다. 주행 중 진동으로 접촉이 흔들리면 그대로 제어 불능이 됩니다.
> down/up 할 때마다 초기화되므로 매번 다시 지정해야 합니다.

**권한 설정 (sudo 없이 실행)**

CAN raw 소켓은 `CAP_NET_RAW`가 필요합니다. `netdev` 그룹만으로는 부족하므로 실행 파일에 capability를 부여합니다.

```bash
# [Pi] 아무 폴더에서
# 1) ros2_control 로 실행할 때 CAN 을 여는 프로그램 (apt 설치본)
sudo setcap cap_net_raw,cap_net_admin=eip /opt/ros/humble/lib/controller_manager/ros2_control_node

# 2) 드라이버만 단독 실행할 때 CAN 을 여는 프로그램 (백포트 빌드본)
sudo setcap cap_net_raw,cap_net_admin=eip ~/canopen_ws/install/canopen_core/lib/canopen_core/device_container_node

# 확인 — 둘 다 "cap_net_admin,cap_net_raw=eip" 가 나오면 정상
getcap /opt/ros/humble/lib/controller_manager/ros2_control_node
getcap ~/canopen_ws/install/canopen_core/lib/canopen_core/device_container_node
```

> `ros2_control_node`에도 반드시 부여해야 합니다. ros2_control을 붙이면 CAN 소켓을 여는 주체가
> `device_container_node`가 아니라 `ros2_control_node`로 바뀌기 때문입니다.
>
> **`device_container_node`는 백포트 빌드본의 경로를 써야 합니다.** `/opt/ros/humble/...` 쪽에 권한을 줘도
> 실제로 실행되는 것은 `~/canopen_ws/...` 쪽이라 `CanController: Operation not permitted`로 죽습니다.
> 그리고 `canopen_ws`를 **다시 빌드하면 이 파일이 새로 만들어져 권한이 사라지므로**, 빌드 후 2)를 다시 실행하세요.
>
> setcap이 걸린 프로그램이 백포트 라이브러리를 찾으려면 2단계 6번의 **라이브러리 경로 등록**이 되어 있어야 합니다.

### J-3. STM32 펌웨어 (CANopenNode + CiA 402)

**베이스**: [CANopenNode](https://github.com/CANopenNode/CANopenNode) + [CANopenNode_STM32](https://github.com/CANopenNode/CanOpenSTM32)

**MotorAxis 구조체로 축 추상화** — 좌우(그리고 향후 3축 매니퓰레이터)를 동일 로직으로 처리합니다.

```c
typedef struct {
    TIM_HandleTypeDef* enc_tim;      // 엔코더 타이머 (TIM3 / TIM4)
    TIM_HandleTypeDef* pwm_tim;      // PWM 타이머 (TIM1)
    uint32_t pwm_channel;            // CHANNEL_1 / CHANNEL_2
    GPIO_TypeDef* dir_port;
    uint16_t dir_pin;
    int8_t enc_sign;                 // 오른쪽 -1, 왼쪽 +1
    // OD 포인터 (축1: 0x60xx, 축2: 0x68xx)
    uint16_t* od_controlword;
    uint16_t* od_statusword;
    int8_t*   od_mode;
    int8_t*   od_mode_display;
    int32_t*  od_target;
    int32_t*  od_actual;             // 실제 속도   (0x606C / 0x686C)
    int32_t*  od_position;           // 실제 위치   (0x6064 / 0x6864)
    // 제어 상태
    int32_t  position_cnt;           // 누적 엔코더 카운트
    float integral, prev_error, vel_filtered;
} MotorAxis;
```

**위치 피드백** — 매 제어 주기마다 엔코더 변화량을 누적해 `0x6064`/`0x6864`에 씁니다.

```c
// controlLoopAxis() 안, 실제 속도를 쓴 직후
ax->position_cnt += delta;              // delta 에는 이미 enc_sign 이 적용됨 → 속도와 부호 일치
*ax->od_position = ax->position_cnt;    // 단위: 엔코더 카운트 (1바퀴 = 1320)
```

> upstream `Cia402Driver`는 `joint_states`의 위치를 `0x6064`에서 직접 읽습니다. 이 코드가 없으면
> 바퀴를 아무리 돌려도 위치가 0으로 나옵니다 (EDS에 오브젝트가 있는 것과, 펌웨어가 값을 써 넣는 것은 별개입니다).
> 라디안 변환은 펌웨어가 아니라 `bus.yml`의 `scale_pos_from_dev`에서 합니다 (J-5).

**축별 OD 매핑** (CiA 402 다축 규칙: 축 N은 `+0x800 × (N-1)`)

| 객체 | 축1 (오른쪽) | 축2 (왼쪽) |
|---|---|---|
| Controlword | `0x6040` | `0x6840` |
| Statusword | `0x6041` | `0x6841` |
| Modes of Operation | `0x6060` | `0x6860` |
| Mode Display | `0x6061` | `0x6861` |
| Position Actual | `0x6064` | `0x6864` |
| Velocity Actual | `0x606C` | `0x686C` |
| Target Velocity | `0x60FF` | `0x68FF` |
| Supported drive modes | `0x6502` | `0x6D02` |

> 축2의 주소가 정확히 `+0x800`인 것이 중요합니다. upstream 드라이버는 채널 N의 주소를
> `기본주소 + N × 0x800`으로 계산합니다 (CiA 402-2 규칙). 그래서 펌웨어를 바꾸지 않고도 그대로 맞아떨어집니다.

**CiA 402 상태머신** (방식 A — 최소 구현)

```
Switch On Disabled (SW 0x0040)
   ↓ CW 0x0006 (Shutdown)
Ready to Switch On (SW 0x0021)
   ↓ CW 0x0007 (Switch On)
Switched On (SW 0x0023)
   ↓ CW 0x000F (Enable Operation)
Operation Enabled (SW 0x0027)   ← 이 상태에서만 모터에 전류가 걸립니다
```

Statusword 비교 시 **하위 비트만 마스킹**(`& 0x006F`)해야 합니다. bit10(Target Reached) 등 상위 비트는
상태와 무관하게 변하므로, 마스킹 없이 `== 0x0027`로 비교하면 오작동합니다.

**제어 루프** (20ms / 50Hz, Arduino 펌웨어에서 그대로 이식)

```
목표속도(0x60FF) → [P I D + 피드포워드] → PWM → 모터
엔코더 → [저역통과 필터] → 실제속도(0x606C)
```

- `Kp=150, Ki=300, Kd=0`, 피드포워드 `PWM = 222.7 × v + 3.4`
- 적분 windup 방지: `int_limit = INTEGRAL_MAX / KI`로 클램핑
- OD는 정수형이므로 속도는 **mm/s** 정수 (0.1 m/s ↔ 100), 위치는 **엔코더 카운트**로 주고받습니다

**PWM 타이머 설정 (중요)**

```c
htim1.Init.Prescaler = 87;      // 84MHz / 88 / 256 ≈ 3.7kHz
htim1.Init.Period    = 255;     // PWM 분해능 0~255 (Arduino analogWrite와 동일)
```

> **MDD10A 권장 상한이 20kHz입니다.** Prescaler=15이면 20.5kHz로 스펙을 살짝 넘겨,
> **저 duty(저속)에서만** 모터가 끊기는 현상이 발생합니다. 여유 있게 3.7kHz로 설정하세요 (트러블슈팅 참고).

**부팅 시 자동 OP 전환 제거**

`CO_app_STM32.c`에서 `CO_NMT_STARTUP_TO_OPERATIONAL`을 제거해, 슬레이브가 스스로 Operational로 가지 않고
**마스터의 NMT Start 명령을 기다리도록** 합니다. 마스터가 PDO 매핑을 설정한 뒤 전환시키는 것이 정상 순서입니다.

### J-4. EDS / Object Dictionary 작성

**도구**: [CANopenEditor](https://github.com/CANopenNode/CANopenEditor) (v4.2.3)

`.xpd` 프로젝트를 원본으로 두고, 여기서 두 가지를 export 합니다.

```
CANopenEditor (.xpd)
   ├── Export EDS         → Pi 측 (dcfgen 입력)
   └── Export CANopenNode → STM32 측 (OD.c / OD.h)
```

**필수 객체 체크리스트**

| 인덱스 | 이름 | AccessType | PDO Mapping | 비고 |
|---|---|---|---|---|
| `0x6040/0x6840` | Controlword | **`rww`** | RPDO | `rw`면 lely 검증 실패 |
| `0x6041/0x6841` | Statusword | `ro` | **TPDO(`tr`)** | |
| `0x6060/0x6860` | Modes of Operation | **`rww`** | RPDO | |
| `0x6061/0x6861` | Mode Display | `ro` | **TPDO(`tr`)** | |
| `0x606C/0x686C` | Velocity Actual | `ro` | **TPDO(`tr`)** | |
| `0x60FF/0x68FF` | Target Velocity | **`rww`** | RPDO | |
| `0x6064/0x6864` | Position Actual | `ro` | **TPDO(`tr`)** | 펌웨어가 값을 써야 함 (J-3) |
| **`0x6502/0x6D02`** | **Supported drive modes** | `ro` | No | **누락 시 SIGSEGV** |

**`0x6502`는 반드시 넣으세요.** `UNSIGNED32`, DefaultValue `0x00000004`(bit2 = pv 모드).
누락하면 드라이버가 부팅 중 SDO abort `0x06020000`을 받고 **세그멘테이션 폴트로 죽습니다.**

> **실제로 구현한 모드만 켜야 합니다.** EDS에 `0x607A`(Target Position)가 있다고 해서 pp 모드를 켜면
> (`0x05`), 드라이버가 위치 모드로 전환을 시도했다가 조용히 실패합니다. 펌웨어에 위치 제어 루프가
> 없으면 사용하지 마세요.

**EDS 후처리 (Pi 측, 매번 export 후 반복 필요)**

CANopenEditor는 `rww`를 생성하지 못하고 `$NODEID` 표현식을 그대로 남기므로 직접 수정합니다.

```bash
cd ~/robot_ws/src/my_robot_canopen/config/my_robot
cp my_robot.eds my_robot.eds.bak

# 1) RPDO 대상 객체의 AccessType을 rww로
python3 - << 'EOF'
import re
path = "my_robot.eds"
content = open(path, encoding='utf-8', errors='ignore').read()
for idx in ['6040', '6060', '60FF', '6840', '6860', '68FF']:
    content = re.sub(r'(\[' + idx + r'\][^\[]*?AccessType=)rw(\s)', r'\1rww\2',
                     content, flags=re.IGNORECASE)
open(path, 'w', encoding='utf-8').write(content)
EOF

# 2) $NODEID 표현식을 고정값으로
sed -i 's/DefaultValue=0x80+\$NODEID/DefaultValue=0x81/' my_robot.eds
sed -i 's/DefaultValue=0x600+\$NODEID/DefaultValue=0x601/' my_robot.eds
sed -i 's/DefaultValue=0x580+\$NODEID/DefaultValue=0x581/' my_robot.eds
sed -i '/^NG_Slave/d' my_robot.eds

# 3) 확인
for i in 6040 60FF 6840 68FF; do grep -A6 "^\[$i\]" my_robot.eds | grep AccessType; done
grep -n 'NODEID' my_robot.eds     # 아무것도 안 나와야 정상
```

**AccessType 규칙**

| 값 | 의미 |
|---|---|
| `rw` | 일반 읽기/쓰기 (PDO 매핑 불가) |
| **`rww`** | **RPDO 방향** (마스터 → 슬레이브) |
| `rwr` | TPDO 방향 (슬레이브 → 마스터) |
| `ro` | 읽기 전용 |

### J-5. bus.yml 작성 및 DCF 생성

`~/robot_ws/src/my_robot_canopen/config/my_robot/bus.yml`:

```yaml
options:
  dcf_path: "/home/lch/robot_ws/install/my_robot_canopen/share/my_robot_canopen/config/my_robot"

master:
  node_id: 2
  driver: "ros2_canopen::MasterDriver"
  package: "canopen_master_driver"
  baudrate: 500
  sync_period: 20000              # 마이크로초 = 20ms
  heartbeat_producer: 100         # 마스터가 100ms마다 하트비트 송신
  heartbeat_multiplier: 3         # 슬레이브는 300ms 동안 못 받으면 고장 처리

motor:
  node_id: 1
  boot_timeout_ms: 5000           # 기본값(~20ms)은 너무 짧아 반드시 늘려야 함
  dcf: "my_robot.eds"
  driver: "ros2_canopen::Cia402Driver"       # upstream 드라이버 (백포트 버전, J-6)
  package: "canopen_402_driver"
  period: 20
  heartbeat_consumer: true
  heartbeat_producer: 100
  heartbeat_multiplier: 3

  # ── 다축(CiA 402-2) 설정 ──
  num_channels: 2                                # 채널 0 = 0x60xx(오른쪽), 채널 1 = 0x68xx(왼쪽)
  channel_names: ["right_wheel", "left_wheel"]   # 서비스·joint 이름에 쓰임 (반드시 문자로 시작)

  # ── 단위 환산 (펌웨어: mm/s·카운트  ↔  ROS: rad/s·rad) ──
  scale_vel_to_dev: 32.5            # rad/s → mm/s   = 바퀴 반지름 0.0325 m × 1000
  scale_vel_from_dev: 0.03076923    # mm/s  → rad/s  = 1 / 32.5
  scale_pos_to_dev: 210.08452       # rad   → count  = 1320 / (2π)
  scale_pos_from_dev: 0.00475999    # count → rad    = (2π) / 1320
  rpdo:
    1: { enabled: true, cob_id: "auto",
         mapping: [{index: 0x6040, sub_index: 0}, {index: 0x6060, sub_index: 0}] }
    2: { enabled: true, cob_id: "auto",
         mapping: [{index: 0x60FF, sub_index: 0}] }
    3: { enabled: true, cob_id: "auto",
         mapping: [{index: 0x6840, sub_index: 0}, {index: 0x6860, sub_index: 0}] }
    4: { enabled: true, cob_id: "auto",
         mapping: [{index: 0x68FF, sub_index: 0}] }
  tpdo:
    1: { enabled: true, cob_id: "auto", transmission: 0x01,
         mapping: [{index: 0x6041, sub_index: 0}, {index: 0x6061, sub_index: 0}] }
    2: { enabled: true, cob_id: "auto", transmission: 0x01,
         mapping: [{index: 0x606C, sub_index: 0}, {index: 0x6064, sub_index: 0}] }
    3: { enabled: true, cob_id: "auto", transmission: 0x01,
         mapping: [{index: 0x6841, sub_index: 0}, {index: 0x6861, sub_index: 0}] }
    4: { enabled: true, cob_id: "auto", transmission: 0x01,
         mapping: [{index: 0x686C, sub_index: 0}, {index: 0x6864, sub_index: 0}] }
```

**설정 항목 설명**

| 키 | 의미 | 빠뜨리면 |
|---|---|---|
| `num_channels` | 이 장치(노드 1)에 붙은 축 수 | 1로 간주 → 오른쪽 바퀴만 동작 |
| `channel_names` | 채널 이름. 서비스는 `/<이름>/init`, joint_states 이름도 이것 | 기본 이름이 숫자로 시작해 **드라이버가 크래시** (upstream #427에서 수정 중) |
| `scale_vel_to_dev` | ROS 속도(rad/s) → 펌웨어 값(mm/s) 곱셈 계수 | 기본 1000 → 바퀴가 약 31배 빠르게 회전 |
| `scale_vel_from_dev` | 펌웨어 값(mm/s) → ROS 속도(rad/s) | 오도메트리가 틀어짐 |
| `scale_pos_from_dev` | 엔코더 카운트 → rad | joint_states 위치가 틀어짐 |

> **단위 환산을 `bus.yml`에서 하는 이유**: `diff_drive_controller`는 바퀴 조인트에 **rad/s**를 주고받습니다.
> 예전 커스텀 드라이버는 이 변환을 C++ 코드에서 `wheel_radius`로 했지만, upstream 드라이버는
> 설정값(`scale_*`)으로 처리합니다. 그래서 URDF에는 더 이상 `wheel_radius` 파라미터가 없습니다.
>
> 검산: 0.1 m/s 직진 → 바퀴 0.1 / 0.0325 = 3.077 rad/s → × 32.5 = **100 mm/s** (RPDO `64 00 00 00`).

**DCF 생성**

`bus.yml`의 **PDO 매핑**을 바꿨을 때만 다시 생성합니다. 채널·단위 환산만 바꿨다면 이 단계는 건너뛰고
아래 "install에 반영"만 하면 됩니다.

```bash
# [Pi]
cd ~/robot_ws/src/my_robot_canopen/config/my_robot
dcfgen -d . -r -v bus.yml        # -r 플래그 필수 (원격 PDO 매핑 생성)
```

> **`-r` 없이 생성하면** 마스터가 슬레이브의 PDO 매핑을 설정하지 못해 통신이 성립하지 않습니다.

**install에 반영 (bus.yml을 고칠 때마다 필수)**

런치 파일은 `src`가 아니라 **`install/.../share/...` 안의 사본**을 읽습니다. `src`만 고치고 빌드하지 않으면
예전 설정으로 실행됩니다 (실제로 TPDO를 추가했는데 반영이 안 된 적이 있습니다).

```bash
# [Pi]
source /opt/ros/humble/setup.bash
source ~/canopen_ws/install/setup.bash
cd ~/robot_ws
colcon build --packages-select my_robot_canopen

# 반영 확인 — 아무것도 출력되지 않으면 src 와 install 이 같은 것
diff src/my_robot_canopen/config/my_robot/bus.yml \
     install/my_robot_canopen/share/my_robot_canopen/config/my_robot/bus.yml
cmp  src/my_robot_canopen/config/my_robot/motor.bin \
     install/my_robot_canopen/share/my_robot_canopen/config/my_robot/motor.bin
```

**COB-ID 자동 할당 결과** (`cob_id: "auto"` → 표준 Pre-defined Connection Set)

| PDO | 공식 | 노드1 | 내용 |
|---|---|---|---|
| RPDO1 | `0x200 + ID` | `0x201` | 축1 Controlword + Mode |
| RPDO2 | `0x300 + ID` | `0x301` | 축1 Target Velocity |
| RPDO3 | `0x400 + ID` | `0x401` | 축2 Controlword + Mode |
| RPDO4 | `0x500 + ID` | `0x501` | 축2 Target Velocity |
| TPDO1 | `0x180 + ID` | `0x181` | 축1 Statusword + Mode Display |
| TPDO2 | `0x280 + ID` | `0x281` | 축1 Velocity Actual + Position Actual (8바이트) |
| TPDO3 | `0x380 + ID` | `0x381` | 축2 Statusword + Mode Display |
| TPDO4 | `0x480 + ID` | `0x481` | 축2 Velocity Actual + Position Actual (8바이트) |

`transmission: 0x01`은 "SYNC마다 전송"입니다. TPDO에만 지정하며, RPDO는 **값이 바뀔 때만**(이벤트 기반)
전송되므로 candump에서 명령 시점에만 나타나는 것이 정상입니다.

### J-6. 다축 드라이버 — 직접 구현에서 upstream 백포트로

**문제**: `apt`로 설치되는 `ros2_canopen` Humble 0.2.13의 `Cia402Driver`는 **1축만** 지원합니다.

```cpp
// canopen_402_driver/motor.hpp (Humble 0.2.13)
typedef ModeForwardHelper<MotorBase::Profiled_Velocity, int32_t, 0x60FF, 0, 0> ProfiledVelocityMode;
//                                                              ↑ 컴파일 타임 상수
```

`0x60FF`가 템플릿 인자로 고정돼 있어서, 같은 노드의 축2(`0x68FF`)를 제어할 방법이 없었습니다.

#### 1차 해결: 커스텀 드라이버 직접 구현 (`my_dual_axis_driver`)

`LelyDriverBridge::universal_set_value(index, ...)`가 인덱스를 **런타임 인자**로 받는다는 점을 이용해,
`NodeCanopenProxyDriver`를 상속한 별도 드라이버를 작성했습니다. 축마다 기준 주소(`0x6000`/`0x6800`)를 두고
`기준 + 오프셋`으로 주소를 계산하는 방식이었고, ros2_control용 `DualAxisSystem`까지 만들어 실제 SLAM 주행에 사용했습니다.

다만 한계가 분명했습니다.
- CiA 402 상태머신을 upstream과 **별도로 다시 구현**한 평행 코드였습니다.
- 축 수가 코드에 2로 고정돼 있었습니다.
- 위치 피드백이 구현되지 않아 항상 0이었습니다.
- Fault 이후 복구하려면 런치를 재시작해야 했습니다.

전체 코드와 당시 설명은 태그 [`custom-dual-axis-driver`](https://github.com/lch-98/2wheel-amr-ros2/tree/custom-dual-axis-driver)에 남아 있습니다.

#### 2차 해결: upstream 다축 기능을 Humble로 백포트

upstream 기여를 준비하며 조사해 보니, `ros2_canopen` **`master` 브랜치에 같은 기능이 이미 머지돼 있었습니다**
([#404](https://github.com/ros-industrial/ros2_canopen/pull/404), 2026-03-13). 설계도 거의 같았습니다.
채널 N의 주소를 `기본주소 + N × 0x800`으로 계산하고, `bus.yml`에 `num_channels`, URDF에 `channel`을 두는 방식입니다.
다만 `master`는 Jazzy 이후 배포판용이고, `humble` 브랜치에는 들어가지 않은 상태였습니다.

그래서 직접 만든 드라이버를 유지하는 대신, **#404를 `humble` 브랜치로 옮겨(백포트) 표준 드라이버로 이전**했습니다.

| 항목 | 커스텀 드라이버 | upstream 백포트 |
|---|---|---|
| CiA 402 로직 | 별도 재구현 | upstream `Motor402` 그대로 사용 |
| 축 수 | 코드에 2로 고정 | `bus.yml`의 `num_channels` |
| 위치 피드백 | 항상 0 | `0x6064` 누적 카운트 → rad |
| 단위 환산 | C++ 코드 (`wheel_radius`) | `bus.yml`의 `scale_*` |
| Fault 복구 | 런치 재시작 | `recover` 서비스 |
| 시작 시 모터 활성화 | `on_activate()`에서 자동 | **서비스 호출 필요 (J-7)** |
| 배포판 이전 (Jazzy 등) | 드라이버 재작성 | `bus.yml`·URDF를 거의 그대로 사용 |

**백포트에서 바뀐 것** — 드라이버가 3개 층으로 나뉘며, 층마다 "채널"이 추가됐습니다.

```
ros2_control (Cia402System)        URDF joint 의 <param name="channel"> 을 읽어
        │                          (노드 ID, 채널) 단위로 명령·상태를 주고받음
Cia402Driver (드라이버 노드)        Motor402 를 채널 수만큼 만들고,
        │                          서비스·joint_states·단위 환산을 채널별로 관리
Motor402 (모터 하나의 CiA 402)      모든 오브젝트 주소에 channel × 0x800 을 더함
        │                          (Controlword, Statusword, 모드, 0x6502, 0x6064, 0x606C, 목표값 …)
STM32  (0x60xx = 채널 0, 0x68xx = 채널 1)
```

- **하위 호환**: 채널을 지정하지 않으면 0번으로 동작합니다. 서비스 이름(`/motor/init`)과 joint 이름도 예전과 같습니다.
- **master와 다르게 한 부분**: effort(`0x6077`, 별도 기능 #316)는 humble에 없어서 뺐고, ros2_control의
  `on_init()`은 humble API를 유지했습니다. 나머지는 #404와 같습니다.
- **코드**: [`lch-98/ros2_canopen` · `humble-multichannel-backport`](https://github.com/lch-98/ros2_canopen/tree/humble-multichannel-backport)
  (upstream `humble` 위에 커밋 2개)
- **upstream 제안**: [ros-industrial/ros2_canopen#449](https://github.com/ros-industrial/ros2_canopen/issues/449)
  — 실기 2채널·단일 채널 시험 결과를 첨부해 humble 반영 여부를 문의했습니다.

**드라이버 단독 검증** (ros2_control 없이, 바퀴를 공중에 띄운 상태)

드라이버만 띄워서 CAN 통신과 채널 동작을 먼저 확인하는 방법입니다. 터미널을 3개 씁니다.
**모든 터미널에서 먼저 아래 3줄을 실행**하세요 (3단계 "source 순서" 참고).

```bash
source /opt/ros/humble/setup.bash
source ~/canopen_ws/install/setup.bash
source ~/robot_ws/install/setup.bash
```

```bash
# [Pi] 터미널 1 — 드라이버 실행
ros2 launch my_robot_canopen my_robot_canopen.launch.py
# 로그에 "num_channels: 2" 가 보이면 2채널 설정이 적용된 것
```

```bash
# [Pi] 터미널 2 — CAN 모니터
candump can0 | grep -E " 181 | 281 | 301 | 381 | 481 | 501 "
```

```bash
# [Pi] 터미널 3 — 서비스 이름 확인
ros2 service list | grep -E "/init|velocity_mode|/target|/recover"
#   /right_wheel/init, /right_wheel/velocity_mode, /right_wheel/target, /right_wheel/recover
#   /left_wheel/...  가 보여야 함.  /motor/init 이 보이면 15. 트러블슈팅의 "STM32 · CANopen" 참고

# 오른쪽 바퀴: 활성화 → Profile Velocity 모드 → 목표속도 3.0769 rad/s (= 0.1 m/s)
ros2 service call /right_wheel/init std_srvs/srv/Trigger
ros2 service call /right_wheel/velocity_mode std_srvs/srv/Trigger
ros2 service call /right_wheel/target canopen_interfaces/srv/COTargetDouble "{target: 3.0769}"

# 왼쪽 바퀴도 같은 순서
ros2 service call /left_wheel/init std_srvs/srv/Trigger
ros2 service call /left_wheel/velocity_mode std_srvs/srv/Trigger
ros2 service call /left_wheel/target canopen_interfaces/srv/COTargetDouble "{target: 3.0769}"

# 정지
ros2 service call /right_wheel/target canopen_interfaces/srv/COTargetDouble "{target: 0.0}"
ros2 service call /left_wheel/target canopen_interfaces/srv/COTargetDouble "{target: 0.0}"
```

터미널 2에서 확인할 것:

| 프레임 | 정상 값 | 의미 |
|---|---|---|
| `181` / `381` | `27 00 03` | Operation Enabled(`0x0027`) + Profile Velocity(`03`) |
| `301` / `501` | `64 00 00 00` | 목표 100 mm/s (단위 환산 정상) |
| `281` / `481` | `[8]` 앞 4바이트 ≈ `64 00 00 00`, 뒤 4바이트 계속 증가 | 실제 속도 + 위치 |

> `target` 값은 **rad/s**입니다. `bus.yml`의 `scale_vel_to_dev`(32.5)를 곱해 mm/s로 바뀌어 전송됩니다.
> 예전 커스텀 드라이버는 m/s(`0.1`)를 받았으므로 값을 그대로 옮겨 쓰면 안 됩니다.

### J-7. ros2_control 통합 (Cia402System)

`diff_drive_controller`를 쓰려면 `hardware_interface::SystemInterface` 구현이 필요합니다.
백포트한 `canopen_ros2_control`의 **`Cia402System`** 을 그대로 씁니다. 직접 작성하는 코드는 없고, URDF 설정만 합니다.

| | 기존 `Cia402System` (apt) | 백포트 `Cia402System` |
|---|---|---|
| 모터 구분 | 노드 ID만 | **(노드 ID, 채널)** |
| 한 노드에 조인트 2개 | 불가 | 가능 (`channel` 0, 1) |

**DeviceContainer를 내부에서 소유** — `device_container_node`를 따로 띄우지 않습니다.
`controller_manager`가 하드웨어 플러그인을 로드하면서 그 프로세스(`ros2_control_node`) 안에서 CANopen 스택이 함께 뜹니다.
그래서 CAN 권한(`setcap`)이 `ros2_control_node`에 필요합니다 (J-2).

**URDF `<ros2_control>` 태그** (`urdf/my_robot_ros2_control.xacro`)

```xml
<ros2_control name="MyRobotCanopenSystem" type="system">
  <hardware>
    <plugin>canopen_ros2_control/Cia402System</plugin>
    <param name="bus_config">${bus_config}</param>
    <param name="master_config">${master_config}</param>
    <param name="can_interface_name">${can_interface}</param>
  </hardware>

  <!-- 채널 0 = 0x6000 (오른쪽) -->
  <joint name="wheel_right_joint">
    <param name="node_id">1</param>
    <param name="channel">0</param>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <!-- 채널 1 = 0x6800 (왼쪽) -->
  <joint name="wheel_left_joint">
    <param name="node_id">1</param>
    <param name="channel">1</param>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

- 두 바퀴가 **같은 `node_id`(1)** 를 쓰고 `channel`로만 구분합니다.
- `channel`을 생략하면 0으로 처리됩니다 (기존 단일 축 설정과 호환).
- 예전의 `axis`, `wheel_radius` 파라미터는 없습니다. 단위 환산은 `bus.yml`이 담당합니다 (J-5).

메인 URDF에는 **조건부로** include 합니다. 시뮬레이션에서는 Gazebo 플러그인이 바퀴를 구동하므로
`ros2_control` 태그가 있으면 충돌합니다.

```xml
<xacro:arg name="use_ros2_control" default="false"/>
<xacro:if value="$(arg use_ros2_control)">
  <xacro:include filename="$(find my_robot)/urdf/my_robot_ros2_control.xacro"/>
  <xacro:my_robot_ros2_control bus_config="$(arg bus_config)"
                               master_config="$(arg master_config)"
                               can_interface="can0"/>
</xacro:if>
```

**모터 활성화는 서비스로 (중요)**

upstream `Cia402System`은 런치만으로는 모터를 활성화하지 **않습니다.** 런치 직후 두 축은
Switch On Disabled 상태이고 모드도 선택되지 않아, `/cmd_vel_out`을 보내도 바퀴가 돌지 않습니다.
펌웨어는 Switch On 명령을 받을 때 릴레이를 켜므로, **릴레이도 이때까지 꺼져 있는 것이 정상**입니다.

런치를 띄운 뒤 **다른 터미널에서** 채널마다 아래 순서로 호출합니다 (source 3줄 먼저).

```bash
# [Pi] 터미널 2
ros2 service call /right_wheel/init std_srvs/srv/Trigger           # ← 이때 릴레이가 붙음
ros2 service call /right_wheel/velocity_mode std_srvs/srv/Trigger
ros2 service call /left_wheel/init std_srvs/srv/Trigger
ros2 service call /left_wheel/velocity_mode std_srvs/srv/Trigger
```

네 번 모두 `success=True`가 나오면 `/cmd_vel_out`(조이스틱·Nav2)으로 주행할 수 있습니다.

| 서비스 | 하는 일 |
|---|---|
| `init` | CiA 402 상태머신을 Operation Enabled까지 진행 (`06 → 07 → 0F`) |
| `velocity_mode` | 동작 모드를 Profile Velocity(3)로 설정 |
| `recover` | Fault 해제(Fault Reset) 후 Operation Enabled로 복귀 |
| `halt` | 정지 |

> **이 수동 호출은 다음 작업에서 런치 파일로 자동화할 예정입니다.** 예전 커스텀 드라이버는
> `on_activate()`에서 자동으로 했지만, upstream 하드웨어 인터페이스에는 그 기능이 없습니다.

**E-Stop 이후 복구** — 런치를 다시 띄울 필요가 없습니다.

```bash
# [Pi] E-Stop 해제 후, 터미널 2
ros2 service call /right_wheel/recover std_srvs/srv/Trigger
ros2 service call /right_wheel/velocity_mode std_srvs/srv/Trigger
ros2 service call /left_wheel/recover std_srvs/srv/Trigger
ros2 service call /left_wheel/velocity_mode std_srvs/srv/Trigger
```

**컨트롤러 설정** (`config/my_robot_controllers.yaml`)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50                # STM32 제어주기 20ms 와 일치
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

diff_drive_controller:
  ros__parameters:
    left_wheel_names:  ["wheel_left_joint"]
    right_wheel_names: ["wheel_right_joint"]
    wheel_separation: 0.185
    wheel_radius: 0.0325
    publish_rate: 50.0
    odom_frame_id: odom
    base_frame_id: base_footprint
    open_loop: false               # 엔코더 피드백으로 오도메트리 계산
    position_feedback: false       # 속도 피드백으로 오도메트리 계산 (위치 피드백은 아직 미검증)
    enable_odom_tf: false          # EKF 가 TF 를 발행하므로 반드시 false
    cmd_vel_timeout: 1.0
    linear:
      x: { has_velocity_limits: true, max_velocity: 0.5, min_velocity: -0.5,
           has_acceleration_limits: true, max_acceleration: 1.0, min_acceleration: -1.0 }
    angular:
      z: { has_velocity_limits: true, max_velocity: 2.0, min_velocity: -2.0,
           has_acceleration_limits: true, max_acceleration: 3.0, min_acceleration: -3.0 }
```

> **`enable_odom_tf: false`가 핵심입니다.** 기존 `base_controller.py`의 `publish_tf:=false`와 같은 역할로,
> `odom → base_footprint` TF는 IMU를 융합하는 EKF가 발행해야 정확합니다. 둘 다 켜면 TF가 충돌합니다.
>
> 이제 펌웨어가 위치(`0x6064`)를 보내므로 `position_feedback: true`로 바꿀 수 있지만, 그 상태의 오도메트리는
> 아직 측정하지 않았습니다. 바꾼다면 F-6(제자리 360° 회전)으로 먼저 검증하세요.

### J-8. 기존 스택과 연결

**인터페이스를 동일하게 유지**하는 것이 목표입니다. 리매핑으로 해결합니다.

```
[변경 전]  twist_mux → /cmd_vel_out → base_controller → 시리얼 → Arduino
                                           ↓ /odom
                                          EKF → TF

[변경 후]  twist_mux → /cmd_vel_out → diff_drive_controller → CANopen → STM32
                                           ↓ /odom
                                          EKF → TF
```

`robot_control.launch.py`의 `ros2_control_node`에 리매핑을 추가합니다.

```python
control_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[robot_description, controllers_file],
    remappings=[
        ('/diff_drive_controller/cmd_vel_unstamped', '/cmd_vel_out'),
        ('/diff_drive_controller/odom', '/odom'),
    ],
    output='screen',
)
```

**중복 노드 제거 (중요)**

`bringup_real.launch.py`가 `robot_state.launch.py`와 `robot_control.launch.py`를 모두 include 하므로
`robot_state_publisher`가 두 개 뜨지 않도록 인자로 제어합니다.

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg, 'launch', 'robot_control.launch.py')),
    launch_arguments={'use_robot_state_publisher': 'false'}.items(),
),
```

`robot_state.launch.py`의 `joint_state_publisher`도 꺼야 합니다. 실물에서는 `joint_state_broadcaster`가
**실제 엔코더 값**으로 `/joint_states`를 발행하므로, URDF만 보고 0을 발행하는 `joint_state_publisher`와
충돌합니다.

| 노드 | 값의 출처 | Arduino 구성 | STM32 구성 |
|---|---|---|---|
| `joint_state_publisher` | URDF만 보고 0 고정 | **사용** | 없음 |
| `joint_state_broadcaster` | 실제 하드웨어 state_interface | 없음 | **사용** |
| `robot_state_publisher` | `/joint_states` + URDF → TF 계산 | 사용 (1개) | 사용 (1개) |

**해결 — bringup 파일을 구성별로 분리**

조건부 인자를 쓰는 대신, 각 bringup이 **자기 구성에 필요한 노드를 온전히 포함**하도록 나눕니다.
이러면 인자가 필요 없고, 두 경로가 서로 간섭하지 않으며, 각각 단독 실행도 자연스럽습니다.

```
bringup_real_arduino.launch.py          (Arduino Serial)
  ├ robot_state.launch.py               # robot_state_publisher + joint_state_publisher
  ├ base_controller.py (publish_tf:=false, cmd_vel → /cmd_vel_out)
  ├ imu_node.py / ekf_node / ydlidar_node / joy_teleop
  └ → 원본 그대로, 수정 불필요

bringup_real_stm32.launch.py            (STM32 + CANopen)
  ├ robot_control.launch.py             # robot_state_publisher + joint_state_broadcaster
  │                                     #   + diff_drive_controller (+ ros2_control_node)
  ├ imu_node.py / ekf_node / ydlidar_node / joy_teleop
  └ → robot_state.launch.py 를 include 하지 않음 (중복 없음)
```

`mapping.launch.py`·`navigation.launch.py`는 `base:=` 인자로 둘 중 하나를 선택합니다.

```python
base = LaunchConfiguration('base')
use_stm32   = PythonExpression(["'", base, "' == 'stm32'"])
use_arduino = PythonExpression(["'", base, "' == 'arduino'"])

return LaunchDescription([
    DeclareLaunchArgument('base', default_value='stm32'),

    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'bringup_real_stm32.launch.py')),
        condition=IfCondition(use_stm32),
    ),
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'bringup_real_arduino.launch.py')),
        condition=IfCondition(use_arduino),
    ),
    # ... SLAM 또는 Nav2 ...
])
```

```bash
ros2 launch my_robot mapping.launch.py                 # STM32 (기본)
ros2 launch my_robot mapping.launch.py base:=arduino   # Arduino
```

> `robot_control.launch.py`가 `robot_state_publisher`를 직접 포함하므로 단독 실행 시에도 TF가 정상 발행됩니다.
> 이때 URDF는 `use_ros2_control:=true`로 처리된 것이지만, `robot_state_publisher`는 링크·조인트만 읽고
> `<ros2_control>` 태그는 무시하므로 문제되지 않습니다.

### J-9. 검증

**준비** — 로봇을 받침대에 올려 **바퀴가 바닥에 닿지 않게** 합니다. 처음 확인할 때는 반드시 이렇게 하세요.

터미널은 4개를 씁니다. **모든 [Pi] 터미널에서 먼저 아래 3줄**을 실행합니다 (`.bashrc`에 넣었다면 생략).

```bash
source /opt/ros/humble/setup.bash
source ~/canopen_ws/install/setup.bash
source ~/robot_ws/install/setup.bash
```

**1. 실행**

```bash
# [Pi] 터미널 1 — 하드웨어 + IMU + EKF + 라이다 + 조이스틱 + SLAM 한 번에
ros2 launch my_robot mapping.launch.py
```

하드웨어만 따로 확인하고 싶다면 `mapping.launch.py` 대신 아래를 씁니다.
```bash
ros2 launch my_robot robot_control.launch.py
```

**2. 모터 활성화** (J-7)

```bash
# [Pi] 터미널 2
ros2 service call /right_wheel/init std_srvs/srv/Trigger
ros2 service call /right_wheel/velocity_mode std_srvs/srv/Trigger
ros2 service call /left_wheel/init std_srvs/srv/Trigger
ros2 service call /left_wheel/velocity_mode std_srvs/srv/Trigger
```

**3. 단계별 확인**

```bash
# [Pi] 터미널 2

# 1) 컨트롤러 활성화
ros2 control list_controllers
#   joint_state_broadcaster  active
#   diff_drive_controller    active

# 2) 노드 중복 없음
ros2 node list | grep robot_state              # 1개만
ros2 topic info /joint_states --verbose        # Publisher: joint_state_broadcaster

# 3) TF 발행 주체 (EKF 하나여야 함)
ros2 run tf2_tools view_frames
#   odom → base_footprint,  Average rate ≈ 30Hz (EKF frequency)
#   50Hz 로 나오면 diff_drive_controller 가 아직 TF 를 내고 있는 것

# 4) 백포트 라이브러리만 로드됐는지 (아무것도 출력되지 않아야 정상)
sudo cat /proc/$(pgrep -f ros2_control_node)/maps \
  | grep -o "/[^ ]*\(canopen\|lely\)[^ ]*\.so[^ ]*" | sort -u | grep "/opt/ros"
```

**4. 주행 검증** (바퀴를 공중에 띄우고)

```bash
# [Pi] 터미널 3 — CAN 모니터
candump can0 | grep -E " 181 | 281 | 301 | 381 | 481 | 501 "
```

```bash
# [Pi] 터미널 2 — 직진 0.1 m/s  (Ctrl+C 로 멈춤, 1초 뒤 cmd_vel_timeout 으로 정지)
ros2 topic pub --rate 20 /cmd_vel_out geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
# candump: 301, 501 모두 64 00 00 00 (=100 mm/s)
#          181, 381 모두 27 00 03     (Operation Enabled, Profile Velocity)

# 제자리 회전 1.0 rad/s
ros2 topic pub --rate 20 /cmd_vel_out geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 1.0}}"
# 계산상 기대값: ±(1.0 × 0.185)/2 = ±0.0925 m/s → 301 = 5C 00 00 00 (+92), 501 = A4 FF FF FF (-92)
```

```bash
# [Pi] 터미널 4 — 직진 명령 중 관절 상태
ros2 topic echo /joint_states --once
```

직진 0.1 m/s일 때 실측값:
```
name:     [wheel_right_joint, wheel_left_joint]
position: [58.31..., 58.52...]        # rad, 바퀴를 돌리는 동안 계속 증가
velocity: [3.0769..., 3.0769...]      # rad/s = 100 mm/s × 0.03076923
effort:   [.nan, .nan]                # effort 는 백포트에서 제외 (정상)
```

> 회전 시 `±92`, 오도메트리 각속도(`/odom`의 `twist.twist.angular.z` ≈ 0.976, 명령 대비 2.4% 오차)는
> **이전 커스텀 드라이버로 측정한 값**입니다. 단위 환산이 같은 결과를 내도록 설정했지만, 백포트 드라이버로는
> 아직 다시 재지 않았습니다.

**5. 안전 계통 확인**

1. 직진 명령을 보내는 중에 **E-Stop**을 누릅니다 → 바퀴 정지, 릴레이 OFF.
2. E-Stop을 해제하고 J-7의 `recover` → `velocity_mode` 순서로 호출합니다 → 릴레이 ON.
3. 직진 명령을 다시 보내 바퀴가 도는지 확인합니다 (**런치 재시작 없이 복귀**).

**6. 단일 채널 하위 호환 확인 (선택)**

upstream에 "채널 기능을 추가해도 기존 단일 축 사용자에게 영향이 없다"는 근거로 낸 시험입니다.
평소 주행에는 필요 없습니다.

```bash
# [Pi] 1) 2채널 설정 백업
cp ~/robot_ws/src/my_robot_canopen/config/my_robot/bus.yml \
   ~/robot_ws/src/my_robot_canopen/config/my_robot/bus.yml.2ch

# 2) bus.yml 에서 num_channels, channel_names 두 줄 삭제
nano ~/robot_ws/src/my_robot_canopen/config/my_robot/bus.yml
#    (지울 줄로 커서를 옮기고 Ctrl+K → Ctrl+O, Enter → Ctrl+X)

# 3) install 에 반영 (PDO 는 그대로라 dcfgen 불필요)
cd ~/robot_ws && colcon build --packages-select my_robot_canopen

# 4) 드라이버 단독 실행 — 로그에 "num_channels: 1"
ros2 launch my_robot_canopen my_robot_canopen.launch.py
```

```bash
# [Pi] 다른 터미널
ros2 service list | grep -E "/init|velocity_mode|/target"   # /motor/... 만 보여야 함
ros2 service call /motor/init std_srvs/srv/Trigger
ros2 service call /motor/velocity_mode std_srvs/srv/Trigger
ros2 service call /motor/target canopen_interfaces/srv/COTargetDouble "{target: 3.0769}"
ros2 topic echo /motor/joint_states --once                  # name: [motor] 하나
```

오른쪽 바퀴(채널 0)만 돌고, candump에서 `181`·`301`만 변하면 정상입니다. 끝나면 되돌립니다.

```bash
# [Pi]
cp ~/robot_ws/src/my_robot_canopen/config/my_robot/bus.yml.2ch \
   ~/robot_ws/src/my_robot_canopen/config/my_robot/bus.yml
cd ~/robot_ws && colcon build --packages-select my_robot_canopen
```

**최종 확인**: 조이스틱으로 실제 주행 → `mapping.launch.py`로 지도 그리기 → 기존과 동일하게 동작

---

## 14. 하드웨어 스펙 & 파라미터

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
| 통신 | `ROS_DOMAIN_ID=30`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (무선 지터 개선, Pi·PC 양쪽 동일 설정 필요) |
| 조이스틱 | 8BitDo Ultimate 2 Wireless, VID:PID `2dc8:310b` |
| 조이스틱 속도 스케일 | linear 0.22 m/s, angular 1.0 rad/s |
| IMU | MPU6050 (I2C 0x68), DLPF 44Hz, 자이로 Z만 EKF 융합 |
| IMU 장착 위치 | base_link 기준 `x=0.025, y=-0.055, z=0.0975 m` |
| EKF | `robot_localization`, two_d_mode, odom→base_footprint TF 발행 |
| 360° 회전 오차 | 바퀴만 27.8° → 융합 1.4° (약 20배 개선) |
| USB 포트 고정 | udev: YDLIDAR(10c4:ea60)→`/dev/ydlidar`, Arduino(1a86:7523)→`/dev/arduino` |
| TF 트리 | `map → odom → base_footprint → base_link → …` |
| **모터 제어기 (현행)** | **STM32 NUCLEO-F446RE, CANopenNode 슬레이브 (CiA 402)** |
| **CAN 어댑터** | **candleLight USB-CAN (`gs_usb`), 500 kbps** |
| **CAN 노드 ID** | **마스터(Pi) = 2, 슬레이브(STM32) = 1** |
| **CiA 402 축 매핑** | **채널 0(오른쪽) `0x6040~0x60FF` / 채널 1(왼쪽) `0x6840~0x68FF` (+0x800 오프셋)** |
| **동작 모드** | **Profile Velocity (mode 3), `0x6502` = `0x04`** |
| **SYNC / 제어 주기** | **20 ms (50 Hz) — 마스터 SYNC, STM32 PID, ros2_control update_rate 모두 일치** |
| **PWM 타이머 (STM32)** | **TIM1, Prescaler 87, Period 255 → 약 3.7 kHz (MDD10A 20kHz 스펙 내)** |
| **엔코더 타이머** | **TIM3(축1) / TIM4(축2), `TIM_ENCODERMODE_TI12` (4체배)** |
| **OD 단위 / 환산** | **속도 mm/s, 위치 엔코더 카운트 / `scale_vel_to_dev: 32.5`(rad/s→mm/s), `scale_pos_from_dev: 0.00475999`(count→rad)** |
| **CANopen 드라이버** | **`ros2_canopen` 다축 백포트 ([humble-multichannel-backport](https://github.com/lch-98/ros2_canopen/tree/humble-multichannel-backport)), `num_channels: 2`** |
| **ros2_control** | **`canopen_ros2_control/Cia402System` + `diff_drive_controller`** |
| **명령 배선 (현행)** | **`twist_mux` → `/cmd_vel_out` → `diff_drive_controller` → CANopen → STM32** |
| 명령 배선 | `joy→/cmd_vel_joy`, `nav2→/cmd_vel_nav→velocity_smoother→/cmd_vel` → `twist_mux` → `/cmd_vel_out` → base_controller |

---

## 15. 트러블슈팅

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

**STM32 · CANopen (J단계)**

- **드라이버 부팅 중 SIGSEGV로 프로세스 사망 (`exit code -11`)** → OD에 `0x6502`(Supported drive modes)가 없는 것. 로그 마지막에 `async_sdo_read: id=1 index=0x6502 ... object does not exist`와 abort `0x06020000`이 찍힙니다. CANopenEditor에서 `0x6502`(UNSIGNED32, ro, DefaultValue `0x04`) 추가 후 **STM32 재플래시**까지 해야 해결됩니다. Pi 측 EDS만 고치면 안 됩니다 — 마스터는 런타임에 슬레이브 OD를 직접 조회하기 때문입니다. 다축이면 `0x6D02`도 추가.
- **`ros2 service call`이 응답 없이 멈춤** → 서비스 서버(드라이버 노드)가 처리 도중 크래시한 것. CLI는 오지 않을 응답을 기다립니다. launch 터미널의 `process has died`를 먼저 확인하세요.
- **PDO 매핑 거부 (`0x06040041`)** → CANopenEditor에서 해당 객체의 Access PDO를 `tr`(TPDO 방향)로 설정하고 OD.c 재생성. RPDO 방향 객체는 EDS의 `AccessType`을 **`rww`**로 (CANopenEditor는 `rww`를 생성하지 못하므로 export 후 직접 수정).
- **`Boot Timeout: The boot configure process timeout!`** → `boot_timeout_ms` 기본값이 ~20ms로 매우 짧습니다. `bus.yml`에 `boot_timeout_ms: 5000` 명시. 1차 실패 후 NMT 리셋으로 재시도해 성공하는 경우도 정상 동작이지만, 매번 5초를 소모하면 값을 더 늘리세요.
- **마스터가 슬레이브 PDO 매핑을 설정하지 못함** → `dcfgen`에 **`-r` 플래그** 누락. `dcfgen -d . -r -v bus.yml`.
- **`@BUS_CONFIG_PATH@`가 치환되지 않음** → `bus.yml`의 `dcf_path`를 install 공간의 실제 절대경로로 지정.
- **CAN 소켓 열기 실패 (`Operation not permitted`)** → `netdev` 그룹만으로는 부족합니다. `sudo setcap cap_net_raw,cap_net_admin=eip <실행파일>`. **ros2_control을 쓰면 `ros2_control_node`에도** 부여해야 합니다(소켓을 여는 주체가 바뀜). setcap을 붙이면 `LD_LIBRARY_PATH`가 무시되므로, 라이브러리를 못 찾으면 `/etc/ld.so.conf.d/`에 경로를 등록하고 `ldconfig`.

**ros2_canopen 백포트 (J단계)**

- **서비스가 `/right_wheel/init`이 아니라 `/motor/init`으로만 보임** → 백포트가 아니라 **apt 버전 본체가 로드**된 것입니다. `setcap`이 걸린 `ros2_control_node`는 `source` 경로를 무시하므로, 플러그인만 백포트이고 그 안의 `libnode_canopen_cia402_driver.so` 등은 `/opt/ros/humble`에서 잡힙니다. 확인: `sudo cat /proc/$(pgrep -f ros2_control_node)/maps | grep -o "/[^ ]*canopen[^ ]*\.so" | sort -u` 에 `/opt/ros/humble`이 섞여 있으면 이 경우입니다. 2단계 6번(라이브러리 경로 등록)을 하세요. 드라이버 로그에 `num_channels: 2`가 찍히는지도 함께 보세요.
- **로드 경로에 `liblely_master_bridge.so`, `libnode_canopen_basic_master.so`만 `/opt/ros`로 남음** → 백포트 빌드 때 `canopen_master_driver`를 빠뜨린 것. `colcon build --packages-up-to canopen_ros2_control canopen_master_driver`로 다시 빌드하고 2단계 6번을 다시 실행하세요 (새 폴더가 목록에 추가되어야 함).
- **`ldconfig -p` 결과에 같은 라이브러리가 두 줄씩 나옴** → 정상입니다. **위쪽 줄이 실제로 쓰이는 것**이므로, 첫 줄이 `canopen_ws`이면 됩니다.
- **드라이버 단독 실행 시 `CanController: Operation not permitted`로 즉시 사망** → 실행되는 `device_container_node`가 백포트 빌드본(`~/canopen_ws/install/...`)인데 그 파일에 권한이 없는 것. J-2의 2)를 실행하세요. `canopen_ws`를 다시 빌드한 뒤에도 같은 증상이 나옵니다.
- **런치는 정상인데 바퀴가 안 돌고 릴레이도 안 붙음** → upstream `Cia402System`은 모터를 자동 활성화하지 않습니다. `init` → `velocity_mode` 서비스를 채널마다 호출하세요 (J-7). `init` 호출 시점에 릴레이가 붙어야 정상입니다.
- **`channel_names`를 생략했더니 `InvalidServiceNameError: topic name token must not start with a number`로 사망** → 기본 채널 이름이 `노드이름/0`처럼 숫자로 시작해 서비스 이름이 될 수 없는 것 (upstream #427에서 수정 제안 중). `bus.yml`에 `channel_names`를 문자로 시작하는 이름으로 명시하세요.
- **E-Stop 후 `init`만 다시 호출하면 안 돌아감** → Fault 상태에서는 `recover`를 먼저 호출해야 합니다. `recover` → `velocity_mode` 순서 (J-7).

**CAN 물리 계층**

- **통신이 간헐적으로 끊기고 USB-CAN LED가 점멸을 멈춤** → 대부분 **GND 접촉 불량**입니다. `ip -details -statistics link show can0`의 `error-warn`/`error-pass`/`bus-off` 카운터를 확인하세요. 정상이면 0에 가깝고, 접촉 불량이면 수만~수십만으로 폭증합니다. 브레드보드 GND는 신뢰하지 말고 납땜하거나 스크류 터미널을 쓰세요.
- **리셋 버튼으로는 안 살아나는데 재플래시하면 살아남** → 펌웨어 문제가 아니라 **보드를 만지는 물리적 진동으로 접촉이 회복된 것**일 수 있습니다. 재플래시가 "고쳤다"고 단정하지 말고 카운터를 확인하세요.
- **`bus-off` 누적 후 `ip link down/up`으로 복구되지 않음** → `gs_usb` 커널 드라이버나 어댑터 펌웨어가 비정상 상태에 갇힌 것. 복구 순서: ① launch 종료 → ② **STM32 USB 전원 분리**(버스를 조용하게) → ③ `sudo reboot` → ④ STM32 연결 → ⑤ `ip link set can0 up ... restart-ms 100`.
- **`restart-ms`가 자꾸 0으로 돌아감** → `ip link down/up` 하면 초기화됩니다. up 할 때마다 `restart-ms 100`을 함께 지정하세요.
- **루프백은 되는데 실제 통신이 안 됨** → 루프백은 어댑터 내부에서만 도는 테스트라 **GND를 타지 않습니다.** 어댑터 결백만 증명할 뿐 버스 전체의 건강을 보장하지 않습니다.
- **종단 저항 확인** → 전원을 끄고 CAN_H ↔ CAN_L 저항 측정. **60Ω**(120Ω 두 개 병렬)이 정상. 120Ω이면 한쪽만, ∞면 단선, 0Ω이면 단락입니다.

**모터 제어**

- **저속에서만 모터가 "덜컹거리고 휙 튐", 후진·좌회전에서 특히 심함** → **PWM 주파수가 모터 드라이버 스펙을 초과**한 것. MDD10A 권장 상한은 20kHz인데, TIM1 `Prescaler=15`이면 `84MHz / 16 / 256 ≈ 20.5kHz`로 살짝 넘깁니다. 낮은 duty(저속)에서 온 시간이 MOSFET 스위칭 시간에 근접해 전류가 제대로 흐르지 않습니다. `Prescaler=87`로 약 3.7kHz까지 낮추면 해결됩니다. **"Arduino에서는 문제없었다"가 결정적 단서** — `analogWrite`는 490Hz입니다.
- **바퀴가 명령의 약 31배 속도로 회전** → 단위 환산 누락. `diff_drive_controller`는 바퀴 조인트에 **rad/s**를 명령하고, 펌웨어는 **mm/s**를 받습니다. `bus.yml`의 `scale_vel_to_dev`가 기본값(1000)이면 3.077 rad/s가 3077로 전송됩니다. `32.5`(= 반지름 0.0325 m × 1000)로 설정하세요 (J-5). 첫 테스트는 반드시 바퀴를 띄우고 하세요.
- **`ros2 service call`로 명령하면 1~3초 지연** → 제어 경로가 아니라 **CLI의 노드 생성·디스커버리 오버헤드**입니다. candump 타임스탬프로 "RPDO 값이 바뀐 시점 → 모터 반응"을 재면 약 42ms(제어주기 2사이클)로 정상입니다. 실제 주행에서는 연결이 유지되므로 이 지연이 없습니다.

**ros2_control 통합**

- **`Unable to parse the value of parameter robot_description as yaml`** → `Command([...])` 결과를 `ParameterValue(..., value_type=str)`로 감싸세요. URDF 문자열을 launch가 YAML로 파싱하려다 실패하는 것입니다.
- **`Loader for controller 'diff_drive_controller' not found`** → `sudo apt install ros-humble-diff-drive-controller`. `ros2 control` 명령이 없으면 `ros-humble-ros2controlcli`도 설치.
- **`ros2_control_node`가 즉시 종료 (`exit code 1`), `failed to load shared library 'librmw_cyclonedds_cpp.so' ... libddsc.so.0: cannot open shared object file`** → **setcap과 CycloneDDS의 충돌**입니다. capability가 부여된 실행 파일은 보안상 `LD_LIBRARY_PATH`가 무시되어 DDS 라이브러리를 찾지 못합니다. 라이브러리 경로를 시스템에 등록하세요.
  ```bash
  find / -name "libddsc.so*" 2>/dev/null      # 실제 위치 확인 (Pi는 aarch64-linux-gnu 하위)

  sudo tee /etc/ld.so.conf.d/ros-humble.conf << 'EOF'
  /opt/ros/humble/lib
  /opt/ros/humble/lib/aarch64-linux-gnu
  EOF
  sudo ldconfig
  ldconfig -p | grep libddsc                  # 경로가 출력되면 성공
  ```
  ROS2는 아키텍처별 하위 폴더(`aarch64-linux-gnu`)에 DDS 라이브러리를 두므로 **상위 `lib`만 등록하면 안 됩니다.**
- **`spawner`가 `/controller_manager/list_controllers`를 무한 대기** → `controller_manager`가 안 뜬 것입니다. `robot_state_publisher`와는 무관하니, 로그 맨 앞에서 `ros2_control_node`가 왜 죽었는지 먼저 확인하세요(위 항목이 흔한 원인).
- **`Could not enable FIFO RT scheduling policy`** → 실시간 우선순위 미부여. `realtime` 그룹을 만들고 `/etc/security/limits.d/`에 `rtprio 99`를 설정한 뒤 재로그인. 50Hz 제어에는 없어도 동작하지만, EKF가 주기를 못 맞추는 원인이 될 수 있습니다.
- **`slam_toolbox: Message Filter dropping message: frame 'base_scan' ... queue is full`** → `robot_state_publisher`가 **두 개** 떠서 TF 타임스탬프가 뒤섞인 것. `ros2 node list | grep robot_state`로 확인하고, `robot_control.launch.py`를 include할 때 `use_robot_state_publisher: 'false'`를 넘기세요.
- **RViz에서 로봇 모델이 사라지거나 바퀴 TF가 고정됨** → `/joint_states` 발행자가 여러 개인지 확인(`ros2 topic info /joint_states --verbose`). 실물에서는 `joint_state_broadcaster` 하나만 있어야 합니다. `joint_state_publisher`(URDF만 보고 0 발행)가 함께 떠 있다면 끄세요. (CANopen 드라이버도 `/motor/joint_states`, `/right_wheel/...` 같은 자체 토픽을 내지만 이름이 달라 `/joint_states`와 충돌하지 않습니다.)

**원격 시각화 (무선)**

- **RViz가 매우 끊기는데 대역폭은 여유 있음** → 총량(약 70KB/s)이 아니라 **지터**가 문제입니다. `ros2 topic hz`를 Pi와 PC에서 각각 재보면, PC 쪽만 `std dev`가 10배 이상 커지고 `min: 0.000s / max: 0.350s`처럼 메시지가 뭉쳐서 도착합니다. **CycloneDDS로 교체하면 크게 개선**됩니다.
  ```bash
  sudo apt install ros-humble-rmw-cyclonedds-cpp        # Pi와 PC 양쪽
  echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
  ```
  **반드시 양쪽 모두** 설정해야 합니다(한쪽만 바꾸면 통신 자체가 끊깁니다). 추가로 RViz의 TF 디스플레이를 끄거나 표시 프레임을 줄이면 렌더링 부하가 줄어듭니다.
- **`New subscription discovered on topic '/scan', requesting incompatible QoS`** → 구독자 중 하나가 RELIABLE로 붙은 것. 라이다는 BEST_EFFORT로 발행하므로 RViz의 LaserScan Reliability를 `Best Effort`로 맞추세요.

---

## License

MIT License