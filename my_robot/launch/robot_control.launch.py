import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_my_robot = get_package_share_directory('my_robot')
    pkg_canopen = get_package_share_directory('my_robot_canopen')

    bus_config = os.path.join(pkg_canopen, 'config', 'my_robot', 'bus.yml')
    master_config = os.path.join(pkg_canopen, 'config', 'my_robot', 'master.dcf')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('my_robot'), 'urdf', 'my_robot.urdf.xacro'])

    # URDF 문자열은 ParameterValue(value_type=str) 로 감싸야 YAML 파싱을 피할 수 있다
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', xacro_file,
                ' use_ros2_control:=true',
                ' bus_config:=', bus_config,
                ' master_config:=', master_config,
            ]),
            value_type=str
        )
    }

    controllers_file = os.path.join(pkg_my_robot, 'config', 'my_robot_controllers.yaml')

    # controller_manager — 하드웨어 플러그인 로드 + CANopen 스택 구동
    # 기존 아두이노 구성과 호환되도록 cmd_vel / odom 을 리매핑한다
    #   twist_mux → /cmd_vel_out → diff_drive_controller
    #   diff_drive_controller → /odom → EKF
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

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    ddc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # joint_state_broadcaster 가 올라온 뒤에 diff_drive_controller 를 올린다
    delay_ddc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[ddc_spawner],
        )
    )

    # ── 드라이브 자동 활성화 ──
    #   diff_drive_controller 가 올라온 뒤 두 축을 init → velocity_mode 로 한 번 켠다.
    #   실패하면 재시도하지 않는다 (init 에 Fault Reset 이 포함돼 있어, 반복하면
    #   E-Stop 해제 즉시 사람 확인 없이 다시 켜지기 때문). 고장 복귀는 수동 recover.
    #   끄려면: ros2 launch my_robot robot_control.launch.py auto_enable:=false
    auto_enable = LaunchConfiguration('auto_enable')

    drive_enabler = Node(
        package='my_robot',
        executable='drive_enabler.py',
        parameters=[{
            'channels': ['right_wheel', 'left_wheel'],   # bus.yml 의 channel_names 와 같아야 함
            'controller': 'diff_drive_controller',
        }],
        output='screen',
        condition=IfCondition(auto_enable),
    )

    delay_enable = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ddc_spawner,
            on_exit=[drive_enabler],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'auto_enable', default_value='true',
            description='true: 컨트롤러가 뜬 뒤 두 축을 자동으로 init + velocity_mode'),
        control_node,
        robot_state_pub,
        jsb_spawner,
        delay_ddc,
        delay_enable,
    ])