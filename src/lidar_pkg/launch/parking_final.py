# 파일 위치: contest_ws/src/lidar_pkg/launch/parking_final.py

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    lidar_pkg_dir = get_package_share_directory('lidar_pkg')
    slam_config_path = os.path.join(lidar_pkg_dir, 'launch', 'slam_config.yaml')

    # 1. LiDAR 퍼블리셔 노드
    lidar_publisher = Node(
        package='lidar_pkg',
        executable='lidar_publisher', # .py 제거
        name='lidar_publisher',
        output='screen'
        # 파라미터는 lidar_publisher.py 내부 기본값 사용
    )

    # 2. 초음파 센서 퍼블리셔 노드
    ultrasonic_publisher = Node(
        package='control_pkg',
        executable='ultrasonic_publisher', # .py 제거 (가정)
        name='ultrasonic_publisher',
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyACM0'},
            {'baud_rate': 9600},
            {'num_sensors': 10}
        ]
    )

    # 3. 주차 공간 인지 노드
    parking_perception = Node(
        package='lidar_pkg',
        executable='parking_perception', # .py 제거
        name='parking_perception',
        output='screen'
    )

    # 4. 경로 계획 노드
    path_planner = Node(
        package='lidar_pkg',
        executable='path_planner', # .py 제거
        name='path_planner',
        output='screen'
    )

    # 5. 차량 제어 노드
    vehicle_controller = Node(
        package='lidar_pkg',
        executable='vehicle_controller', # .py 제거
        name='vehicle_controller',
        output='screen',
        parameters=[
            {'target_velocity': 0.15},
            {'lookahead_distance': 0.8},
            {'max_steer_deg': 25.0},
            {'vehicle_length': 0.3}
        ]
    )

    # 6. cmd_vel_translator 노드 (번역기)
    cmd_vel_translator = Node(
        package='control_pkg',
        executable='cmd_vel_translator', # .py 제거 (가정)
        name='cmd_vel_translator_node'
    )

    # 7. control 노드 (아두이노 통신)
    control = Node(
        package='control_pkg',
        executable='control', # .py 제거 (가정)
        name='control_node'
    )

    # 8. SLAM 노드
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_path]
    )

    # 모든 노드를 런치 디스크립션에 추가
    ld.add_action(lidar_publisher)
    ld.add_action(ultrasonic_publisher)
    ld.add_action(parking_perception)
    ld.add_action(path_planner)
    ld.add_action(vehicle_controller)
    ld.add_action(cmd_vel_translator)
    ld.add_action(control)
    ld.add_action(slam_toolbox)

    return ld