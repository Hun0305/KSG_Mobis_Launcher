# dual_yolo_seg.py (Node 버전)
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input0', default_value='/cam0/image_raw', description='Input for YOLO seg 0'),
        DeclareLaunchArgument('input1', default_value='/cam1/image_raw', description='Input for YOLO seg 1'),

        # GPU 환경 설정
        SetEnvironmentVariable('CUDA_VISIBLE_DEVICES', '0'),

        Node(
            package='camera_pkg',
            executable='yolo_seg',
            name='yolo_seg0',
            namespace='cam0',
            output='screen',
            parameters=[
                {'device': 'cuda:0'},
                {'model_path': '/home/sg/contest_ws/src/camera_pkg/camera_pkg/model/best.pt'},
                {'threshold': 0.5}
            ],
            remappings=[
                ('/image_raw', LaunchConfiguration('input0')),
                ('detections', 'detections')  # => /cam0/detections
            ]
        ),

        Node(
            package='camera_pkg',
            executable='yolo_seg',
            name='yolo_seg1',
            namespace='cam1',
            output='screen',
            parameters=[
                {'device': 'cuda:0'},
                {'model_path': '/home/sg/contest_ws/src/camera_pkg/camera_pkg/model/best.pt'},
                {'threshold': 0.5}
            ],
            remappings=[
                ('/image_raw', LaunchConfiguration('input1')),
                ('detections', 'detections')  # => /cam1/detections
            ]
        )
    ])


