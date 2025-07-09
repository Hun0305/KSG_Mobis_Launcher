# dual_yolo_obj.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input0', default_value='/cam0/image_raw', description='Input for YOLO object 0'),
        DeclareLaunchArgument('input1', default_value='/cam1/image_raw', description='Input for YOLO object 1'),

        SetEnvironmentVariable('CUDA_VISIBLE_DEVICES', '0'),

        Node(
            package='camera_pkg',
            executable='yolo_obj',
            name='yolo_obj0',
            namespace='cam0',
            parameters=[
                {'device': 'cuda:0'},
                {'model_path': '/home/sg/contest_ws/src/camera_pkg/camera_pkg/model/yolo11n.pt'},
                {'threshold': 0.5},
                {'enable': True},
                {'show_result': True}
            ],
            remappings=[
                ('/image_raw', LaunchConfiguration('input0'))
            ]
        ),

        Node(
            package='camera_pkg',
            executable='yolo_obj',
            name='yolo_obj1',
            namespace='cam1',
            parameters=[
                {'device': 'cuda:0'},
                {'model_path': '/home/sg/contest_ws/src/camera_pkg/camera_pkg/model/yolo11n.pt'},
                {'threshold': 0.5},
                {'enable': True},
                {'show_result': True}
            ],
            remappings=[
                ('/image_raw', LaunchConfiguration('input1'))
            ]
        )
    ])


