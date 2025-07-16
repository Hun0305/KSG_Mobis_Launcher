from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lidar_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 폴더 안의 모든 .py와 .yaml 파일을 복사하도록 수정
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.[py,yaml]*'))),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Lidar-based autonomous parking system for competition',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 각 Python 노드 스크립트를 실행 가능한 엔트리 포인트로 등록
            'lidar = lidar_pkg.lidar_publisher:main',
            'ultrasonic_publisher = lidar_pkg.ultrasonic_publisher:main',
            'parking_perception = lidar_pkg.parking_perception:main',
            'path_planner = lidar_pkg.path_planner:main',
            'vehicle_controller = lidar_pkg.vehicle_controller:main',
        ],
    },
)