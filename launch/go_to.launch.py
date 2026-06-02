"""go_to 노드 Launch 파일.

사용법 (Docker 컨테이너 안에서):
  ros2 launch social_nav_bringup go_to.launch.py \
    target:=meeting_room

기본적으로 패키지에 포함된 semantic_locations.office.yaml을 사용합니다.
공유 워크스페이스의 source 파일을 직접 수정했다면 locations_file로 명시 경로를 넘기세요.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_locations_file = os.path.join(
        get_package_share_directory("social_nav_bringup"),
        "config",
        "semantic_locations.office.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "target",
                default_value="",
                description="Semantic Location 이름 (예: meeting_room)",
            ),
            DeclareLaunchArgument(
                "locations_file",
                default_value=default_locations_file,
                description="semantic_locations.office.yaml 파일 경로 (기본: 패키지 기본 파일)",
            ),
            DeclareLaunchArgument(
                "target_x",
                default_value="nan",
                description="직접 좌표 X (target보다 우선)",
            ),
            DeclareLaunchArgument(
                "target_y",
                default_value="nan",
                description="직접 좌표 Y",
            ),
            DeclareLaunchArgument(
                "target_yaw",
                default_value="0.0",
                description="직접 좌표 Yaw (라디안)",
            ),
            Node(
                package="social_nav_bringup",
                executable="go_to_node",
                name="go_to_node",
                output="screen",
                parameters=[
                    {
                        "target": LaunchConfiguration("target"),
                        "locations_file": LaunchConfiguration("locations_file"),
                        "target_x": LaunchConfiguration("target_x"),
                        "target_y": LaunchConfiguration("target_y"),
                        "target_yaw": LaunchConfiguration("target_yaw"),
                    }
                ],
            ),
        ]
    )
