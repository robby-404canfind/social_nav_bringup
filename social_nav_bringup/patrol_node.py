#!/usr/bin/env python3
"""patrol() Unit Action 노드.

Semantic Location의 웨이포인트를 순서대로 순환 방문합니다.
go_to()를 반복 호출하는 상위 액션입니다.

사용법 (Docker 컨테이너 안에서):
  ros2 run social_nav_bringup patrol_node --ros-args \
    -p waypoints:="['office_desk_1','meeting_room','corridor_a']" \
    -p duration_sec:=120.0

공유 워크스페이스의 source YAML을 직접 수정했다면:
  ros2 run social_nav_bringup patrol_node --ros-args \
    -p locations_file:=/home/hunav_webots_ws/src/social_nav_bringup/config/semantic_locations.office.yaml \
    -p waypoints:="['office_desk_1','meeting_room','corridor_a']" \
    -p duration_sec:=120.0
"""

import time
import threading
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node

from social_nav_bringup.nav2_navigator import Nav2Navigator
from social_nav_bringup.go_to_node import (
    exec_go_to,
    get_stuck_params,
    load_semantic_locations,
    resolve_locations_file,
)


def exec_patrol(
    node: Node,
    navigator: Nav2Navigator,
    nav_feedback: Dict[str, Any],
    waypoints: List[Dict[str, float]],
    waypoint_names: List[str],
    duration_sec: float,
    stuck_params: Dict[str, Any],
) -> bool:
    """patrol() 단위 액션 실행 로직.

    지정된 웨이포인트를 순서대로 순환 방문합니다.
    duration_sec은 새 웨이포인트로 출발하기 전에 확인합니다.
    이미 실행 중인 go_to()를 강제로 취소하지는 않습니다.

    Args:
        node: ROS2 노드
        navigator: Nav2Navigator 인스턴스
        nav_feedback: feedback dict
        waypoints: 좌표 리스트 [{"x", "y", "yaw"}, ...]
        waypoint_names: 이름 리스트 (로그용)
        duration_sec: 순찰 시간 기준값 (초)
        stuck_params: stuck 감지 파라미터

    Returns:
        True (시간 만료로 정상 종료) / False (중단)
    """
    if not waypoints:
        node.get_logger().error("[patrol] 웨이포인트가 비어 있습니다")
        return False

    start_time = time.time()
    idx = 0
    total = len(waypoints)
    cycle = 0

    node.get_logger().info(
        f"[patrol] 시작: {total}개 웨이포인트, duration={duration_sec}s"
    )

    while rclpy.ok():
        elapsed = time.time() - start_time
        if elapsed >= duration_sec:
            node.get_logger().info(
                f"[patrol] 시간 만료 ({duration_sec}s) → 정상 종료, "
                f"총 {cycle}회 순환"
            )
            return True

        name = waypoint_names[idx]
        goal = waypoints[idx]
        remaining = duration_sec - elapsed

        node.get_logger().info(
            f"[patrol] 웨이포인트 {idx + 1}/{total}: '{name}' "
            f"({goal['x']:.2f}, {goal['y']:.2f}), 남은 시간: {remaining:.0f}s"
        )

        success = exec_go_to(node, navigator, nav_feedback, goal, stuck_params)

        if success:
            node.get_logger().info(f"[patrol] '{name}' 도착 완료")
        else:
            node.get_logger().warn(f"[patrol] '{name}' 실패 → 다음 웨이포인트로 건너뜀")

        # 다음 웨이포인트 (순환)
        idx = (idx + 1) % total
        if idx == 0:
            cycle += 1
            node.get_logger().info(f"[patrol] 순환 {cycle}회 완료")

    node.get_logger().warn("[patrol] rclpy 종료 → 중단")
    return False


class PatrolNode(Node):
    """patrol() Unit Action 실행 노드.

    파라미터:
      - locations_file: semantic_locations.office.yaml 경로
      - waypoints: 순찰할 위치 이름 리스트 (YAML 배열 문자열)
      - duration_sec: 순찰 시간 기준값 (초, 기본 300)
      - action_timeout_sec: 각 go_to() 전체 타임아웃 (기본 180초)
      - hard_stuck_timeout_sec, progress_grace_sec 등: stuck 감지 파라미터
    """

    def __init__(self):
        super().__init__("patrol_node")

        self.declare_parameter("locations_file", "")
        self.declare_parameter("waypoints", [""])
        self.declare_parameter("duration_sec", 300.0)
        self.declare_parameter("action_timeout_sec", 180.0)
        self.declare_parameter("hard_stuck_timeout_sec", 60.0)
        self.declare_parameter("progress_grace_sec", 50.0)
        self.declare_parameter("progress_epsilon_m", 0.05)
        self.declare_parameter("near_goal_epsilon_m", 1.0)
        self._done = threading.Event()
        self._success = None

        # Nav2Navigator 생성
        self._nav_feedback: Dict[str, Any] = {
            "distance_remaining": None,
            "stamp": time.time(),
        }
        self._navigator = Nav2Navigator(
            self, feedback_cb=self._nav_feedback.update
        )

        # 워커 스레드 시작 (메인 스레드는 rclpy.spin_once() 반복)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        """웨이포인트 해석 → exec_patrol 실행."""
        try:
            locations_file = resolve_locations_file(
                self.get_parameter("locations_file").value
            )
            waypoint_names = list(self.get_parameter("waypoints").value)
            duration_sec = self.get_parameter("duration_sec").value

            stuck_params = get_stuck_params(self)

            # Semantic Location 로드
            if not locations_file:
                self.get_logger().error(
                    "[PatrolNode] semantic_locations.office.yaml을 찾지 못했습니다. "
                    "locations_file 파라미터를 명시하세요"
                )
                return

            try:
                locations = load_semantic_locations(locations_file)
            except Exception as e:
                self.get_logger().error(f"[PatrolNode] {e}")
                return

            # 웨이포인트 이름 → 좌표 변환
            waypoint_names = [n for n in waypoint_names if n]  # 빈 문자열 제거
            waypoints = []
            for name in waypoint_names:
                if name not in locations:
                    self.get_logger().error(
                        f"[PatrolNode] '{name}'이(가) locations에 없습니다. "
                        f"사용 가능: {list(locations.keys())}"
                    )
                    return
                waypoints.append(locations[name])

            self.get_logger().info(
                f"[PatrolNode] 순찰 경로: {waypoint_names}, duration={duration_sec}s "
                f"(file={locations_file})"
            )

            self._success = exec_patrol(
                self,
                self._navigator,
                self._nav_feedback,
                waypoints,
                waypoint_names,
                duration_sec,
                stuck_params,
            )

            if self._success:
                self.get_logger().info("[PatrolNode] === patrol() 완료: SUCCESS ===")
            else:
                self.get_logger().warn("[PatrolNode] === patrol() 완료: FAILED ===")
        finally:
            self._done.set()

    def done(self) -> bool:
        """워커 스레드 완료 여부를 반환합니다."""
        return self._done.is_set()


def main():
    rclpy.init()
    node = PatrolNode()
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node._navigator.cancel_all()
    finally:
        if node._thread.is_alive():
            node._thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
