#!/usr/bin/env python3
"""go_to() Unit Action 노드.

Semantic Location 이름을 받아 Nav2 NavigateToPose Goal로 변환하고,
피드백 모니터링 + stuck 감지를 수행합니다.

사용법 (Docker 컨테이너 안에서):
  ros2 run social_nav_bringup go_to_node --ros-args \
    -p target:=meeting_room

또는 직접 좌표 지정:
  ros2 run social_nav_bringup go_to_node --ros-args \
    -p target_x:=1.0 -p target_y:=2.0 -p target_yaw:=1.57

공유 워크스페이스의 source YAML을 직접 수정했다면:
  ros2 run social_nav_bringup go_to_node --ros-args \
    -p target:=meeting_room \
    -p locations_file:=/home/hunav_webots_ws/src/social_nav_bringup/config/semantic_locations.office.yaml
"""

import math
import os
import threading
import time
from typing import Any, Dict, Optional

import yaml
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node

from social_nav_bringup.nav2_navigator import Nav2Navigator


def get_default_locations_file() -> str:
    """패키지 기본 office semantic location 파일 경로를 반환합니다."""
    try:
        return os.path.join(
            get_package_share_directory("social_nav_bringup"),
            "config",
            "semantic_locations.office.yaml",
        )
    except PackageNotFoundError:
        return ""


def resolve_locations_file(file_path: str) -> str:
    """명시 경로가 없으면 패키지 기본 config 경로를 사용합니다."""
    if file_path:
        return file_path
    return get_default_locations_file()


def load_semantic_locations(file_path: str) -> Dict[str, Dict[str, float]]:
    """YAML 파일에서 Semantic Location을 로드합니다.

    예상 YAML 형식:
      locations:
        meeting_room:
          x: 1.0
          y: 2.0
          yaw: 1.57
          description: "회의실"
    """
    try:
        with open(file_path, "r") as f:
            data = yaml.safe_load(f)
        locations = data.get("locations", {})
        return {
            name: {"x": float(loc["x"]), "y": float(loc["y"]), "yaw": float(loc.get("yaw", 0.0))}
            for name, loc in locations.items()
        }
    except Exception as e:
        raise RuntimeError(f"Semantic Location 파일 로드 실패: {file_path} — {e}")


def get_stuck_params(node: Node) -> Dict[str, Any]:
    """go_to()와 patrol()이 공유하는 stuck 감지 파라미터를 읽습니다."""
    return {
        "action_timeout_sec": node.get_parameter("action_timeout_sec").value,
        "hard_stuck_timeout_sec": node.get_parameter("hard_stuck_timeout_sec").value,
        "progress_grace_sec": node.get_parameter("progress_grace_sec").value,
        "progress_epsilon_m": node.get_parameter("progress_epsilon_m").value,
        "near_goal_epsilon_m": node.get_parameter("near_goal_epsilon_m").value,
    }


def feedback_pose_distance(nav_feedback: Dict[str, Any], goal: Dict[str, float]) -> Optional[float]:
    """Nav2 feedback current_pose와 goal 사이의 2D 거리입니다."""
    try:
        current_x = nav_feedback.get("current_x")
        current_y = nav_feedback.get("current_y")
        if current_x is None or current_y is None:
            return None
        return math.hypot(float(current_x) - float(goal["x"]), float(current_y) - float(goal["y"]))
    except (KeyError, TypeError, ValueError):
        return None


def exec_go_to(
    node: Node,
    navigator: Nav2Navigator,
    nav_feedback: Dict[str, Any],
    goal: Dict[str, float],
    stuck_params: Optional[Dict[str, Any]] = None,
) -> bool:
    """go_to() 단위 액션 실행 로직.

    Args:
        node: ROS2 노드
        navigator: Nav2Navigator 인스턴스
        nav_feedback: feedback_cb에서 업데이트하는 dict
        goal: {"x": float, "y": float, "yaw": float}
        stuck_params: stuck 감지 파라미터 (선택)

    Returns:
        True(도착) / False(실패)
    """
    params = dict(stuck_params or {})
    action_timeout_sec = float(params.get("action_timeout_sec", 180.0))
    hard_stuck_timeout = float(params.get("hard_stuck_timeout_sec", 60.0))
    grace_sec = float(params.get("progress_grace_sec", 50.0))
    eps_m = float(params.get("progress_epsilon_m", 0.05))
    near_goal_eps = float(params.get("near_goal_epsilon_m", 1.0))

    node.get_logger().info(
        f"[go_to] 시작: goal=({goal['x']:.2f}, {goal['y']:.2f}, yaw={goal.get('yaw', 0):.2f}), "
        f"timeout={action_timeout_sec}s, stuck: hard={hard_stuck_timeout}s, "
        f"grace={grace_sec}s, eps={eps_m}m"
    )

    # Nav2 Goal 전송
    goal_handle, result_future = navigator.start(goal)
    if not goal_handle or not result_future:
        node.get_logger().error("[go_to] Nav2 Goal 전송 실패")
        return False

    # 진행 상황 추적 변수
    accept_t = time.time()
    last_prog = accept_t
    best_dist = float("inf")
    last_log = 0.0

    while rclpy.ok():
        time.sleep(0.2)
        now = time.time()

        # 주기적 로그 (5초마다)
        if now - last_log > 5.0:
            dist_dbg = nav_feedback.get("distance_remaining")
            node.get_logger().info(
                f"[go_to] distance_remaining={dist_dbg if dist_dbg is not None else 'N/A'}, "
                f"best={best_dist:.2f}m, elapsed={now - accept_t:.0f}s"
            )
            last_log = now

        # feedback 유무와 무관한 전체 액션 타임아웃
        if (now - accept_t) > action_timeout_sec:
            node.get_logger().warn(
                f"[go_to] Action timeout ({action_timeout_sec}s 초과) → Goal 취소"
            )
            navigator.cancel(goal_handle)
            return False

        # ---- Stuck 감지 ----
        dist = nav_feedback.get("distance_remaining")
        if dist is not None:
            # best_dist 갱신 (eps_m 이상 전진해야 "진전"으로 인정)
            if dist < (best_dist - eps_m):
                best_dist = dist
                last_prog = now

            # 목표 근처(near_goal_eps 이내)에서는 stuck 판정 제외
            if dist >= near_goal_eps:
                # grace 기간 이후, hard_stuck_timeout 동안 진전 없으면 실패
                if (now - accept_t) > grace_sec and (now - last_prog) > hard_stuck_timeout:
                    node.get_logger().warn(
                        f"[go_to] Stuck 감지! dist={dist:.2f}m, best={best_dist:.2f}m, "
                        f"정체 시간={now - last_prog:.0f}s → Goal 취소"
                    )
                    navigator.cancel(goal_handle)
                    return False

        # ---- Nav2 결과 확인 ----
        if result_future.done():
            try:
                res = result_future.result()
            except Exception as e:
                node.get_logger().warn(f"[go_to] result 예외: {e}")
                return False

            status = getattr(res, "status", None)
            ok = bool(status == GoalStatus.STATUS_SUCCEEDED)
            pose_dist = feedback_pose_distance(nav_feedback, goal)
            if not ok and pose_dist is not None and pose_dist <= near_goal_eps:
                node.get_logger().warn(
                    f"[go_to] Nav2 status={status}이나 현재 pose가 목표 근처입니다 "
                    f"(pose_dist={pose_dist:.2f}m <= {near_goal_eps:.2f}m) → 성공 처리"
                )
                ok = True
            elif not ok and pose_dist is None and best_dist <= near_goal_eps:
                node.get_logger().warn(
                    f"[go_to] Nav2 status={status}이나 feedback상 목표 근처입니다 "
                    f"(best_dist={best_dist:.2f}m <= {near_goal_eps:.2f}m) → 성공 처리"
                )
                ok = True
            if ok:
                node.get_logger().info(
                    f"[go_to] 성공! 소요 시간: {now - accept_t:.1f}s"
                )
            else:
                node.get_logger().warn(f"[go_to] 실패, status={status}")
            return ok

    node.get_logger().warn("[go_to] rclpy 종료 → 실패")
    return False


class GoToNode(Node):
    """go_to() Unit Action 실행 노드.

    파라미터:
      - target: Semantic Location 이름 (예: "meeting_room")
      - locations_file: semantic_locations.office.yaml 경로
      - target_x, target_y, target_yaw: 직접 좌표 지정 (target보다 우선)
      - action_timeout_sec: 전체 액션 타임아웃 (기본 180초)
      - hard_stuck_timeout_sec: stuck 판정 임계 (기본 60초)
      - progress_grace_sec: 초기 grace 기간 (기본 50초)
      - progress_epsilon_m: 진전 판정 최소 거리 (기본 0.05m)
      - near_goal_epsilon_m: 목표 근처 stuck 면제 거리 (기본 1.0m)
    """

    def __init__(self):
        super().__init__("go_to_node")

        # 파라미터 선언
        self.declare_parameter("target", "")
        self.declare_parameter("locations_file", "")
        self.declare_parameter("target_x", float("nan"))
        self.declare_parameter("target_y", float("nan"))
        self.declare_parameter("target_yaw", 0.0)
        self.declare_parameter("action_timeout_sec", 180.0)
        self.declare_parameter("hard_stuck_timeout_sec", 60.0)
        self.declare_parameter("progress_grace_sec", 50.0)
        self.declare_parameter("progress_epsilon_m", 0.05)
        self.declare_parameter("near_goal_epsilon_m", 1.0)
        self._done = threading.Event()
        self._success: Optional[bool] = None

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
        """Goal 해석 → exec_go_to 실행 → 결과 로그."""
        try:
            # 파라미터 가져오기
            target = self.get_parameter("target").value
            locations_file = resolve_locations_file(
                self.get_parameter("locations_file").value
            )
            target_x = self.get_parameter("target_x").value
            target_y = self.get_parameter("target_y").value
            target_yaw = self.get_parameter("target_yaw").value

            stuck_params = get_stuck_params(self)

            # Goal 결정: 직접 좌표 vs Semantic Location
            if not math.isnan(target_x) and not math.isnan(target_y):
                goal = {"x": target_x, "y": target_y, "yaw": target_yaw}
                self.get_logger().info(f"[GoToNode] 직접 좌표 사용: {goal}")
            elif target:
                if not locations_file:
                    self.get_logger().error(
                        "[GoToNode] semantic_locations.office.yaml을 찾지 못했습니다. "
                        "locations_file 파라미터를 명시하세요."
                    )
                    return
                try:
                    locations = load_semantic_locations(locations_file)
                except Exception as e:
                    self.get_logger().error(f"[GoToNode] {e}")
                    return
                if target not in locations:
                    self.get_logger().error(
                        f"[GoToNode] '{target}'이(가) locations에 없습니다. "
                        f"사용 가능: {list(locations.keys())}"
                    )
                    return
                goal = locations[target]
                self.get_logger().info(
                    f"[GoToNode] Semantic: '{target}' → {goal} "
                    f"(file={locations_file})"
                )
            else:
                self.get_logger().error(
                    "[GoToNode] target 또는 target_x/target_y를 지정하세요"
                )
                return

            # 실행
            self._success = exec_go_to(
                self, self._navigator, self._nav_feedback, goal, stuck_params
            )

            if self._success:
                self.get_logger().info("[GoToNode] === go_to() 완료: SUCCESS ===")
            else:
                self.get_logger().warn("[GoToNode] === go_to() 완료: FAILED ===")
        finally:
            self._done.set()

    def done(self) -> bool:
        """워커 스레드 완료 여부를 반환합니다."""
        return self._done.is_set()


def main():
    rclpy.init()
    node = GoToNode()
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
