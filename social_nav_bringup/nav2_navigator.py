#!/usr/bin/env python3
"""Nav2 NavigateToPose Action Client 래퍼.

Semantic Location 또는 직접 좌표로 만든 goal dict를 Nav2 NavigateToPose
Goal 메시지로 변환하고, feedback을 호출자에게 전달합니다.
"""

import math
import time
from typing import Any, Callable, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose


class Nav2Navigator:
    """Nav2 NavigateToPose 액션에 대한 래퍼.

    go_to_node와 patrol_node는 메인 스레드에서 rclpy.spin_once()를 반복하고,
    워커 스레드에서 goal 전송과 결과 확인을 수행합니다. 여기서는
    spin_until_future_complete 대신 짧은 polling으로 future 완료를 기다립니다.
    """

    def __init__(
        self,
        node: Node,
        feedback_cb: Optional[Callable[[Dict[str, Any]], None]] = None,
        action_name: str = "navigate_to_pose",
        frame_id: str = "map",
    ):
        self._node = node
        self._feedback_cb = feedback_cb
        self._action_name = action_name
        self._frame_id = frame_id

        self._client = ActionClient(node, NavigateToPose, self._action_name)
        self._last_goal_handle = None

        self._node.get_logger().info(
            f"[Nav2Navigator] action='{self._action_name}', frame='{self._frame_id}'"
        )

    # ---- yaw(rad) -> Quaternion 변환 ----
    @staticmethod
    def yaw_to_quat(yaw: float) -> Quaternion:
        """2D yaw 각도(라디안)를 Quaternion으로 변환합니다."""
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    # ---- 내부 feedback 핸들러 ----
    def _on_feedback(self, feedback_msg):
        try:
            fb = feedback_msg.feedback
            data = {
                "distance_remaining": getattr(fb, "distance_remaining", None),
                "stamp": time.time(),
            }
            if self._feedback_cb:
                self._feedback_cb(data)
        except Exception as e:
            self._node.get_logger().warn(f"[Nav2Navigator] feedback 예외: {e}")

    # ---- Goal 전송 ----
    def start(self, goal: Dict[str, Any]):
        """goal: {"x": float, "y": float, "yaw": float}

        반환: (goal_handle, result_future) 또는 (None, None)
        """
        if not isinstance(goal, dict):
            self._node.get_logger().error("[Nav2Navigator] goal은 dict여야 합니다")
            return None, None

        try:
            x = float(goal["x"])
            y = float(goal["y"])
            yaw = float(goal.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError) as e:
            self._node.get_logger().error(f"[Nav2Navigator] goal 파싱 실패: {e}")
            return None, None

        # Action Server 준비 대기 (polling, 최대 10초)
        t0 = time.time()
        while not self._client.wait_for_server(timeout_sec=0.5):
            if not rclpy.ok():
                return None, None
            if time.time() - t0 > 10.0:
                self._node.get_logger().error(
                    f"[Nav2Navigator] '{self._action_name}' 서버 연결 실패 (10초 초과)"
                )
                return None, None

        # Goal 메시지 생성
        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.header.frame_id = self._frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = self.yaw_to_quat(yaw)

        self._node.get_logger().info(
            f"[Nav2Navigator] Goal 전송: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

        # 비동기 Goal 전송
        send_goal_future = self._client.send_goal_async(
            msg, feedback_callback=self._on_feedback
        )

        # polling으로 전송 완료 대기
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.01)

        if not rclpy.ok():
            return None, None

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self._node.get_logger().warn("[Nav2Navigator] Goal이 거부되었습니다")
            return None, None

        self._last_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        self._node.get_logger().info("[Nav2Navigator] Goal 수락됨")
        return goal_handle, result_future

    # ---- Goal 취소 ----
    def cancel(self, goal_handle):
        """실행 중인 Goal을 취소합니다."""
        if goal_handle is None:
            return
        try:
            self._node.get_logger().info("[Nav2Navigator] Goal 취소 요청")
            fut = goal_handle.cancel_goal_async()
            t0 = time.time()
            while rclpy.ok() and not fut.done() and (time.time() - t0) < 3.0:
                time.sleep(0.01)
        except Exception as e:
            self._node.get_logger().warn(f"[Nav2Navigator] 취소 실패: {e}")

    def cancel_all(self):
        """마지막으로 전송한 Goal을 취소합니다."""
        if self._last_goal_handle:
            self.cancel(self._last_goal_handle)
