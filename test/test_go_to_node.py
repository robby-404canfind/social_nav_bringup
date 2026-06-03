import unittest

from social_nav_bringup.go_to_node import feedback_pose_distance


class GoToNodeTest(unittest.TestCase):
    def test_feedback_pose_distance_uses_current_pose(self):
        dist = feedback_pose_distance(
            {"current_x": 2.9, "current_y": 4.9},
            {"x": 3.0, "y": 5.0},
        )

        self.assertLess(dist, 0.15)

    def test_feedback_pose_distance_missing_pose_returns_none(self):
        self.assertIsNone(
            feedback_pose_distance(
                {"distance_remaining": 0.0},
                {"x": 3.0, "y": 5.0},
            )
        )


if __name__ == "__main__":
    unittest.main()
