#!/usr/bin/env python3
import os
import sys
import types
import unittest

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
HRI_DIR = os.path.dirname(TEST_DIR)
sys.path.append(HRI_DIR)

sys.modules.setdefault('rclpy', types.SimpleNamespace(ok=lambda: True, spin_once=lambda *a, **k: None))
sys.modules.setdefault('rclpy.node', types.SimpleNamespace(Node=object))
sys.modules.setdefault('rclpy.executors', types.SimpleNamespace(SingleThreadedExecutor=object))
sys.modules.setdefault('smach', types.SimpleNamespace(State=object))
sys.modules.setdefault('std_msgs.msg', types.SimpleNamespace(String=object))
sys.modules.setdefault('geometry_msgs.msg', types.SimpleNamespace(Twist=object))
sys.modules.setdefault(
    'direct_joint_control',
    types.SimpleNamespace(ARM_POSE_EXTEND_LEFT={}, ARM_POSE_EXTEND_RIGHT={}, HOME_POSE={})
)
sys.modules.setdefault('bag_grasp_ik', types.SimpleNamespace(calculate_bag_grasp_joints=lambda *a, **k: {}))
sys.modules.setdefault('yolo_track', types.SimpleNamespace(YoloHumanTracker=object))

from yolo_states import YoloTrackingState


def make_state():
    state = YoloTrackingState.__new__(YoloTrackingState)
    state.distance_threshold = 1.05
    state.ideal_distance = 0.7
    state.center_threshold = 0.45
    state.stationary_angle_threshold = 0.08
    state.stationary_depth_threshold = 0.18
    return state


class TrackingLogicTest(unittest.TestCase):

    def test_candidate_score_prefers_near_center_guest(self):
        state = make_state()
        detections = [
            {'label': 'person', 'distance_z': 0.9, 'angle_rad': 0.85, 'bbox_width_ratio': 0.4},
            {'label': 'person', 'distance_z': 0.75, 'angle_rad': 0.08, 'bbox_width_ratio': 0.35},
            {'label': 'chair', 'distance_z': 1.0, 'angle_rad': 0.0},
        ]

        candidates = state._person_candidates(detections)

        self.assertEqual(len(candidates), 2)
        self.assertAlmostEqual(candidates[0]['z'], 0.75)
        self.assertTrue(state._is_good_guest_candidate(candidates[0]))

    def test_stationary_candidate_allows_small_motion(self):
        state = make_state()
        previous = {'z': 1.02, 'angle_rad': 0.10}
        current = {'z': 1.12, 'angle_rad': 0.15}

        self.assertTrue(state._is_stationary_candidate(current, previous))

    def test_good_candidate_rejects_far_or_off_center_person(self):
        state = make_state()

        self.assertFalse(state._is_good_guest_candidate({'z': 1.7, 'angle_rad': 0.1}))
        self.assertFalse(state._is_good_guest_candidate({'z': 1.0, 'angle_rad': 0.7}))


if __name__ == '__main__':
    unittest.main()
