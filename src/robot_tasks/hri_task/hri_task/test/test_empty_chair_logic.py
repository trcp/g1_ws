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

from yolo_states import YoloEmptyChairState


def make_state():
    state = YoloEmptyChairState.__new__(YoloEmptyChairState)
    state.expected_seat_count = 3
    state.image_width = 640
    return state


class EmptyChairLogicTest(unittest.TestCase):

    def test_middle_slot_is_empty_when_people_are_on_both_sides(self):
        state = make_state()
        sofa = {'label': 'sofa', 'confidence': 0.9, 'bbox': [110, 220, 530, 420]}
        people = [
            {'label': 'person', 'confidence': 0.95, 'bbox': [95, 130, 210, 450], 'distance_z': 2.0},
            {'label': 'person', 'confidence': 0.95, 'bbox': [430, 130, 545, 450], 'distance_z': 2.1},
        ]

        foreground, background = state._filter_foreground_people(people)
        seats = state._build_seat_candidates([sofa], 640, foreground)
        state._assign_people_to_seats(foreground, seats)

        self.assertEqual(len(background), 0)
        self.assertEqual(len(seats), 3)
        self.assertTrue(seats[0]['occupied'])
        self.assertFalse(seats[1]['occupied'])
        self.assertTrue(seats[2]['occupied'])
        self.assertEqual(state._select_empty_seat([s for s in seats if not s['occupied']])['index'], 2)
        self.assertLess(seats[0]['center_x'], seats[1]['center_x'])
        self.assertLess(seats[1]['center_x'], seats[2]['center_x'])

    def test_hidden_chairs_are_inferred_from_people_and_partial_chair(self):
        state = make_state()
        chair = {'label': 'chair', 'confidence': 0.8, 'bbox': [260, 250, 380, 430]}
        people = [
            {'label': 'person', 'confidence': 0.95, 'bbox': [90, 120, 220, 455], 'distance_z': 2.2},
            {'label': 'person', 'confidence': 0.95, 'bbox': [420, 125, 550, 455], 'distance_z': 2.2},
        ]

        foreground, _ = state._filter_foreground_people(people)
        seats = state._build_seat_candidates([chair], 640, foreground)
        state._assign_people_to_seats(foreground, seats)

        self.assertEqual(len(seats), 3)
        self.assertTrue(all(s['inferred'] for s in seats))
        self.assertTrue(seats[0]['occupied'])
        self.assertFalse(seats[1]['occupied'])
        self.assertTrue(seats[2]['occupied'])

    def test_background_people_are_ignored(self):
        state = make_state()
        people = [
            {'label': 'person', 'confidence': 0.95, 'bbox': [235, 120, 345, 455], 'distance_z': 2.0},
            {'label': 'person', 'confidence': 0.90, 'bbox': [40, 80, 100, 230], 'distance_z': 5.5},
            {'label': 'person', 'confidence': 0.90, 'bbox': [510, 75, 570, 225], 'distance_z': 5.8},
        ]

        foreground, background = state._filter_foreground_people(people)

        self.assertEqual(len(foreground), 1)
        self.assertEqual(len(background), 2)

    def test_split_slots_have_individual_centers(self):
        state = make_state()
        sofa = {'label': 'sofa', 'confidence': 0.9, 'bbox': [100, 220, 550, 420]}

        seats = state._build_seat_candidates([sofa], 640, [])
        centers = [s['center_x'] for s in seats]

        self.assertEqual(len(seats), 3)
        self.assertEqual(len(set(centers)), 3)
        self.assertEqual([s['index'] for s in seats], [1, 2, 3])


if __name__ == '__main__':
    unittest.main()
