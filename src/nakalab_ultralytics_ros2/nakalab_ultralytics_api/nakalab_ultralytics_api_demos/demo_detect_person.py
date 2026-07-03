#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

import smach

from nakalab_ultralytics_api.nu_api import PersonDetectorState


def main():
    rclpy.init()
    node = Node('demo_person_detect')

    try:
        sm = smach.StateMachine(outcomes=['success', 'failure'])
        sm.userdata.person_poses = []

        with sm:
            smach.StateMachine.add(
                'PERSON_DETECT',
                PersonDetectorState(
                    node=node,
                    timeout_sec=10.0,
                    scan_time_sec=5.0,
                    confedence=0.5,
                    condition='hand_up',
                ),
                transitions={
                    'success': 'success',
                    'failure': 'failure',
                    'timeout': 'PERSON_DETECT',
                },
                remapping={
                    'person_poses': 'person_poses',
                },
            )

        outcome = sm.execute()
        node.get_logger().info(
            'demo_detect_person outcome: %s' % outcome
        )
        raise SystemExit(0 if outcome == 'success' else 1)
    except Exception as err:
        node.get_logger().error(str(err))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
