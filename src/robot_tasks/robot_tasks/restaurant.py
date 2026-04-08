"""
レストランタスク
"""

#!/usr/bin/env python3

# ROS2
from rclpy.node import Node
import rclpy
import smach

# eraasers g1 APIs
from erasers_g1_api.tts import TTS
from erasers_g1_api.robot_control import G1Navigation

# preferences
import traceback


"""
周囲のマップを作成
"""
@smach.cb_interface(outcomes=['success', 'failure'],
                    input_keys=[],
                    output_keys=[])
def cb_state_create_around_map(userdata, node:Node, tts_say:TTS.say, navigation:G1Navigation):
    try:
        tts_say("I will create aound map. Please wait a moment.")

        navigation.move_rel(y=0.25, yaw=1.57)
        navigation.move_rel(x=-0.25, yaw=1.57)
        navigation.move_rel(y=0.25, yaw=1.57)
        navigation.move_rel(x=-0.25, yaw=1.57, tolerance=0.0)

        tts_say("I created aound map!")
        return 'success'
    except Exception as e:
        node.get_logger().error(f"Error in grasp_bag: {e}\n{traceback.format_exc()}")
        return 'failure'


"""
main
"""
def main():
    # Init ROS2
    rclpy.init()
    node = Node("restaurant_task")

    # init APIs
    tts = TTS(node)
    SAY = tts.say
    NAVIGATION = G1Navigation(node)

    # init smach
    sm = smach.StateMachine(outcomes=['success', 'failure'])


    with sm:
        smach.StateMachine.add('CREATE_AROUND_MAP', smach.CBState(cb=cb_state_create_around_map,
                                                                  cb_kwargs={'node': node, 'tts_say': SAY, 'navigation': NAVIGATION},),
                                transitions={'success': 'success', 
                                             'failure': 'failure'})
    
    # execute smach states
    outcome = sm.execute()
    if outcome == 'success':
        SAY("I finished the task.")
    else:
        SAY("I failed to finish the task.")
    node.destroy_node()