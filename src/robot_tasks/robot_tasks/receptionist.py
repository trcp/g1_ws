"""
Receptionist タスク
"""

#!/usr/bin/env python3

# ROS2
from rclpy.node import Node
import rclpy
import smach

# interfaces
from lor_interfaces.msg import Person3D, Persons3D # Light Weight Open Pose

# eraasers g1 APIs
from erasers_g1_api.tts import TTS
from erasers_g1_api.robot_control import G1Navigation, G1Control, ArmControl, Collision, Grasp
from erasers_g1_api.state_skills.recongnition import SpeechToText, LOR, Sam3ObjectDetector
from erasers_g1_api.state_skills.grasping import object_grasping

# preferences
from ament_index_python.packages import get_package_share_directory
from typing import List
import traceback
import yaml
import os


"""
パラメータを読み込む
"""
def load_params(node:Node, params_file:str):
    node.get_logger().info("Get tasks parameter from: %s"%params_file)
    try:
        with open(params_file, 'r') as f:
            params = yaml.safe_load(f)
            locations:dict = params['receptionist_task']['ros_parameters']['locations']
            locations_str = "\n".join(locations.keys())
            node.get_logger().info(f"Locations list:\n{locations_str}")
            return locations
    except Exception as e:
        node.get_logger().error(f"Failed to load params: {e}")
        node.get_logger().error(traceback.format_exc())


"""
指定されたロケーションに移動する
"""
@smach.cb_interface(outcomes=['success', 'timeout', 'failure'],
                    input_keys=['locations'],
                    output_keys=[])
def move_to_location_cb(userdata, node:Node, tts_say:TTS.say, navigation:G1Navigation, control:G1Control, location:str):
    try:
        locations = userdata.locations
        control.pose_policy('running')
        tts_say(f"go to the {location}.")
        navigation.move_abs(locations[location][0], locations[location][1], locations[location][2])
        control.pose_policy('start')
        return 'success'

    except Exception as e:
        node.get_logger().error(f"Error in move_to_location_cb: {e}\n{traceback.format_exc()}")
        return 'failure'


"""
main
"""
def main():
    # init ROS
    rclpy.init()
    node = Node('receptionist_task')

    # ロケーションリストを取得
    default_params_file_path = os.path.join(get_package_share_directory('robot_tasks'), 'params', 'receptionist_task.yaml')
    node.declare_parameter('task_params', default_params_file_path)
    locations = load_params(node, node.get_parameter('task_params').value)

    # init API
    tts = TTS(node)
    SAY = tts.say
    CONROL = G1Control(node)
    NAVIGATION = G1Navigation(node)

    # init smach
    sm = smach.StateMachine(outcomes=['success', 'failure'])

    # userdata
    sm.userdata.locations = locations

    SAY('receptionist task start!')

    
    with sm:
        smach.StateMachine.add('MOVE_TO_ENTRANCE', smach.CBState(cb=move_to_location_cb,
                                                                 cb_kwargs={'node': node, 'tts_say': SAY, 'navigation': NAVIGATION, 'control': CONROL,
                                                                            'location': 'entrance'}),
                               transitions={'success': 'success',
                                            'timeout': 'failure',
                                            'failure': 'failure'})
    
    # execute smach states
    outcome = sm.execute()
    if outcome == 'success':
        SAY("I finished the task.")
    else:
        SAY("I failed to finish the task.")
    node.destroy_node()