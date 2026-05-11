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
from erasers_g1_api.state_skills.gemini import GeminiVLMState
from erasers_g1_api.state_skills.grasping import object_grasping

# preferences
from ament_index_python.packages import get_package_share_directory
from typing import List
import traceback
import json
import yaml
import os


DEBUG = False


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
                    input_keys=['locations', 'guest_info'],
                    output_keys=[])
def move_to_location_cb(userdata, node:Node, tts_say:TTS.say, navigation:G1Navigation, control:G1Control, location:str, message:str=None):
    try:
        locations = userdata.locations
        control.pose_policy('running')
        control.move_head(tilt=-0.5)
        if message is None:
            tts_say(f"go to the {location}.")
        else:
            tts_say(message)
        navigation.move_abs(locations[location][0], locations[location][1], locations[location][2])
        control.pose_policy('start')
        control.move_head(tilt=0.0)
        return 'success'

    except Exception as e:
        node.get_logger().error(f"Error in move_to_location_cb: {e}\n{traceback.format_exc()}")
        return 'failure'


"""
GUEST 対応
"""
def respond_guest(node: Node, tts_say:TTS.say, arm_control:ArmControl)-> smach.StateMachine:

    @smach.cb_interface(outcomes=['have_bag', 'no_bag', 'failure'],
                            input_keys=['result'],
                            output_keys=['guest_info'])
    def gemini_result_processing_cb(userdata, node:Node):
        try:
            result = json.loads(userdata.result)
            userdata.guest_info.append(result)
            tts_say('Hi! %s. You like a %s.'%(result['person_name'], result['favorite_drink']))
            if result['bag']:
                node.get_logger().info('Guest have a bag.')
                return 'have_bag'
            return 'no_bag'

        except Exception as e:
            node.get_logger().error(f"Error in move_to_location_cb: {e}\n{traceback.format_exc()}")
            return 'failure'
    

    @smach.cb_interface(outcomes=['success', 'failure'],
                            input_keys=[],
                            output_keys=[])
    def ask_grasp_bag(userdata, node:Node, tts_say:TTS.say, arm_control:ArmControl):
        try:
            tts_say('It looks like you have a bag with you. Let me carry it for you.')
            return 'success'
        except Exception as e:
            node.get_logger().error(f"Error in move_to_location_cb: {e}\n{traceback.format_exc()}")
            return 'failure'
        
            
    sm = smach.StateMachine(outcomes=['success', 'failure'],
                            input_keys=['prompt_message', 'stt_text'])


    with sm:
        smach.StateMachine.add('ASK_GEMINI', GeminiVLMState(node=node),
                               transitions={'success': 'GEMINI_RESULT_PROCESSING',
                                            'failure': 'failure'},
                                remapping={'ud_prompt': 'stt_text'})
        
        smach.StateMachine.add('GEMINI_RESULT_PROCESSING', smach.CBState(cb=gemini_result_processing_cb,
                                                                         cb_kwargs={'node': node}),
                                transitions={'have_bag': 'GRASP_BAG',
                                             'no_bag': 'success',
                                            'failure': 'failure'})
        
        smach.StateMachine.add('GRASP_BAG', smach.CBState(cb=ask_grasp_bag,
                                                                         cb_kwargs={'node': node, 'tts_say':tts_say, 'arm_control':arm_control}),
                                transitions={'success': 'success',
                                            'failure': 'failure'})
        
    
    return sm


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
    tts = TTS(node, debug=DEBUG)
    SAY = tts.say
    ARM = ArmControl(node)
    CONTROL = G1Control(node)
    NAVIGATION = G1Navigation(node)

    # init smach
    sm = smach.StateMachine(outcomes=['success', 'failure'])

    # userdata
    sm.userdata.locations = locations
    sm.userdata.num_challenge = 3
    sm.userdata.success_keywards = []
    sm.userdata.guest_info = []
    sm.userdata.person_prompt_message = '''
    あなたはパーティーホストロボットで，ゲストを迎え入れたり，ゲストを案内する役割を持ちます．
    画像に写っている人物はかばんを持っているか？ソファに人は座っていますか？
    以下のフォーマットに準拠し結果を出力してください。。
    以下のフォーマットはコードブロックで囲わず、そのままテキストとして出力すること。
    ```
    {
        "bag": true, # or false
        "person_name": "", # ここに人の名前を入力してください．
        "favorite_drink": "", # ここに人の好きな飲み物を入力してください．．
        "sofa_empty": false # or true (If there is someone on the sofa, then it's false.)
    }
    ```
    また次のテキストにゲストの名前，ゲストの好きな飲み物が含まれる文字列があります．これも解析して上記の JSON に適切に記述してください．
    '''

    SAY('receptionist task start!')

    
    with sm:
        smach.StateMachine.add('MOVE_TO_ENTRANCE', smach.CBState(cb=move_to_location_cb,
                                                                 cb_kwargs={'node': node, 'tts_say': SAY, 'navigation': NAVIGATION, 'control': CONTROL,
                                                                            'location': 'entrance'}),
                               transitions={'success': 'success',
                                            'timeout': 'failure',
                                            'failure': 'failure'})
        
        smach.StateMachine.add('FIND_GUEST', LOR(node=node,
                                                tts_say=SAY,
                                                robot_control=CONTROL,
                                                success_msg='I found a guest.',
                                                timeout_msg='Sorry. I can not found the person.'),
                               transitions={'success': 'GET_GUEST_INFO',
                                            'timeout': 'failure',
                                            'failure': 'failure'})

        smach.StateMachine.add('GET_GUEST_INFO', SpeechToText(node=node,
                                                              tts=tts,
                                                              start_msg='Hi. welcome to our party. I am host robot. Please tell me your name and favorite. When you hear the beep, please speak loudly.',
                                                              success_msg='OK.'),
                                transitions={'success': 'RESPOND_GUEST',
                                            'timeout': 'failure',
                                            'failure': 'failure'})
        
        smach.StateMachine.add('RESPOND_GUEST', respond_guest(node=node, tts_say=SAY, arm_control=ARM),
                               transitions={'success': 'success',
                                            'failure': 'failure'},
                                remapping={'prompt_message': 'person_prompt_message'})
        
        smach.StateMachine.add('MOVE_TO_LIVING', smach.CBState(cb=move_to_location_cb,
                                                                 cb_kwargs={'node': node, 'tts_say': SAY, 'navigation': NAVIGATION, 'control': CONTROL,
                                                                            'location': 'living_room'}),
                               transitions={'success': 'success',
                                            'timeout': 'failure',
                                            'failure': 'failure'})
        smach.StateMachine.add('SOFA_CHECK', respond_guest(node=node, tts_say=SAY, arm_control=ARM),
                               transitions={'success': 'success',
                                            'failure': 'failure'},
                                remapping={'prompt_message': 'person_prompt_message'})
    
    # execute smach states
    outcome = sm.execute()
    if outcome == 'success':
        SAY("I finished the task.")
    else:
        SAY("I failed to finish the task.")
    node.destroy_node()
