"""
レストランタスク
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
from erasers_g1_api.robot_control import G1Navigation, G1Control
from erasers_g1_api.state_skills.recongnition import SpeechToText, LOR

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
            objects = params['restaurant_task']['ros_parameters']['objects']
            objects_str = "\n".join(objects.keys())
            node.get_logger().info(f"Objects list:\n{objects_str}")
            return [item for items in objects.values() for item in items]
    except Exception as e:
        node.get_logger().error(f"Failed to load params: {e}")
        node.get_logger().error(traceback.format_exc())



"""
周囲のマップを作成
"""
@smach.cb_interface(outcomes=['success', 'failure'],
                    input_keys=[],
                    output_keys=[])
def cb_state_create_around_map(userdata, node:Node, tts_say:TTS.say, navigation:G1Navigation, control:G1Control):
    try:
        control.pose_policy('running')
        tts_say("I will create aound map. Please wait a moment.")

        navigation.move_rel(yaw=1.57)
        navigation.move_rel(yaw=1.57)
        navigation.move_rel(yaw=1.57)
        navigation.move_rel(yaw=1.57)
        navigation.move_abs()

        tts_say("I created aound map!")
        control.pose_policy('start')
        return 'success'
    except Exception as e:
        node.get_logger().error(f"Error in grasp_bag: {e}\n{traceback.format_exc()}")
        return 'failure'


"""
お客さんまで移動
"""
@smach.cb_interface(outcomes=['success', 'failure'],
                    input_keys=['person_poses'],
                    output_keys=[])
def cb_state_move_to_customer(userdata, node:Node, tts_say:TTS.say, navigation:G1Navigation, control:G1Control):
    try:
        control.pose_policy('running')
        tts_say("I will move to the customer. Please wait a moment.")
        person_pose:Person3D = userdata.person_poses[0]
        node.get_logger().info('PERSON POSE')
        node.get_logger().info('x %f'%(person_pose.pose.position.x))
        node.get_logger().info('y %f'%(person_pose.pose.position.y))
        navigation.move_rel(x=person_pose.pose.position.x, y=person_pose.pose.position.y, yaw=0.0, tolerance=1.2)
        tts_say("I reached the customer!")
        control.pose_policy('start')
        return 'success'

    except Exception as e:
        node.get_logger().error(f"Error in grasp_bag: {e}\n{traceback.format_exc()}")
        return 'failure'


"""
お客さんの注文を確認する
"""
@smach.cb_interface(outcomes=['success', 'timeout', 'failure'],
                    input_keys=['object_keywards', 'stt_text'],
                    output_keys=['order_list'])
def cb_state_check_order(userdata, node:Node, tts_say:TTS.say):
    try:
        voice_message = userdata.stt_text
        order_list = [kw for kw in userdata.object_keywards if kw in voice_message.lower()]
        userdata.order_list = order_list
        if order_list == []:
            tts_say('Sorry, I failed to understand your order. Please try again.')
            return 'timeout'
        userdata.object_keywards.extend(order_list)
        tts_say('You ordered %s.' % '，'.join(order_list))
        return 'success'
    except:
        node.get_logger().error('Error is occured in check_order\n%s'%traceback.format_exc())
        return 'failure'


"""
お客さんのからの確認結果
"""
@smach.cb_interface(outcomes=['apply', 'reject', 'failure'],
                    input_keys=['stt_text'],
                    output_keys=['stt_text'])
def cb_state_check_order_confirmation(userdata, node:Node, tts_say:TTS.say):
    try:
        voice_message = userdata.stt_text.lower()
        if 'yes' in voice_message and 'no' not in voice_message:
            tts_say('OK. I will go to the bar counter.')
            return 'apply'
        else:
            tts_say('OK. I will ask your order again.')
            userdata.order_list = []
            return 'reject'
        
    except:
        node.get_logger().error('Error is occured in check_order\n%s'%traceback.format_exc())
        return 'failure'


"""
バーカウンターへ戻る
"""
@smach.cb_interface(outcomes=['success', 'failure'],
                    input_keys=[],
                    output_keys=[])
def cb_state_move_to_bar_counter(userdata, node:Node, navigation:G1Navigation, control:G1Control):
    try:
        control.pose_policy('running')

        navigation.move_abs(yaw=1.57)

        control.pose_policy('start')
        return 'success'
    except Exception as e:
        node.get_logger().error(f"Error in grasp_bag: {e}\n{traceback.format_exc()}")
        return 'failure'


"""
お客さんの注文を報告する
"""
@smach.cb_interface(outcomes=['success', 'failure'],
                    input_keys=['order_list'],
                    output_keys=['order_list'])
def cb_state_order_report(userdata, node:Node, tts_say:TTS.say):
    try:
        tts_say('Barman, the customer ordered %s.' % '，'.join(userdata.order_list))
        tts_say("I will grasp there." if len(userdata.order_list) > 1 else "I will grasp it.")
        userdata.order_list = []
        return 'success'
    except:
        node.get_logger().error('Error is occured in check_order\n%s'%traceback.format_exc())
        return 'failure'


"""
main
"""
def main():
    # Init ROS2
    rclpy.init()
    node = Node("restaurant_task")

    # 注文品リストを取得する
    default_params_file_path = os.path.join(get_package_share_directory('robot_tasks'), 'params', 'restaurant_task.yaml')
    node.declare_parameter('task_params', default_params_file_path)
    object_keywards = load_params(node, node.get_parameter('task_params').value)


    # init APIs
    tts = TTS(node)
    CONROL = G1Control(node)
    SAY = tts.say
    NAVIGATION = G1Navigation(node)

    # init smach
    sm = smach.StateMachine(outcomes=['success', 'failure'])

    # userdatas
    sm.userdata.num_challenge = 0
    sm.userdata.stt_text = "" # 音声認識の結果
    sm.userdata.order_list = [] # 注文品リスト
    sm.userdata.object_keywards = object_keywards


    with sm:
        # smach.StateMachine.add('CREATE_AROUND_MAP', smach.CBState(cb=cb_state_create_around_map,
        #                                                           cb_kwargs={'node': node, 'tts_say': SAY, 'navigation': NAVIGATION, 'control': CONROL},),
        #                         transitions={'success': 'SEARCH_CUSTOMER', 
        #                                      'failure': 'failure'})
        smach.StateMachine.add('SEARCH_CUSTOMER', LOR(node=node,
                                                      tts_say=SAY,
                                                      start_msg='Hi customers! Please raise your hand if you want to order.',
                                                      timeout_msg="Sorry, I couldn't find any customers.",
                                                      success_msg="I found a customer!",
                                                      detect_condition='hand_up'),
                               transitions={'success': 'MOVE_TO_CUSTOMER', 
                                            'timeout': 'SEARCH_CUSTOMER',
                                            'failure': 'failure'})
        
        smach.StateMachine.add('MOVE_TO_CUSTOMER', smach.CBState(cb=cb_state_move_to_customer,
                                                                  cb_kwargs={'node': node, 'tts_say': SAY, 'navigation': NAVIGATION, 'control': CONROL},),
                                transitions={'success': 'REQUEST_ORDER', 
                                             'failure': 'failure'})
        
        smach.StateMachine.add('REQUEST_ORDER', SpeechToText(node=node,
                                                             tts_say=SAY,
                                                             start_msg="hi customer. What is your order ?",
                                                             success_msg="OK",
                                                             timeout_msg="Sorry, I didn't hear your order.",
                                                             success_keywards=object_keywards,
                                                             device="cpu",
                                                             lang="en"),
                                transitions={'success': 'REQUEST_CHECK',
                                             'timeout': 'REQUEST_ORDER',
                                             'failure': 'failure'})

        smach.StateMachine.add('REQUEST_CHECK', smach.CBState(cb=cb_state_check_order,
                                                                  cb_kwargs={'node': node, 'tts_say': SAY},),
                                transitions={'success': 'REQUEST_APPLY', 
                                             'timeout': 'REQUEST_ORDER',
                                             'failure': 'failure'})

        smach.StateMachine.add('REQUEST_APPLY', SpeechToText(node=node,
                                                             tts_say=SAY,
                                                             start_msg="Is this correct ? Please say yes or no.",
                                                             success_msg="OK",
                                                             timeout_msg="OK. I will go back to the bar counter.",
                                                             success_keywards=['yes', 'no'],
                                                             device="cpu",
                                                             lang="en"),
                                transitions={'success': 'ORDER_CONFIRMATION',
                                             'timeout': 'MOVE_TO_BARCOUNTER',
                                             'failure': 'failure'})

        smach.StateMachine.add('ORDER_CONFIRMATION', smach.CBState(cb=cb_state_check_order_confirmation,
                                                                  cb_kwargs={'node': node, 'tts_say': SAY},),
                                transitions={'apply': 'MOVE_TO_BARCOUNTER', 
                                             'reject': 'REQUEST_ORDER',
                                             'failure': 'failure'})
        
        smach.StateMachine.add('MOVE_TO_BARCOUNTER', smach.CBState(cb=cb_state_move_to_bar_counter,
                                                                  cb_kwargs={'node': node, 'navigation': NAVIGATION, 'control': CONROL},),
                                transitions={'success': 'ORDER_REPORT', 
                                             'failure': 'failure'})

        smach.StateMachine.add('ORDER_REPORT', smach.CBState(cb=cb_state_order_report,
                                                                  cb_kwargs={'node': node, 'tts_say': SAY},),
                                transitions={'success': 'success', 
                                             'failure': 'failure'})
    
    # execute smach states
    outcome = sm.execute()
    if outcome == 'success':
        SAY("I finished the task.")
    else:
        SAY("I failed to finish the task.")
    node.destroy_node()