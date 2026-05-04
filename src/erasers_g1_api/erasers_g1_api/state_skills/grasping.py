import smach
from rclpy.node import Node
from erasers_g1_api.robot_control import G1Control, Grasp, Collision
from erasers_g1_api.tts import TTS

import traceback
import math

@smach.cb_interface(
    outcomes=['success', 'failure'],
    input_keys=['object_poses_dict_list']
)
def object_grasping(userdata, node: Node, arm_control: Grasp, tts_say: TTS.say, approach: str = 'side'):
    """
    認識した物体のリストから最初の物体を両腕で把持するステート。

    Parameters
    ----------
    userdata : smach.user_data.Remapper
        状態間で共有されるデータ。
    node : Node
        ROS 2ノード。
    arm_control : Grasp
        腕の制御インスタンス。
    tts_say : TTS.say
        発話関数。
    approach : str, optional
        把持のアプローチ方向 ('side' または 'top'), by default 'side'

    Userdata
    --------
    Input Keys:
        object_poses_dict_list : list of dict
            検出された物体の情報リスト。

    Outcomes
    --------
    success : str
        物体把持に成功した場合。
    failure : str
        物体リストが空、あるいは把持に失敗した場合。
    """
    try:
        if not userdata.object_poses_dict_list:
            node.get_logger().warn("No objects to grasp.")
            return 'failure'

        target_obj = userdata.object_poses_dict_list[0]
        name = target_obj['name']
        xyz = target_obj['pose']['xyz']
        rpy = target_obj['pose']['rpy']
        size = target_obj['size']
        y_val = xyz[1]

        tts_say(f"I will grasp the {name}.")

        # プランニングシーンにオブジェクトを追加
        arm_control.collision.add_box(name, "base_link", xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2], size)
        
        # 把持の実行（新メソッド grasp() は自動的に最適な腕を選択します）
        node.get_logger().info(f"Initiating dynamic grasp for '{name}'")
        success = arm_control.grasp(target_name=name)

        if success:
            tts_say("Grasping finished.")
            return 'success'
        else:
            tts_say("Failed to grasp the object with all strategies.")
            return 'failure'
            
    except Exception as e:
        node.get_logger().error(f"Error in object_grasping: {e}")
        traceback.print_exc()
        tts_say("An error occurred while grasping.")
        return 'failure'
