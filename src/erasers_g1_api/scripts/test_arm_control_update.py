import rclpy
from rclpy.node import Node
from erasers_g1_api.robot_control import ArmControl
import time

def main():
    rclpy.init()
    node = Node("test_arm_control")
    arm = ArmControl(node)
    
    time.sleep(2) # Wait for TF
    
    print("--- Testing get_current_pose (arm_left) ---")
    pose = arm.get_current_pose(simple=True, planning_group='arm_left')
    print(f"Current Left Arm Pose: {pose}")
    
    print("\n--- Testing move_groupstate 'home' (arm_left) ---")
    success = arm.move_groupstate(group_name='arm_left', group_state='home')
    print(f"Home success: {success}")
    
    print("\n--- Testing move_dual_abs (Current Poses) ---")
    l_now = arm.get_current_pose(simple=True, planning_group='arm_left')
    r_now = arm.get_current_pose(simple=True, planning_group='arm_right')
    if l_now and r_now:
        success = arm.move_dual_abs(lx=l_now[0], ly=l_now[1], lz=l_now[2], lr=l_now[3], lp=l_now[4], lyaw=l_now[5],
                                    rx=r_now[0], ry=r_now[1], rz=r_now[2], rr=r_now[3], rp=r_now[4], ryaw=r_now[5],
                                    planning_time=15.0, planning_attempts=50)
        print(f"MoveDualAbs (Current) success: {success}")

    print("\n--- Testing move_dual_abs (Relaxed Targets) ---")
    # Move both arms to a closer symmetric position with very loose tolerances
    success = arm.move_dual_abs(lx=0.3, ly=0.15, lz=0.2, lr=0, lp=0, lyaw=0,
                                rx=0.3, ry=-0.15, rz=0.2, rr=0, rp=0, ryaw=0,
                                planning_time=20.0, planning_attempts=100)
    print(f"MoveDualAbs (Relaxed) success: {success}")
    
    print("\n--- Testing move_dual_rel ---")
    # Move both arms up 2cm (smaller step)
    success = arm.move_dual_rel(lz=0.02, rz=0.02, planning_time=20.0, planning_attempts=100)
    print(f"MoveDualRel success: {success}")
    
    print("\n--- Testing move_rel (arm_left) ---")
    # Move a bit more forward
    success = arm.move_rel(x=0.05, y=0, z=0, planning_group='arm_left')
    print(f"MoveRel success: {success}")
    
    print("\n--- Testing get_current_joints_pose (arm_left) ---")
    joints = arm.get_current_joints_pose(planning_group='arm_left')
    print(f"Current Joints: {joints}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
