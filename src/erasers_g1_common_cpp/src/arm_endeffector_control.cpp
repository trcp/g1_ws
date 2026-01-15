#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>

#include "urdf/model.h"

using namespace std::chrono_literals;

class ArmEndEffectorControl : public rclcpp::Node
{
public:
  ArmEndEffectorControl() : Node("arm_endeffector_control")
  {
    // Parameters
    this->declare_parameter("urdf", "/home/unitree/colcon_ws/src/erasers_g1/g1_description/urdf/g1_comp.urdf");
    this->declare_parameter("base_link", "torso_link");
    this->declare_parameter("left_ee_link", "left_wrist_roll_rubber_hand");
    this->declare_parameter("right_ee_link", "right_wrist_roll_rubber_hand");

    std::string urdf_file = this->get_parameter("urdf").as_string();
    base_link_ = this->get_parameter("base_link").as_string();
    left_ee_link_ = this->get_parameter("left_ee_link").as_string();
    right_ee_link_ = this->get_parameter("right_ee_link").as_string();

    // Publisher for joint control (connects to arm_joint_control node)
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm_joint_control", 10);

    // Initialize KDL Tree from URDF
    if (!loadURDF(urdf_file)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load URDF");
      return;
    }

    // Initialize Interactive Marker Server
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("g1_ee_control", this);

    // Create markers and solvers for Left arm
    RCLCPP_INFO(this->get_logger(), "Initializing Left Arm IK...");
    if (setupArm("left", left_ee_link_)) {
        createMarker("left", left_ee_link_, 0.2, 0.2, 0.8); // Blue
    }

    // Create markers and solvers for Right arm
    RCLCPP_INFO(this->get_logger(), "Initializing Right Arm IK...");
    if (setupArm("right", right_ee_link_)) {
        createMarker("right", right_ee_link_, 0.2, 0.8, 0.2); // Green
    }

    server_->applyChanges();

    // Timer for publishing joint states
    timer_ = this->create_wall_timer(50ms, std::bind(&ArmEndEffectorControl::timerCallback, this));
  }

private:
  struct ArmData {
    std::string side;
    std::string tip_link;
    KDL::Chain chain;
    std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver;
    // We also need joint limits if we want to be strict, but LMA handles limits if configure properly.
    // simpler LMA constructor doesn't take limits, we might need a better solver or check limits manually.
    // LMA solver in KDL (ChainIkSolverPos_LMA) uses Levenberg-Marquardt.
    
    KDL::JntArray q_current; // Current estimated joint positions (seed)
    geometry_msgs::msg::Pose target_pose;
    bool target_updated = false;
    std::vector<std::string> joint_names;
  };

  std::map<std::string, ArmData> arms_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  KDL::Tree tree_;
  std::string base_link_, left_ee_link_, right_ee_link_;

  bool loadURDF(const std::string& urdf_path) {
    if (!kdl_parser::treeFromFile(urdf_path, tree_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree from URDF file: %s", urdf_path.c_str());
      return false;
    }
    return true;
  }

  bool setupArm(const std::string& side, const std::string& tip_link) {
    ArmData arm;
    arm.side = side;
    arm.tip_link = tip_link;

    if (!tree_.getChain(base_link_, tip_link, arm.chain)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain from %s to %s", base_link_.c_str(), tip_link.c_str());
      return false;
    }

    int n_joints = arm.chain.getNrOfJoints();
    RCLCPP_INFO(this->get_logger(), "Arm %s has %d joints", side.c_str(), n_joints);

    // Get joint names
    for (unsigned int i = 0; i < arm.chain.getNrOfSegments(); ++i) {
      const KDL::Segment& seg = arm.chain.getSegment(i);
      const KDL::Joint& jnt = seg.getJoint();
      if (jnt.getType() != KDL::Joint::None) {
        arm.joint_names.push_back(jnt.getName());
      }
    }

    // Initialize Solver
    // LMA solver: epsilon=1E-5, maxiter=500, eps_joints=1E-15
    Eigen::Matrix<double, 6, 1> L;
    L.fill(1.0); // Weight for X,Y,Z, Roll, Pitch, Yaw
    arm.ik_solver = std::make_shared<KDL::ChainIkSolverPos_LMA>(arm.chain, L);

    arm.q_current.resize(n_joints);
    // Initialize with some reasonable default values if possible, for now 0
    // A better approach would be to read current state from /joint_states but for simplicity we start at 0
    // or a "home" pose if known.
    for (int i=0; i<n_joints; ++i) arm.q_current(i) = 0.0;
    
    // Set Initial Target Pose via FK to have marker at current zero-pose
    KDL::ChainFkSolverPos_recursive fk_solver(arm.chain);
    KDL::Frame init_frame;
    fk_solver.JntToCart(arm.q_current, init_frame);
    
    // Convert KDL Frame to Geometry Msg
    arm.target_pose.position.x = init_frame.p.x();
    arm.target_pose.position.y = init_frame.p.y();
    arm.target_pose.position.z = init_frame.p.z();
    double x, y, z, w;
    init_frame.M.GetQuaternion(x, y, z, w);
    arm.target_pose.orientation.x = x;
    arm.target_pose.orientation.y = y;
    arm.target_pose.orientation.z = z;
    arm.target_pose.orientation.w = w;

    arms_[side] = arm;
    return true;
  }

  void createMarker(const std::string& side, const std::string& tip_link, float r, float g, float b) {
    auto& arm = arms_[side];
    
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = base_link_;
    int_marker.header.stamp = this->now();
    int_marker.name = side + "_ee_control";
    int_marker.description = side + " Arm EE Control";
    int_marker.scale = 0.2;
    int_marker.pose = arm.target_pose;

    // Visual Marker (Cube)
    visualization_msgs::msg::Marker box_marker;
    box_marker.type = visualization_msgs::msg::Marker::CUBE;
    box_marker.scale.x = 0.1;
    box_marker.scale.y = 0.1;
    box_marker.scale.z = 0.1;
    box_marker.color.r = r;
    box_marker.color.g = g;
    box_marker.color.b = b;
    box_marker.color.a = 0.8;

    visualization_msgs::msg::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(box_marker);
    int_marker.controls.push_back(box_control);

    // 6DOF Controls
    addControl(int_marker, visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS, "x");
    addControl(int_marker, visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS, "y");
    addControl(int_marker, visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS, "z");
    addControl(int_marker, visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS, "x");
    addControl(int_marker, visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS, "y");
    addControl(int_marker, visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS, "z");

    server_->insert(int_marker);
    server_->setCallback(int_marker.name, std::bind(&ArmEndEffectorControl::processFeedback, this, std::placeholders::_1, side));
  }

  void addControl(visualization_msgs::msg::InteractiveMarker& msg, uint8_t mode, const std::string& axis) {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.interaction_mode = mode;
    if (axis == "x") {
        control.orientation.w = 1; control.orientation.x = 1; control.orientation.y = 0; control.orientation.z = 0;
        control.name = "move_x";
    } else if (axis == "y") {
        control.orientation.w = 1; control.orientation.x = 0; control.orientation.y = 1; control.orientation.z = 0;
        control.name = "move_y";
    } else if (axis == "z") {
        control.orientation.w = 1; control.orientation.x = 0; control.orientation.y = 0; control.orientation.z = 1;
        control.name = "move_z";
    }
    if (mode == visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS) {
        control.name = "rotate_" + axis;
    }
    msg.controls.push_back(control);
  }

  void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback, const std::string& side) {
    if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
        if (arms_.count(side)) {
            arms_[side].target_pose = feedback->pose;
            arms_[side].target_updated = true;
        }
    }
  }

  void timerCallback() {
    sensor_msgs::msg::JointState js_msg;
    js_msg.header.stamp = this->now();

    for (auto& pair : arms_) {
        ArmData& arm = pair.second;
        if (arm.target_updated) {
            // Solve IK
            KDL::Frame target_frame;
            target_frame.p = KDL::Vector(arm.target_pose.position.x, arm.target_pose.position.y, arm.target_pose.position.z);
            target_frame.M = KDL::Rotation::Quaternion(
                arm.target_pose.orientation.x,
                arm.target_pose.orientation.y,
                arm.target_pose.orientation.z,
                arm.target_pose.orientation.w
            );

            KDL::JntArray q_out;
            q_out.resize(arm.chain.getNrOfJoints());
            
            // Use current joint estimate as seed
            int ret = arm.ik_solver->CartToJnt(arm.q_current, target_frame, q_out);
            
            if (ret >= 0) {
                // Update current estimate
                arm.q_current = q_out;
                
                // Add to JointState message
                for (unsigned int i=0; i<arm.joint_names.size(); ++i) {
                    js_msg.name.push_back(arm.joint_names[i]);
                    js_msg.position.push_back(q_out(i));
                }
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "IK Solver failed for %s arm! Error code: %d", arm.side.c_str(), ret);
            }
        }
    }

    if (!js_msg.name.empty()) {
        joint_pub_->publish(js_msg);
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmEndEffectorControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
