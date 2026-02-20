#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>
#include <random>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <interactive_markers/interactive_marker_server.hpp>

// Pinocchio
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

using namespace std::chrono_literals;

class ArmEndEffectorControl : public rclcpp::Node
{
public:
  ArmEndEffectorControl() : Node("arm_endeffector_control")
  {
    this->declare_parameter("urdf_path", "/home/unitree/colcon_ws/src/erasers_g1/g1_description/urdf/g1_comp.urdf");
    
    std::string urdf_path = this->get_parameter("urdf_path").as_string();

    if (!initPinocchio(urdf_path)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize Pinocchio model.");
    }

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/upper_joints_control", 10);
    
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", qos, 
      std::bind(&ArmEndEffectorControl::jointStateCallback, this, std::placeholders::_1));

    left_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/left_arm/target_pose", 10,
      std::bind(&ArmEndEffectorControl::leftPoseCallback, this, std::placeholders::_1));

    right_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/right_arm/target_pose", 10,
      std::bind(&ArmEndEffectorControl::rightPoseCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(50ms, std::bind(&ArmEndEffectorControl::timerCallback, this));
    
    // Interactive Marker Server
    im_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "arm_endeffector_control",
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_topics_interface(),
      this->get_node_services_interface()
    );

    // Create interactive markers based on FK poses
    make6DofMarker("left", targets_["left"].position, targets_["left"].orientation);
    make6DofMarker("right", targets_["right"].position, targets_["right"].orientation);
    im_server_->applyChanges();
  }

private:
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_;
  pinocchio::Model model_full_, model_;
  pinocchio::Data data_full_, data_;
  Eigen::VectorXd q_current_full_; 
  Eigen::VectorXd q_solution_; // Commanded state
  Eigen::VectorXd q_measured_; // Measured state
  Eigen::VectorXd q_last_solve_; // For smoothness cost
  
  std::vector<std::string> locked_joints_;
  std::vector<std::string> active_joint_names_;

  struct ArmTarget {
      Eigen::Vector3d position;
      Eigen::Quaterniond orientation;
      bool is_interacting = false;
  };
  std::map<std::string, ArmTarget> targets_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex arm_mutex_;

  pinocchio::FrameIndex left_ee_frame_id_, right_ee_frame_id_;

  bool initPinocchio(const std::string& urdf_path) {
      try {
          // Load URDF using FreeFlyer to ensure a valid root joint and avoid fixed-base assertion errors in reduced models
          std::ifstream urdf_file(urdf_path);
          if (!urdf_file) {
             RCLCPP_ERROR(this->get_logger(), "File not found: %s", urdf_path.c_str());
             return false;
          }
          pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), model_full_);
          RCLCPP_INFO(this->get_logger(), "Pinocchio model initialized from %s", urdf_path.c_str());
      } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "Pinocchio build exception: %s", e.what());
          // Fallback to fixed base if FreeFlyer fails
          pinocchio::urdf::buildModel(urdf_path, model_full_);
      }

      // Identify active arm/waist joints and locked joints
      std::vector<std::string> allowed_keywords = {
          "waist_yaw", "waist_roll", "waist_pitch",
          "torso",
          "left_shoulder", "left_elbow", "left_wrist",
          "right_shoulder", "right_elbow", "right_wrist"
      };

      std::vector<pinocchio::JointIndex> joints_to_lock;
      // Evaluate all joints (from 1 to njoints-1)
      for (pinocchio::JointIndex joint_id = 1; joint_id < model_full_.joints.size(); ++joint_id) {
          std::string name = model_full_.names[joint_id];
          bool is_active = false;
          for (const auto& kw : allowed_keywords) {
              if (name.find(kw) != std::string::npos) {
                  is_active = true;
                  break;
              }
          }
          if (!is_active) {
              joints_to_lock.push_back(joint_id);
              locked_joints_.push_back(name);
          } else {
              active_joint_names_.push_back(name);
          }
      }

      // For safety, let's keep neutral position references
      Eigen::VectorXd q_ref = pinocchio::neutral(model_full_);
      
      // Build reduced model keeping only active joints
      model_ = pinocchio::buildReducedModel(model_full_, joints_to_lock, q_ref);
      
      // Initialize states
      q_solution_ = pinocchio::neutral(model_);
      q_measured_ = pinocchio::neutral(model_);
      q_last_solve_ = q_solution_;
      q_current_full_ = pinocchio::neutral(model_full_);

      // Add Offset Frames (0.2m x) relative to wrist_roll_joint
      addToFrame("left_wrist_roll_joint", "L_ee", Eigen::Vector3d(0.2, 0, 0));
      addToFrame("right_wrist_roll_joint", "R_ee", Eigen::Vector3d(0.2, 0, 0));

      left_ee_frame_id_ = model_.getFrameId("L_ee");
      right_ee_frame_id_ = model_.getFrameId("R_ee");

      // Verify the model and data consistency after addition
      data_ = pinocchio::Data(model_);
      if (!model_.check(data_)) {
          RCLCPP_ERROR(this->get_logger(), "Model and Data consistency check failed after frame addition.");
          return false;
      }

      // Invalidate data to force update
      pinocchio::framesForwardKinematics(model_, data_, q_solution_);
      
      // Initialize targets
      updateTargetFromFK("left");
      updateTargetFromFK("right");

      return true;
  }
  
  void addToFrame(const std::string& joint_name, const std::string& new_frame_name, const Eigen::Vector3d& trans) {
      if (!model_.existJointName(joint_name)) {
          RCLCPP_WARN(this->get_logger(), "Joint %s not found in reduced model", joint_name.c_str());
          return;
      }
      pinocchio::JointIndex parent_joint = model_.getJointId(joint_name);
      pinocchio::FrameIndex parent_frame = model_.getFrameId(joint_name); 
      
      pinocchio::Frame frame(new_frame_name, parent_joint, parent_frame, 
                             pinocchio::SE3(Eigen::Matrix3d::Identity(), trans), 
                             pinocchio::OP_FRAME);
      model_.addFrame(frame);
      // Rebuild data after adding frame
      data_ = pinocchio::Data(model_);
  }

  void updateTargetFromFK(const std::string& side) {
      pinocchio::FrameIndex fid = (side == "left") ? left_ee_frame_id_ : right_ee_frame_id_;
      pinocchio::framesForwardKinematics(model_, data_, q_solution_); // Use solution
      const auto& pose = data_.oMf[fid];
      
      ArmTarget& t = targets_[side];
      t.position = pose.translation();
      t.orientation = Eigen::Quaterniond(pose.rotation());
  }

  void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
      std::lock_guard<std::mutex> lock(arm_mutex_);
      
      std::string side = (feedback->marker_name == "left_ee_marker") ? "left" : "right";
      
      targets_[side].position << feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z;
      targets_[side].orientation = Eigen::Quaterniond(
          feedback->pose.orientation.w,
          feedback->pose.orientation.x,
          feedback->pose.orientation.y,
          feedback->pose.orientation.z
      );
      targets_[side].is_interacting = true;
  }

  void make6DofMarker(const std::string& side, const Eigen::Vector3d& pos, const Eigen::Quaterniond& ori) {
      visualization_msgs::msg::InteractiveMarker int_marker;
      int_marker.header.frame_id = "pelvis";
      int_marker.name = side + "_ee_marker";
      int_marker.description = "Control " + side + " Arm EE";
      int_marker.scale = 0.15;
      int_marker.pose.position.x = pos.x();
      int_marker.pose.position.y = pos.y();
      int_marker.pose.position.z = pos.z();
      int_marker.pose.orientation.w = ori.w();
      int_marker.pose.orientation.x = ori.x();
      int_marker.pose.orientation.y = ori.y();
      int_marker.pose.orientation.z = ori.z();
      
      visualization_msgs::msg::InteractiveMarkerControl control;
      control.orientation.w = 1;
      control.orientation.x = 1; control.orientation.y = 0; control.orientation.z = 0;
      control.name = "rotate_x"; control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_x";   control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0; control.orientation.y = 1; control.orientation.z = 0;
      control.name = "rotate_z"; control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_z";   control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0; control.orientation.y = 0; control.orientation.z = 1;
      control.name = "rotate_y"; control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_y";   control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);

      im_server_->insert(int_marker, std::bind(&ArmEndEffectorControl::processFeedback, this, std::placeholders::_1));
  }

  void leftPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(arm_mutex_);
      targets_["left"].position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
      targets_["left"].orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
      targets_["left"].is_interacting = true;
      
      // Update interactive marker to match Topic input
      im_server_->setPose("left_ee_marker", msg->pose);
      im_server_->applyChanges();
  }

  void rightPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(arm_mutex_);
      targets_["right"].position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
      targets_["right"].orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
      targets_["right"].is_interacting = true;
      
      // Update interactive marker to match Topic input
      im_server_->setPose("right_ee_marker", msg->pose);
      im_server_->applyChanges();
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(arm_mutex_);
      for (size_t i = 0; i < msg->name.size(); ++i) {
          if (model_.existJointName(msg->name[i])) {
              pinocchio::JointIndex jid = model_.getJointId(msg->name[i]);
              // Check if idx_q is valid (could be -1 for fixed joints)
              int idx = model_.joints[jid].idx_q();
              if (idx >= 0 && idx < q_measured_.size()) {
                 q_measured_[idx] = msg->position[i];
              }
          }
      }
      // On startup or first message, if solution is zero, maybe sync solution to measured?
      // But we want to start at Zero as per spec. So we leave q_solution_ alone.
  }


  void timerCallback() {
      std::lock_guard<std::mutex> lock(arm_mutex_);

      bool interacting_any = targets_["left"].is_interacting || targets_["right"].is_interacting;
      
      if (interacting_any) {
          solveIK();
          publishJoints(q_solution_);
          q_last_solve_ = q_solution_; 
      } else {
          // HOLD POSE (Do not reset to zero)
          // Just publish the last solution
          publishJoints(q_solution_);
      }
  }

  void solveIK() {
      // Damped Least Squares Optimization
      double lambda = 1e-4; 
      int max_iter = 20; 
      double clamp_p = 0.5; // max translation error to attempt in one step
      double clamp_r = 0.5; // max rotation error to attempt in one step
      
      for (int iter=0; iter<max_iter; ++iter) {
          pinocchio::framesForwardKinematics(model_, data_, q_solution_);
          pinocchio::computeJointJacobians(model_, data_, q_solution_);
          
          Eigen::VectorXd grad = Eigen::VectorXd::Zero(model_.nv);
          Eigen::MatrixXd hess = Eigen::MatrixXd::Zero(model_.nv, model_.nv);
          
          if (targets_["left"].is_interacting) addCost(left_ee_frame_id_, targets_["left"], grad, hess, clamp_p, clamp_r);
          if (targets_["right"].is_interacting) addCost(right_ee_frame_id_, targets_["right"], grad, hess, clamp_p, clamp_r);
           
          // Regularization (pull towards neutral/0)
          grad += 0.02 * q_solution_;
          hess.diagonal().array() += 0.02;
          
          // Smoothness (pull towards last solution)
          grad += 0.1 * (q_solution_ - q_last_solve_);
          hess.diagonal().array() += 0.1;
          
          hess.diagonal().array() += lambda; // Damping
          
          Eigen::VectorXd dq = hess.ldlt().solve(-grad);
          q_solution_ += dq;
          
          for (int k=0; k<model_.nq; ++k) {
               if (k >= q_solution_.size()) break; // Safety
               if (q_solution_[k] < model_.lowerPositionLimit[k]) q_solution_[k] = model_.lowerPositionLimit[k];
               if (q_solution_[k] > model_.upperPositionLimit[k]) q_solution_[k] = model_.upperPositionLimit[k];
          }
      }
  }

  void addCost(pinocchio::FrameIndex fid, const ArmTarget& target, Eigen::VectorXd& grad, Eigen::MatrixXd& hess, double clamp_p, double clamp_r) {
      const auto& transform = data_.oMf[fid];
      Eigen::MatrixXd J(6, model_.nv); J.setZero();
      pinocchio::getFrameJacobian(model_, data_, fid, pinocchio::LOCAL_WORLD_ALIGNED, J);
      
      Eigen::Vector3d ep = transform.translation() - target.position;
      Eigen::Vector3d er = pinocchio::log3(transform.rotation() * target.orientation.inverse()); 
      
      // Error Clamping to avoid extreme gradients causing divergence
      if (ep.norm() > clamp_p) ep = ep.normalized() * clamp_p;
      if (er.norm() > clamp_r) er = er.normalized() * clamp_r;

      // Position highly weighted, Rotation relaxed a bit to allow 5 DOF + Waist reaching
      double w_p = 50.0;
      double w_r = 1.0;
      
      grad += w_p * J.topRows(3).transpose() * ep;
      hess += w_p * J.topRows(3).transpose() * J.topRows(3);
      
      grad += w_r * J.bottomRows(3).transpose() * er;
      hess += w_r * J.bottomRows(3).transpose() * J.bottomRows(3);
  }
  
  void publishJoints(const Eigen::VectorXd& q) {
      if (q.size() != model_.nq) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "q size mismatch: %ld vs %d", q.size(), model_.nq);
          return;
      }
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = this->now();
      
      // Map active joints only
      for (size_t k = 0; k < active_joint_names_.size(); ++k) {
          msg.name.push_back(active_joint_names_[k]);
          
          // Find index in reduced q
          if (model_.existJointName(active_joint_names_[k])) {
             pinocchio::JointIndex jid = model_.getJointId(active_joint_names_[k]);
             int q_idx = model_.joints[jid].idx_q();
             if (q_idx >= 0 && q_idx < q.size()) msg.position.push_back(q[q_idx]);
             else msg.position.push_back(0.0);
          } else {
             msg.position.push_back(0.0);
          }
      }
      joint_pub_->publish(msg);
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
