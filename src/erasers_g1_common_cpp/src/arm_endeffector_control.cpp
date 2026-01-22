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
#include "interactive_markers/interactive_marker_server.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"

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

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm_joint_control", 10);
    
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", qos, 
      std::bind(&ArmEndEffectorControl::jointStateCallback, this, std::placeholders::_1));

    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("g1_ee_control", this);

    if (!initPinocchio(urdf_path)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize Pinocchio model");
        return;
    }

    createMarker("left");
    createMarker("right");
    server_->applyChanges();

    timer_ = this->create_wall_timer(50ms, std::bind(&ArmEndEffectorControl::timerCallback, this));
  }

private:
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

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex arm_mutex_;

  pinocchio::FrameIndex left_ee_frame_id_, right_ee_frame_id_;

  bool initPinocchio(const std::string& urdf_path) {
      try {
          // Load URDF from file (using direct file path usually works with Pinocchio)
          // If root inertia issues persist, Pinocchio usually handles them by adding a universe link, 
          // or we can use the string patching method. 
          // Given the KDL issue, let's use the patch to be safe and consistent with previous request.
          std::ifstream urdf_file(urdf_path);
          if (!urdf_file) {
             RCLCPP_ERROR(this->get_logger(), "File not found: %s", urdf_path.c_str());
             return false;
          }
          std::stringstream buffer;
          buffer << urdf_file.rdbuf();
          std::string urdf_xml = buffer.str();

          size_t pelvis_pos = urdf_xml.find("<link name=\"pelvis\">");
          if (pelvis_pos != std::string::npos) {
              size_t inertial_start = urdf_xml.find("<inertial>", pelvis_pos);
              size_t link_end = urdf_xml.find("</link>", pelvis_pos);
              if (inertial_start != std::string::npos && inertial_start < link_end) {
                  size_t inertial_end = urdf_xml.find("</inertial>", inertial_start);
                  if (inertial_end != std::string::npos) {
                      urdf_xml.erase(inertial_start, inertial_end - inertial_start + 11);
                      RCLCPP_INFO(this->get_logger(), "Patched URDF to remove root inertia");
                  }
              }
          }

          pinocchio::urdf::buildModelFromXML(urdf_xml, model_full_);
      } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "Pinocchio build exception: %s", e.what());
           pinocchio::urdf::buildModel(urdf_path, model_full_);
      }

      // Identify arm joints
      std::vector<std::string> allowed_keywords = {
          "left_shoulder", "left_elbow", "left_wrist",
          "right_shoulder", "right_elbow", "right_wrist"
      };

      std::vector<pinocchio::JointIndex> joints_to_lock;
      // Joint 1 is root
      for (pinocchio::JointIndex joint_id = 1; joint_id < model_full_.joints.size(); ++joint_id) {
          std::string name = model_full_.names[joint_id];
          bool is_arm = false;
          for (const auto& kw : allowed_keywords) {
              if (name.find(kw) != std::string::npos) {
                  is_arm = true;
                  break;
              }
          }
          if (!is_arm) {
              joints_to_lock.push_back(joint_id);
              locked_joints_.push_back(name);
          } else {
              active_joint_names_.push_back(name);
          }
      }

      Eigen::VectorXd q_ref = pinocchio::neutral(model_full_);
      model_ = pinocchio::buildReducedModel(model_full_, joints_to_lock, q_ref);
      data_ = pinocchio::Data(model_);
      
      q_solution_ = pinocchio::neutral(model_);
      q_measured_ = pinocchio::neutral(model_);
      q_last_solve_ = q_solution_;
      q_current_full_ = pinocchio::neutral(model_full_);

      // Add Offset Frames (0.2m x) relative to wrist_roll_joint
      addToFrame("left_wrist_roll_joint", "L_ee", Eigen::Vector3d(0.2, 0, 0));
      addToFrame("right_wrist_roll_joint", "R_ee", Eigen::Vector3d(0.2, 0, 0));

      left_ee_frame_id_ = model_.getFrameId("L_ee");
      right_ee_frame_id_ = model_.getFrameId("R_ee");

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

  void createMarker(const std::string& side) {
      ArmTarget& t = targets_[side];

      visualization_msgs::msg::InteractiveMarker int_marker;
      int_marker.header.frame_id = "torso_link";
      int_marker.header.stamp = this->now();
      int_marker.name = side + "_ee_control";
      int_marker.scale = 0.2;
      
      int_marker.pose.position.x = t.position.x();
      int_marker.pose.position.y = t.position.y();
      int_marker.pose.position.z = t.position.z();
      int_marker.pose.orientation.x = t.orientation.x();
      int_marker.pose.orientation.y = t.orientation.y();
      int_marker.pose.orientation.z = t.orientation.z();
      int_marker.pose.orientation.w = t.orientation.w();

      visualization_msgs::msg::InteractiveMarkerControl box_control;
      box_control.always_visible = true;
      visualization_msgs::msg::Marker box_marker;
      box_marker.type = visualization_msgs::msg::Marker::CUBE;
      box_marker.scale.x = 0.05; box_marker.scale.y = 0.05; box_marker.scale.z = 0.05;
      box_marker.color.r = (side=="left")?0.2:0.2; box_marker.color.g = (side=="left")?0.2:0.8; box_marker.color.b = (side=="left")?0.8:0.2; 
      box_marker.color.a = 0.8;
      box_control.markers.push_back(box_marker);
      box_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
      int_marker.controls.push_back(box_control);
      
      add6DOFControls(int_marker);

      server_->insert(int_marker);
      server_->setCallback(int_marker.name, std::bind(&ArmEndEffectorControl::processFeedback, this, std::placeholders::_1, side));
  }
  
  void add6DOFControls(visualization_msgs::msg::InteractiveMarker& msg) {
      visualization_msgs::msg::InteractiveMarkerControl control;

      control.orientation.w = 1; control.orientation.x = 1; control.orientation.y = 0; control.orientation.z = 0;
      control.name = "rotate_x"; control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS; msg.controls.push_back(control);
      control.name = "move_x"; control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS; msg.controls.push_back(control);

      control.orientation.w = 1; control.orientation.x = 0; control.orientation.y = 1; control.orientation.z = 0;
      control.name = "rotate_z"; control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS; msg.controls.push_back(control);
      control.name = "move_z"; control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS; msg.controls.push_back(control);

      control.orientation.w = 1; control.orientation.x = 0; control.orientation.y = 0; control.orientation.z = 1;
      control.name = "rotate_y"; control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS; msg.controls.push_back(control);
      control.name = "move_y"; control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS; msg.controls.push_back(control);
  }

  void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback, const std::string& side) {
      std::lock_guard<std::mutex> lock(arm_mutex_);
      ArmTarget& t = targets_[side];
      
      if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN) {
          t.is_interacting = true;
          // When starting interaction, sync target to current marker pose
          t.position.x() = feedback->pose.position.x;
          t.position.y() = feedback->pose.position.y;
          t.position.z() = feedback->pose.position.z;
          t.orientation = Eigen::Quaterniond(feedback->pose.orientation.w, feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z);
          RCLCPP_INFO(this->get_logger(), "%s INTERACT START", side.c_str());
      } else if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
          t.is_interacting = false;
          // Keep the marker where it is (or snap to valid IK sol).
          // If we snap to FK(q_solution_), it ensures marker stays on valid workspace.
          updateTargetFromFK(side);
          
          geometry_msgs::msg::Pose p;
          p.position.x = t.position.x(); p.position.y = t.position.y(); p.position.z = t.position.z();
          p.orientation.x = t.orientation.x(); p.orientation.y = t.orientation.y(); p.orientation.z = t.orientation.z(); p.orientation.w = t.orientation.w();
          
          server_->setPose(side+"_ee_control", p);
          server_->applyChanges();
          RCLCPP_INFO(this->get_logger(), "%s INTERACT END", side.c_str());
      } else if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
          t.position.x() = feedback->pose.position.x;
          t.position.y() = feedback->pose.position.y;
          t.position.z() = feedback->pose.position.z;
          t.orientation = Eigen::Quaterniond(feedback->pose.orientation.w, feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z);
      }
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
      double lambda = 1e-3; 
      int max_iter = 10; 
      
      // Seed with last solution (smoother) or measured (safer)?
      // If we use measured, we need to be careful about noise.
      // Let's use q_solution_ as we are controlling it.
      
      for (int iter=0; iter<max_iter; ++iter) {
          pinocchio::framesForwardKinematics(model_, data_, q_solution_);
          pinocchio::computeJointJacobians(model_, data_, q_solution_);
          
          Eigen::VectorXd grad = Eigen::VectorXd::Zero(model_.nv);
          Eigen::MatrixXd hess = Eigen::MatrixXd::Zero(model_.nv, model_.nv);
          
          if (targets_["left"].is_interacting) addCost(left_ee_frame_id_, targets_["left"], grad, hess);
          if (targets_["right"].is_interacting) addCost(right_ee_frame_id_, targets_["right"], grad, hess);
           
          // Regularization: 0.02 * ||q||^2
          grad += 0.04 * q_solution_;
          hess.diagonal().array() += 0.04;
          
          // Smoothness: 0.1 * ||q - q_last||^2
          grad += 0.2 * (q_solution_ - q_last_solve_);
          hess.diagonal().array() += 0.2; // 2 * 0.1
          
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

  void addCost(pinocchio::FrameIndex fid, const ArmTarget& target, Eigen::VectorXd& grad, Eigen::MatrixXd& hess) {
      const auto& transform = data_.oMf[fid];
      Eigen::MatrixXd J(6, model_.nv); J.setZero();
      pinocchio::getFrameJacobian(model_, data_, fid, pinocchio::LOCAL_WORLD_ALIGNED, J);
      
      Eigen::Vector3d ep = transform.translation() - target.position;
      Eigen::Vector3d er = pinocchio::log3(transform.rotation() * target.orientation.inverse()); 
      
      // Weights from Python: 50, 0.5
      double w_p = 50.0;
      double w_r = 0.5;
      
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
