#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;

class UnitreeOdomConverter : public rclcpp::Node
{
public:
  UnitreeOdomConverter()
  : Node("unitree_odom_converter")
  {
    this->declare_parameter("parent_frame", "odom");
    this->declare_parameter("base_footprint_frame", "base_footprint");
    this->declare_parameter("base_link_frame", "base_link");
    this->declare_parameter("publish_tf", true);

    parent_frame_ = this->get_parameter("parent_frame").as_string();
    base_footprint_frame_ = this->get_parameter("base_footprint_frame").as_string();
    base_link_frame_ = this->get_parameter("base_link_frame").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
      "/odommodestate", qos,
      std::bind(&UnitreeOdomConverter::topic_callback, this, _1));

    odom_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>("/odom", qos);

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "Unitree Odom Converter started.");
  }

private:
  void topic_callback(const unitree_go::msg::SportModeState::SharedPtr msg)
  {
    auto current_time = this->get_clock()->now();

    rclcpp::Time msg_time(msg->stamp.sec, msg->stamp.nanosec);
    if (msg_time.seconds() == 0) {
      msg_time = current_time;
    }

    tf2::Quaternion q(
      msg->imu_state.quaternion[1],
      msg->imu_state.quaternion[2],
      msg->imu_state.quaternion[3],
      msg->imu_state.quaternion[0]);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    //------------------------------------------
    // Odometry (odom -> base_footprint)
    //------------------------------------------
    nav_msgs::msg::Odometry odom_msg;

    odom_msg.header.stamp = msg_time;
    odom_msg.header.frame_id = parent_frame_;
    odom_msg.child_frame_id = base_footprint_frame_;

    odom_msg.pose.pose.position.x = msg->position[0];
    odom_msg.pose.pose.position.y = msg->position[1];
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q_yaw;
    q_yaw.setRPY(0.0, 0.0, yaw);

    odom_msg.pose.pose.orientation.x = q_yaw.x();
    odom_msg.pose.pose.orientation.y = q_yaw.y();
    odom_msg.pose.pose.orientation.z = q_yaw.z();
    odom_msg.pose.pose.orientation.w = q_yaw.w();

    //------------------------------------------
    // velocity
    //------------------------------------------
    odom_msg.twist.twist.linear.x = msg->velocity[0];
    odom_msg.twist.twist.linear.y = msg->velocity[1];
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = msg->imu_state.gyroscope[2];
    
    odom_msg.pose.covariance[0]  = 0.01;  // x
    odom_msg.pose.covariance[7]  = 0.01;  // y
    odom_msg.pose.covariance[14] = 0.01;  // z
    odom_msg.pose.covariance[21] = 0.01;  // roll
    odom_msg.pose.covariance[28] = 0.01;  // pitch
    odom_msg.pose.covariance[35] = 0.01;  // yaw

    odom_msg.twist.covariance[0]  = 0.01;
    odom_msg.twist.covariance[7]  = 0.01;
    odom_msg.twist.covariance[14] = 0.01;
    odom_msg.twist.covariance[21] = 0.01;
    odom_msg.twist.covariance[28] = 0.01;
    odom_msg.twist.covariance[35] = 0.01;

    odom_pub_->publish(odom_msg);

    //------------------------------------------
    // TF publish
    //------------------------------------------
    if (publish_tf_) {

      //------------------------------
      // odom -> base_footprint
      //------------------------------
      geometry_msgs::msg::TransformStamped t_fp;

      t_fp.header.stamp = msg_time;
      t_fp.header.frame_id = parent_frame_;
      t_fp.child_frame_id = base_footprint_frame_;

      t_fp.transform.translation.x = msg->position[0];
      t_fp.transform.translation.y = msg->position[1];
      t_fp.transform.translation.z = 0.0;

      t_fp.transform.rotation.x = q_yaw.x();
      t_fp.transform.rotation.y = q_yaw.y();
      t_fp.transform.rotation.z = q_yaw.z();
      t_fp.transform.rotation.w = q_yaw.w();

      tf_broadcaster_->sendTransform(t_fp);

      //------------------------------
      // base_footprint -> base_link
      //------------------------------
      geometry_msgs::msg::TransformStamped t_bl;

      t_bl.header.stamp = msg_time;
      t_bl.header.frame_id = base_footprint_frame_;
      t_bl.child_frame_id = base_link_frame_;

      t_bl.transform.translation.x = 0.0;
      t_bl.transform.translation.y = 0.0;
      t_bl.transform.translation.z = msg->position[2];

      tf2::Quaternion q_rp;
      q_rp.setRPY(roll, pitch, 0.0);

      t_bl.transform.rotation.x = q_rp.x();
      t_bl.transform.rotation.y = q_rp.y();
      t_bl.transform.rotation.z = q_rp.z();
      t_bl.transform.rotation.w = q_rp.w();

      tf_broadcaster_->sendTransform(t_bl);
    }
  }

  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string parent_frame_;
  std::string base_footprint_frame_;
  std::string base_link_frame_;
  bool publish_tf_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UnitreeOdomConverter>());
  rclcpp::shutdown();
  return 0;
}
