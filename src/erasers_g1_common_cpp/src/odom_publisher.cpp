#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
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
    this->declare_parameter("child_frame", "base_link"); // G1 は base_link が base_link のような存在
    this->declare_parameter("publish_tf", true);

    parent_frame_ = this->get_parameter("parent_frame").as_string();
    child_frame_ = this->get_parameter("child_frame").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
      "/odommodestate", qos, std::bind(&UnitreeOdomConverter::topic_callback, this, _1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", qos);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "Unitree G1 Odom Converter started.");
  }

private:
  void topic_callback(const unitree_go::msg::SportModeState::SharedPtr msg)
  {
    auto current_time = this->get_clock()->now();

    rclcpp::Time msg_time(msg->stamp.sec, msg->stamp.nanosec);
    if (msg_time.seconds() == 0) {
      msg_time = current_time;
    }

    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = msg_time;
    odom_msg.header.frame_id = parent_frame_;
    odom_msg.child_frame_id = child_frame_;

    odom_msg.pose.pose.position.x = msg->position[0];
    odom_msg.pose.pose.position.y = msg->position[1];
    odom_msg.pose.pose.position.z = msg->position[2];

    odom_msg.pose.pose.orientation.w = msg->imu_state.quaternion[0];
    odom_msg.pose.pose.orientation.x = msg->imu_state.quaternion[1];
    odom_msg.pose.pose.orientation.y = msg->imu_state.quaternion[2];
    odom_msg.pose.pose.orientation.z = msg->imu_state.quaternion[3];

    odom_msg.twist.twist.linear.x = msg->velocity[0];
    odom_msg.twist.twist.linear.y = msg->velocity[1];
    odom_msg.twist.twist.linear.z = msg->velocity[2];

    odom_msg.twist.twist.angular.x = msg->imu_state.gyroscope[0];
    odom_msg.twist.twist.angular.y = msg->imu_state.gyroscope[1];
    odom_msg.twist.twist.angular.z = msg->imu_state.gyroscope[2];

    odom_msg.pose.covariance[0] = 0.01;  // x
    odom_msg.pose.covariance[7] = 0.01;  // y
    odom_msg.pose.covariance[14] = 0.01; // z
    odom_msg.pose.covariance[21] = 0.01; // roll
    odom_msg.pose.covariance[28] = 0.01; // pitch
    odom_msg.pose.covariance[35] = 0.01; // yaw

    odom_msg.twist.covariance[0] = 0.01;
    odom_msg.twist.covariance[7] = 0.01;
    odom_msg.twist.covariance[14] = 0.01;
    odom_msg.twist.covariance[21] = 0.01;
    odom_msg.twist.covariance[28] = 0.01;
    odom_msg.twist.covariance[35] = 0.01;

    odom_pub_->publish(odom_msg);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = msg_time;
      t.header.frame_id = parent_frame_;
      t.child_frame_id = child_frame_;

      t.transform.translation.x = msg->position[0];
      t.transform.translation.y = msg->position[1];
      t.transform.translation.z = msg->position[2];

      t.transform.rotation = odom_msg.pose.pose.orientation;

      tf_broadcaster_->sendTransform(t);
    }
  }

  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  std::string parent_frame_;
  std::string child_frame_;
  bool publish_tf_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UnitreeOdomConverter>());
  rclcpp::shutdown();
  return 0;
}
