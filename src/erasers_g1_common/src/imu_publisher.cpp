#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "unitree_hg/msg/low_state.hpp"

class ImuPublisher : public rclcpp::Node {
public:
    ImuPublisher()
    : Node("imu_publisher") {
        this->declare_parameter<std::string>("frame_name", "imu_in_torso");
        
        this->get_parameter("frame_name", frame_id_);
        
        RCLCPP_INFO(this->get_logger(), "Using frame_id: '%s'", frame_id_.c_str());

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
        state_sub_ = this->create_subscription<unitree_hg::msg::LowState>(
            "/lowstate", 10, std::bind(&ImuPublisher::stateCallback, this, std::placeholders::_1));
    }

private:
    void stateCallback(const unitree_hg::msg::LowState::SharedPtr data) {
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = frame_id_;

        imu_msg.orientation.w = data->imu_state.quaternion[0];
        imu_msg.orientation.x = data->imu_state.quaternion[1];
        imu_msg.orientation.y = data->imu_state.quaternion[2];
        imu_msg.orientation.z = data->imu_state.quaternion[3];

        imu_msg.angular_velocity.x = data->imu_state.gyroscope[0];
        imu_msg.angular_velocity.y = data->imu_state.gyroscope[1];
        imu_msg.angular_velocity.z = data->imu_state.gyroscope[2];

        imu_msg.linear_acceleration.x = data->imu_state.accelerometer[0];
        imu_msg.linear_acceleration.y = data->imu_state.accelerometer[1];
        imu_msg.linear_acceleration.z = data->imu_state.accelerometer[2];

        imu_pub_->publish(imu_msg);

        // This log can be very verbose, so it's commented out for practical use.
        // RCLCPP_INFO(this->get_logger(), "IMU message published!");
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr state_sub_;
    std::string frame_id_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
