#include <memory>
#include <chrono>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"

class PointCloudTimestampFixer : public rclcpp::Node
{
public:
  PointCloudTimestampFixer()
  : Node("pointcloud_timestamp_fixer")
  {
    // QoS設定: パケットロスを防ぐためIMUのバッファを大きく確保
    auto qos_lidar = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    auto qos_imu = rclcpp::QoS(rclcpp::KeepLast(2000)).reliable().durability_volatile();

    publisher_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/utlidar/cloud_livox_mid360_fixed", qos_lidar);
    publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "/utlidar/imu_livox_mid360_fixed", qos_imu);

    subscription_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/utlidar/cloud_livox_mid360", qos_lidar,
      std::bind(&PointCloudTimestampFixer::lidar_topic_callback, this, std::placeholders::_1));

    subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/utlidar/imu_livox_mid360", qos_imu,
      std::bind(&PointCloudTimestampFixer::imu_topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "C++ Fixer: Full optimization (Sync, Sanitizer, IIR Filter) running.");
  }

private:
  // --- タイムスタンプ同期用メンバ変数 ---
  bool offset_initialized_ = false;
  rclcpp::Duration time_offset_{0, 0};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  std::mutex sync_mutex_;

  // --- IIRローパスフィルタ用メンバ変数 ---
  bool imu_filter_initialized_ = false;
  double filtered_acc_[3] = {0.0, 0.0, 0.0};
  double filtered_gyro_[3] = {0.0, 0.0, 0.0};
  // フィルタ係数 (1000Hzを想定したカットオフ設定)
  const double alpha_acc_ = 0.02;  
  const double alpha_gyro_ = 0.1;  

  void lidar_topic_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // メッセージの中身を書き換えるためディープコピー
    sensor_msgs::msg::PointCloud2 msg_out = *msg;

    rclcpp::Time native_time(msg_out.header.stamp);
    rclcpp::Time smoothed_time;
    {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        rclcpp::Time now = this->now();
        
        // 共有オフセットの初期化
        if (!offset_initialized_) {
          time_offset_ = now - native_time;
          offset_initialized_ = true;
        }
        smoothed_time = native_time + time_offset_;

        // ROS時刻から大きくズレた場合（0.1秒以上）はオフセットを再計算
        if (std::abs((smoothed_time - now).seconds()) > 0.1) {
          time_offset_ = now - native_time;
          smoothed_time = native_time + time_offset_;
        }
    }

    msg_out.header.stamp = smoothed_time;

    // 【重要】Cartographer をフリーズさせる内部時間フィールドを無効化 (サニタイズ)
    for (auto& field : msg_out.fields) {
      if (field.name == "time" || field.name == "t" || field.name == "timestamp") {
        field.name = "ignored_time";
      }
    }

    publisher_lidar_->publish(msg_out);
  }

  void imu_topic_callback(sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // メッセージの中身を書き換えるためディープコピー
    sensor_msgs::msg::Imu msg_out = *msg;

    // --- タイムスタンプ同期処理 ---
    rclcpp::Time native_time(msg_out.header.stamp);
    rclcpp::Time smoothed_time;
    {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        rclcpp::Time now = this->now();
        
        // 共有オフセットの初期化
        if (!offset_initialized_) {
          time_offset_ = now - native_time;
          offset_initialized_ = true;
        }
        smoothed_time = native_time + time_offset_;

        // ROS時刻から大きくズレた場合（0.1秒以上）はオフセットを再計算
        if (std::abs((smoothed_time - now).seconds()) > 0.1) {
          time_offset_ = now - native_time;
          smoothed_time = native_time + time_offset_;
        }

        // Cartographer のキューを止めないための厳密な単調増加保証
        if (smoothed_time <= last_imu_time_) {
          // 過去または同時刻の場合は、強制的に 1マイクロ秒 未来にする
          smoothed_time = last_imu_time_ + rclcpp::Duration(0, 1000); 
        }
        last_imu_time_ = smoothed_time;
    }

    msg_out.header.stamp = smoothed_time;
    msg_out.header.frame_id = "livox_frame";

    // --- IIR ローパスフィルタの適用 (歩行振動対策) ---
    if (!imu_filter_initialized_) {
      filtered_acc_[0] = msg_out.linear_acceleration.x;
      filtered_acc_[1] = msg_out.linear_acceleration.y;
      filtered_acc_[2] = msg_out.linear_acceleration.z;
      filtered_gyro_[0] = msg_out.angular_velocity.x;
      filtered_gyro_[1] = msg_out.angular_velocity.y;
      filtered_gyro_[2] = msg_out.angular_velocity.z;
      imu_filter_initialized_ = true;
    } else {
      // 加速度の平滑化 (重力ベクトルの安定化)
      filtered_acc_[0] = alpha_acc_ * msg_out.linear_acceleration.x + (1.0 - alpha_acc_) * filtered_acc_[0];
      filtered_acc_[1] = alpha_acc_ * msg_out.linear_acceleration.y + (1.0 - alpha_acc_) * filtered_acc_[1];
      filtered_acc_[2] = alpha_acc_ * msg_out.linear_acceleration.z + (1.0 - alpha_acc_) * filtered_acc_[2];
      
      // 角速度の平滑化
      filtered_gyro_[0] = alpha_gyro_ * msg_out.angular_velocity.x + (1.0 - alpha_gyro_) * filtered_gyro_[0];
      filtered_gyro_[1] = alpha_gyro_ * msg_out.angular_velocity.y + (1.0 - alpha_gyro_) * filtered_gyro_[1];
      filtered_gyro_[2] = alpha_gyro_ * msg_out.angular_velocity.z + (1.0 - alpha_gyro_) * filtered_gyro_[2];
    }

    // フィルタリングされた値をメッセージに書き戻す
    msg_out.linear_acceleration.x = filtered_acc_[0];
    msg_out.linear_acceleration.y = filtered_acc_[1];
    msg_out.linear_acceleration.z = filtered_acc_[2];
    msg_out.angular_velocity.x = filtered_gyro_[0];
    msg_out.angular_velocity.y = filtered_gyro_[1];
    msg_out.angular_velocity.z = filtered_gyro_[2];

    publisher_imu_->publish(msg_out);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_lidar_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_lidar_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTimestampFixer>());
  rclcpp::shutdown();
  return 0;
}
