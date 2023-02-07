/*
 * @Description: 在ros中发布IMU数据
 */
#ifndef PUBLISHER_IMU_PUBLISHER_HPP_
#define PUBLISHER_IMU_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "../sensor_data/imu_data.hpp"

namespace robot_localization {
class IMUPublisher {
  public:
    IMUPublisher(
      std::shared_ptr<rclcpp::Node> &node_,
      std::string topic_name,
      std::string frame_id,
      size_t buff_size
    );
    IMUPublisher() = default;

    void Publish(const ImuData &imu_data, double time);
    void Publish(const ImuData &imu_data);
    void Publish(const ImuData &imu_data, builtin_interfaces::msg::Time time);

    bool HasSubscribers(void);

  private:
    void PublishData(const ImuData &imu_data, builtin_interfaces::msg::Time time);
    rclcpp::Node::SharedPtr ros2_node_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;

    // ros::NodeHandle nh_;
    // ros::Publisher publisher_;
    std::string frame_id_;

    sensor_msgs::msg::Imu imu_;
};
} 
#endif