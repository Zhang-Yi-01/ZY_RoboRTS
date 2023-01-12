/*
 * @Description: 通过ros发布点云
 */
#include "../../include/publisher/imu_publisher.hpp"

#include "glog/logging.h"

namespace robot_localization {

IMUPublisher::IMUPublisher(
    std::shared_ptr<rclcpp::Node> &node_,
    std::string topic_name,
    std::string frame_id,
    size_t buff_size
)   :ros2_node_(node_), frame_id_(frame_id) 
{
    publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>(topic_name,buff_size);
    imu_.header.frame_id = frame_id_;

}

void IMUPublisher::Publish(const ImuData &imu_data, double time) 
{   
    // double 转换 builtin_interfaces::msg::Time找不到方法！！！！！！
    // ros::Time ros_time(time);
    time = time*1;
    builtin_interfaces::msg::Time ros2_time = ros2_node_->now();

    PublishData(imu_data, ros2_time);
}

void IMUPublisher::Publish(const ImuData &imu_data, builtin_interfaces::msg::Time time) 
{
    PublishData(imu_data, time);
}

void IMUPublisher::Publish(const ImuData &imu_data) 
{
    builtin_interfaces::msg::Time time = ros2_node_->now();
    PublishData(imu_data, time);
}

void IMUPublisher::PublishData(const ImuData &imu_data, builtin_interfaces::msg::Time time) 
{
    imu_.header.stamp = time;
    
    // set orientation:
    imu_.orientation.w = imu_data.orientation_.w;
    imu_.orientation.x = imu_data.orientation_.x;
    imu_.orientation.y = imu_data.orientation_.y;
    imu_.orientation.z = imu_data.orientation_.z;

    // set angular velocity:
    imu_.angular_velocity.x = imu_data.angular_velocity_.x;
    imu_.angular_velocity.y = imu_data.angular_velocity_.y;
    imu_.angular_velocity.z = imu_data.angular_velocity_.z;

    // set linear acceleration:
    imu_.linear_acceleration.x = imu_data.linear_acceleration_.x;
    imu_.linear_acceleration.y = imu_data.linear_acceleration_.y;
    imu_.linear_acceleration.z = imu_data.linear_acceleration_.z;

    imu_.header.frame_id = frame_id_;
    publisher_->publish(imu_);
}

bool IMUPublisher::HasSubscribers(void) 
{
    // return publisher_.getNumSubscribers() != 0;
    // ros2 好像没有这个函数了,留待后面吧
    return true;
}

} // namespace lidar_localization