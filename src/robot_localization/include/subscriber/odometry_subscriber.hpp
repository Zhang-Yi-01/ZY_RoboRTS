/*
 * @Description: 订阅odometry数据
 */
#ifndef ODOMETRY_SUBSCRIBER_HPP_
#define ODOMETRY_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
// #include <Eigen/Dense>

#include "../sensor_data/pose_data.hpp"
namespace robot_localization 
{


class OdometrySubscriber
{
  
  public:
    OdometrySubscriber(std::shared_ptr<rclcpp::Node> &node_, std::string topic_name, size_t buff_size);
    OdometrySubscriber() = default;
    void ParseData(std::deque<PoseData>& deque_pose_data);

  private:
    void msg_callback(const nav_msgs::msg::Odometry& odom_msg_ptr);

  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    // ros::Subscriber subscriber_;
    std::deque<PoseData> new_pose_data_;

    std::mutex buff_mutex_; 
};
}
#endif