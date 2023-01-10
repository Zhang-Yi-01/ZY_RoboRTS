/*
 * @Description: 里程计发布
 */
#ifndef ODOMETRY_PUBLISHER_HPP
#define ODOMETRY_PUBLISHER_HPP

// ros
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
// c++
#include <string>
// eigen
#include <Eigen/Dense>

namespace robot_localization
{
    class OdometryPublisher
    {
    public:
        OdometryPublisher(std::shared_ptr<rclcpp::Node>& node_,
                          std::string topic_name,
                          std::string base_frame_id,
                          std::string child_frame_id,
                          size_t buff_size);
        OdometryPublisher() = default;

        // void Publish(const Eigen::Matrix4d &transform_matrix, double time);
        void Publish(const Eigen::Matrix4d &transform_matrix);
        // void Publish(const Eigen::Matrix4d &transform_matrix, const Eigen::Vector3d &vel, double time);

        bool HasSubscriber();

    private:
        void PublishData(const Eigen::Matrix4d &transform_matrix, builtin_interfaces::msg::Time time);

        void PublishData( 
                          const Eigen::Matrix4d &transform_matrix, 
                          const Eigen::Vector3d &vel, 
                          builtin_interfaces::msg::Time time
                        );

    private:
        rclcpp::Node::SharedPtr ros2_node_ = nullptr;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
        
        nav_msgs::msg::Odometry odometry_;
    };

} // namespace robot_localization

#endif