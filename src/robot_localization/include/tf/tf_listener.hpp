/*
 * @Description: tf监听模块,应该只用于监听雷达到imu的转换部分，好像并不需要再转到ros2
 */
#ifndef TF_LISTENER_HPP_
#define TF_LISTENER_HPP_

// c++
#include <string>
// ros
#include "rclcpp/rclcpp.hpp"
// tf
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Transform.h"
// #include <tf/transform_listener.h>
// eigen
#include <Eigen/Dense>

namespace robot_localization
{
    class TFListener
    {
    public:
        TFListener(std::shared_ptr<rclcpp::Node>& node_, std::string base_frame_id, std::string child_frame_id);
        TFListener() = default;

        bool LookUpData(Eigen::Matrix4d &transform_matrix);

    private:
        bool TransformMatrix(const geometry_msgs::msg::TransformStamped &transform, Eigen::Matrix4d &transform_matrix);

    private:
        rclcpp::Node::SharedPtr ros2_node_ = nullptr;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        // tf::TransformListener listener_;
        std::string base_frame_id_;
        std::string child_frame_id_;
    };

} // namespace robot_localization

#endif