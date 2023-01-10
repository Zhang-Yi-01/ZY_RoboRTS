/*
 * @Description: tf广播,这🐶ros2还好ros模块改动不算太大
 * @Author: ZY 、 Genshin_Yi
 * @Date: 2022-12-03 15:23:26
 */
#ifndef PUBLISHER_TF_BROADCASTER_HPP_
#define PUBLISHER_TF_BROADCASTER_HPP_

// c++
#include <string>
// ros
#include "rclcpp/rclcpp.hpp"
// tf
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
// eigen
#include <Eigen/Dense>

namespace robot_localization
{
    class TFBroadcaster
    {
    public:
        TFBroadcaster(std::shared_ptr<rclcpp::Node>& node_,std::string frame_id, std::string child_frame_id);
        TFBroadcaster() = default;

        void SendTransform(Eigen::Matrix4d pose, double time);

    protected:
        
        geometry_msgs::msg::TransformStamped transform_;
        // tf2_ros::TransformBroadcaster broadcaster_;
        rclcpp::Node::SharedPtr ros2_node_ = nullptr;
        
        // tf2::Transform transform_;
        // tf::StampedTransform transform_;
        // tf::TransformBroadcaster broadcaster_;
    };

} // namespace robot_localization

#endif