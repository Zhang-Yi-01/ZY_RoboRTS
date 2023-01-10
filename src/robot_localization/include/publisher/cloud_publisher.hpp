/*
 * @Description: 点云发布
 * @Author: ZY
 * @Date: 
 */
#ifndef PUBLISHER_CLOUD_PUBLISHER_HPP_
#define PUBLISHER_CLOUD_PUBLISHER_HPP_

// ros
#include "sensor_msgs/msg/point_cloud2.hpp"
// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// sensor_data
#include "../sensor_data/cloud_data.hpp"

namespace robot_localization
{
    class CloudPublisher
    {
    public:
        CloudPublisher(std::shared_ptr<rclcpp::Node> &node_,
                       std::string topic_name,
                       std::string frame_id,
                       size_t buff_size);
        CloudPublisher() = default;

        void Publish(CloudData::CLOUD_PTR &cloud_ptr_input, double time);
        void Publish(CloudData::CLOUD_PTR &cloud_ptr_input);

        bool HasSubscribers();

    private:
        void PublishData(CloudData::CLOUD_PTR &cloud_ptr_input, builtin_interfaces::msg::Time time);
        rclcpp::Node::SharedPtr ros2_node_ = nullptr;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
        std::string frame_id_;
    };

} // namespace robot_localization

#endif