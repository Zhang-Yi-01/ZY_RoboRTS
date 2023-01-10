/*
 * @Description: 激光雷达数据订阅
 */
#ifndef CLOUD_SUBSCRIBER_HPP
#define CLOUD_SUBSCRIBER_HPP

// c++
#include <deque>
// ros
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
//自定义点云数据类型
#include "../sensor_data/cloud_data.hpp"

namespace robot_localization
{
    class CloudSubscriber
    {
    public:
        CloudSubscriber(std::shared_ptr<rclcpp::Node> &node_, std::string topic_name, size_t buff_size);
        CloudSubscriber() = default;
        void ParseData(std::deque<CloudData> &cloud_data_buff);

    private:
        void MsgCallback(const sensor_msgs::msg::PointCloud2 &cloud_msg);

    private:

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;

        std::deque<CloudData> new_cloud_data_buff_;
    };

} // namespace robot_localization

#endif