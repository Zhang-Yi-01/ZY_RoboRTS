/*
 * @Description: 激光雷达数据订阅
 */

#include "../../include/subscriber/cloud_subscriber.hpp"
// pcl
#include "pcl_conversions/pcl_conversions.h"
using std::placeholders::_1;

namespace robot_localization
{
    /**
     * @brief 点云订阅
     * @note
     * @todo
     **/
    CloudSubscriber::CloudSubscriber(
                                        std::shared_ptr<rclcpp::Node> &node_, 
                                        std::string topic_name, 
                                        size_t buff_size
                                    )
    {
        subscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                        topic_name,
                        buff_size,
                        std::bind(&CloudSubscriber::MsgCallback, this, _1)
                                                                     );

    }

    /**
     * @brief 点云订阅回调函数
     * @note
     * @todo
     **/
    void CloudSubscriber::MsgCallback(const sensor_msgs::msg::PointCloud2 &cloud_msg)
    {
        CloudData cloud_data;
    // ROS2的时间类型 builtin_interfaces::msg::Time 需要把秒和纳秒相加才能表示当前时间
    // 如果获取的是msg时间戳，恢复成完整表达如下
    // double now_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    // auto now_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        cloud_data.time_stamp_ = cloud_msg.header.stamp.sec + cloud_msg.header.stamp.nanosec* 1e-9;

        // cloud_data.time_stamp_ = cloud_msg_ptr->header.stamp.sec + cloud_msg_ptr->header.stamp.nanosec* 1e-9;

        pcl::fromROSMsg(cloud_msg, *(cloud_data.cloud_ptr_));

        new_cloud_data_buff_.push_back(cloud_data);
    }

    /**
     * @brief 解析数据并清除缓冲区
     * @note
     * @todo
     **/
    void CloudSubscriber::ParseData(std::deque<CloudData> &cloud_data_buff)
    {
        if (new_cloud_data_buff_.size() > 0)
        {
            cloud_data_buff.insert(cloud_data_buff.end(),
                                   new_cloud_data_buff_.begin(), new_cloud_data_buff_.end());
            new_cloud_data_buff_.clear();
        }
    }

} // namespace robot_localization
