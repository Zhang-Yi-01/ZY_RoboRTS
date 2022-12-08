/*
 * @Description: 激光雷达数据订阅
 * @Author: ZY
 * @Date: 2022.10.24
 */

#include "../../include/subscriber/cloud_subscriber.hpp"
// pcl
#include "pcl_conversions/pcl_conversions.h"

namespace robot_localization
{
    /**
     * @brief 点云订阅
     * @note
     * @todo
     **/
    CloudSubscriber::CloudSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size)
        : nh_(nh)
    {
        subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::MsgCallback, this);
    }

    /**
     * @brief 点云订阅回调函数
     * @note
     * @todo
     **/
    void CloudSubscriber::MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr)
    {
        CloudData cloud_data;
        cloud_data.time_stamp_ = cloud_msg_ptr->header.stamp.toSec();
        pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr_));

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
