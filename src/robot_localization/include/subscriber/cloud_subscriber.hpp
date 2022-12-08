/*
 * @Description: 激光雷达数据订阅
 * @Author: ZY
 * @Date: 2022.10.24
 */
#ifndef CLOUD_SUBSCRIBER_HPP
#define CLOUD_SUBSCRIBER_HPP

// c++
#include <deque>
// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//自定义点云数据类型
#include "../sensor_data/cloud_data.hpp"

namespace robot_localization
{
    class CloudSubscriber
    {
    public:
        CloudSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
        CloudSubscriber() = default;
        void ParseData(std::deque<CloudData> &cloud_data_buff);

    private:
        void MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<CloudData> new_cloud_data_buff_;
    };

} // namespace robot_localization

#endif