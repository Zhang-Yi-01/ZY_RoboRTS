/*
 * @Description: imu数据订阅
 * @Author: ZY
 * @Date: 2022.10.24
 */
#ifndef IMU_SUBSCRIBER_HPP_
#define IMU_SUBSCRIBER_HPP_

// c++
// #include <deque>
// ros
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
// 自定义imu消息类型
#include "../sensor_data/imu_data.hpp"

namespace robot_localization
{
    class ImuSubscriber
    {
    public:
        ImuSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
        ImuSubscriber() = default;
        void ParseData(std::deque<ImuData> &imu_data_buff);

    private:
        void MsgCallback(const sensor_msgs::ImuConstPtr &imu_msg_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<ImuData> new_imu_data_buff_;
    };

} // namespace robot_localization

#endif