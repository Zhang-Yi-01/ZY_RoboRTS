/*
 * @Description: imu数据订阅
 * @Author: ZY
 * @Date: 2022.10.24
 */
#include "../../include/subscriber/imu_subscriber.hpp"

namespace robot_localization
{
    /**
     * @brief IMU订阅初始化
     * @note
     * @todo
     **/
    ImuSubscriber::ImuSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size)
        : nh_(nh)
    {
        subscriber_ = nh_.subscribe(topic_name, buff_size, &ImuSubscriber::MsgCallback, this);
    }

    /**
     * @brief IMU订阅回调函数
     * @note
     * @todo
     **/
    void ImuSubscriber::MsgCallback(const sensor_msgs::ImuConstPtr &imu_msg_ptr)
    {
        ImuData imu_data;
        imu_data.time_stamp_ = imu_msg_ptr->header.stamp.toSec();

        imu_data.linear_acceleration_.x = imu_msg_ptr->linear_acceleration.x;
        imu_data.linear_acceleration_.y = imu_msg_ptr->linear_acceleration.y;
        imu_data.linear_acceleration_.z = imu_msg_ptr->linear_acceleration.z;

        imu_data.angular_velocity_.x = imu_msg_ptr->angular_velocity.x;
        imu_data.angular_velocity_.y = imu_msg_ptr->angular_velocity.y;
        imu_data.angular_velocity_.z = imu_msg_ptr->angular_velocity.z;

        imu_data.orientation_.x = imu_msg_ptr->orientation.x;
        imu_data.orientation_.y = imu_msg_ptr->orientation.y;
        imu_data.orientation_.z = imu_msg_ptr->orientation.z;
        imu_data.orientation_.w = imu_msg_ptr->orientation.w;

        new_imu_data_buff_.push_back(imu_data);
    }

    /**
     * @brief 读取缓冲区并清空
     * @note
     * @todo
     **/
    void ImuSubscriber::ParseData(std::deque<ImuData> &imu_data_buff)
    {
        if (new_imu_data_buff_.size() > 0)
        {
            imu_data_buff.insert(imu_data_buff.end(), new_imu_data_buff_.begin(), new_imu_data_buff_.end());
            new_imu_data_buff_.clear();
        }
    }

} // namespace robot_localization
