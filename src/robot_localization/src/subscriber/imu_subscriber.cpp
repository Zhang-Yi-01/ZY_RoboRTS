/*
 * @Description: imu数据订阅
 */
#include "../../include/subscriber/imu_subscriber.hpp"

using std::placeholders::_1;

namespace robot_localization
{
    /**
     * @brief IMU订阅初始化
     * @note
     * @todo
     **/
    ImuSubscriber::ImuSubscriber(
                                std::shared_ptr<rclcpp::Node>& node_, 
                                std::string topic_name, 
                                size_t buff_size)
    {   
        subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
                        topic_name,
                        buff_size,
                        std::bind(&ImuSubscriber::MsgCallback, this, _1)
                                                                     );
        
    }

    /**
     * @brief IMU订阅回调函数
     * @note
     * @todo
     **/
    void ImuSubscriber::MsgCallback(const sensor_msgs::msg::Imu& imu_msg)
    {
        ImuData imu_data;
    // ROS2的时间类型 builtin_interfaces::msg::Time 需要把秒和纳秒相加才能表示当前时间
    // 如果获取的是msg时间戳，恢复成完整表达如下
    // double now_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    // auto now_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        imu_data.ros2_time = imu_msg.header.stamp;
        imu_data.time_stamp_ = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec* 1e-9;
        
        imu_data.linear_acceleration_.x = imu_msg.linear_acceleration.x;
        imu_data.linear_acceleration_.y = imu_msg.linear_acceleration.y;
        imu_data.linear_acceleration_.z = imu_msg.linear_acceleration.z;

        imu_data.angular_velocity_.x = imu_msg.angular_velocity.x;
        imu_data.angular_velocity_.y = imu_msg.angular_velocity.y;
        imu_data.angular_velocity_.z = imu_msg.angular_velocity.z;

        imu_data.orientation_.x = imu_msg.orientation.x;
        imu_data.orientation_.y = imu_msg.orientation.y;
        imu_data.orientation_.z = imu_msg.orientation.z;
        imu_data.orientation_.w = imu_msg.orientation.w;

        // static bool in_or_not=false;
        // if(in_or_not==false)
        // {
        //     in_or_not=true;
        //     new_imu_data_buff_.push_back(imu_data);
        // }
        // else{
        //     in_or_not=false;
        // }
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
