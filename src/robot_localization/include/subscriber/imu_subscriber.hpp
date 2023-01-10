/*
 * @Description: imu数据订阅
 */
#ifndef IMU_SUBSCRIBER_HPP_
#define IMU_SUBSCRIBER_HPP_

// c++
// #include <deque>
// ros
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
// 自定义imu消息类型
#include "../sensor_data/imu_data.hpp"

namespace robot_localization
{
    class ImuSubscriber
    {
    public:
        ImuSubscriber(std::shared_ptr<rclcpp::Node>& node_,std::string topic_name,size_t buff_size);
        ImuSubscriber() = default;
        void ParseData(std::deque<ImuData> &imu_data_buff);

    private:
        void MsgCallback(const sensor_msgs::msg::Imu& imu_msg_ptr);

    private:
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;

        std::deque<ImuData> new_imu_data_buff_;
    };

} // namespace robot_localization

#endif