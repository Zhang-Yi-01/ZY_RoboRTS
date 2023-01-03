/*
 * @Description: 在ros中发布IMU数据
 * @Author: Genshin_Yi
 * @Date: 
 */
#ifndef PUBLISHER_IMU_PUBLISHER_HPP_
#define PUBLISHER_IMU_PUBLISHER_HPP_

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "../sensor_data/imu_data.hpp"

namespace robot_localization {
class IMUPublisher {
  public:
    IMUPublisher(
      ros::NodeHandle& nh,
      std::string topic_name,
      std::string frame_id,
      size_t buff_size
    );
    IMUPublisher() = default;

    void Publish(const ImuData &imu_data, double time);
    void Publish(const ImuData &imu_data);

    bool HasSubscribers(void);

  private:
    void PublishData(const ImuData &imu_data, ros::Time time);

    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;

    sensor_msgs::Imu imu_;
};
} 
#endif