/*
 * @Description: 存放处理后的IMU姿态,想了想认为还是加过来合适
 */
#ifndef SENSOR_DATA_POSE_DATA_HPP_
#define SENSOR_DATA_POSE_DATA_HPP_


#include "rclcpp/time.hpp"
#include <Eigen/Dense>


namespace robot_localization {

class PoseData 
{
  public:
    double time = 0.0;
    builtin_interfaces::msg::Time ros2_time = rclcpp::Time(0,0);

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

    struct 
    {
      Eigen::Vector3f v = Eigen::Vector3f::Zero();
      Eigen::Vector3f w = Eigen::Vector3f::Zero();
    } vel;
    
  public:
    Eigen::Quaternionf GetQuaternion();

};

}

#endif