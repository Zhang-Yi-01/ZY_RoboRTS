/*
 * @Description: 自定义imu数据
 */
#ifndef SENSOR_DATA_IMU_DATA_HPP_
#define SENSOR_DATA_IMU_DATA_HPP_

#include <deque>
#include <cmath>
#include <Eigen/Dense>
#include "rclcpp/time.hpp"

namespace robot_localization
{
    class ImuData
    {
    public:
        struct LinerAcceleration
        {
            double x = 0.0, y = 0.0, z = 0.0;
        };
        struct AngularVelocity
        {
            double x = 0.0, y = 0.0, z = 0.0;
        };
        struct AccelBias 
        {
            double x = 0.0 ; double y = 0.0 ; double z = 0.0;
        };
        struct GyroBias 
        { 
            double x = 0.0; double y = 0.0; double z = 0.0;
        };


        class Orientation
        {
        public:
            double x = 0.0, y = 0.0, z = 0.0, w = 0.0;

        public:
            void Normlize()
            {
                double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
                x /= norm;
                y /= norm;
                z /= norm;
                w /= norm;
            }
        };

        double time_stamp_ = 0.0;
        builtin_interfaces::msg::Time ros2_time = rclcpp::Time(0,0);

        LinerAcceleration linear_acceleration_;
        AngularVelocity angular_velocity_;
        AccelBias accel_bias;
        GyroBias gyro_bias;



        Orientation orientation_;

    public:
        Eigen::Matrix3d OrientationToMatrix() const;
        static bool SyncData(std::deque<ImuData> &unsynced_data_buff,
                             std::deque<ImuData> &synced_data_buff,
                             builtin_interfaces::msg::Time cloud_ros2_time,
                             double sync_time);
    };

} // namespace robot_localization

#endif