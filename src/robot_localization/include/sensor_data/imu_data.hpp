/*
 * @Description: 自定义imu数据
 * @Author: ZY
 * @Date: 2022.10.24
 */
#ifndef CLOUD_DATA_HPP_
#define CLOUD_DATA_HPP_

#include <deque>
#include <cmath>
#include <Eigen/Dense>

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
        LinerAcceleration linear_acceleration_;
        AngularVelocity angular_velocity_;
        Orientation orientation_;

    public:
        Eigen::Matrix3d OrientationToMatrix() const;
        static bool SyncData(std::deque<ImuData> &unsynced_data_buff,
                             std::deque<ImuData> &synced_data_buff,
                             double sync_time);
    };

} // namespace robot_localization

#endif