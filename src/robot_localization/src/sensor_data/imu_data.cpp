/*
 * @Description: 自定义imu数据
 */
#include "../../include/sensor_data/imu_data.hpp"
// c++
// #include <cmath>

namespace robot_localization
{
    /**
     * @brief 四元数-->旋转矩阵
     * @note
     * @todo
     **/
    Eigen::Matrix3d ImuData::OrientationToMatrix() const
    {
        Eigen::Quaterniond q(orientation_.w, orientation_.x, orientation_.y, orientation_.z);
        Eigen::Matrix3d matrix = q.matrix().cast<double>();
        return matrix;
    }

    /**
     * @brief 时间同步
     * @note 时间戳为ms+ns
     * @todo try Slerp(Sopherical Liner Interpolation)
     **/
    bool ImuData::SyncData(std::deque<ImuData> &unsynced_data_buff, std::deque<ImuData> &synced_data_buff, double sync_time)
    {
        while (unsynced_data_buff.size() >= 2)
        {
            // 异常1：sync_time>[0]>[1]
            if (unsynced_data_buff.at(0).time_stamp_ > sync_time)
                return false;
            // 异常2：[0]>[1]>sync_time
            if (unsynced_data_buff.at(1).time_stamp_ < sync_time)
            {
                unsynced_data_buff.pop_front();
                continue;
            }
            // 异常3：[0]>>sync_time>[1]
            if (sync_time - unsynced_data_buff.at(0).time_stamp_ > 0.2)
            {
                unsynced_data_buff.pop_front();
                break;
            }
            // 异常4：[0]>sync_time>>[1]
            if (unsynced_data_buff.at(1).time_stamp_ - sync_time > 0.2)
            {
                unsynced_data_buff.pop_front();
                break;
            }
            break;
        }

        if (unsynced_data_buff.size() < 2)
            return false;

        ImuData front_data = unsynced_data_buff.at(0);
        ImuData back_data = unsynced_data_buff.at(1);
        ImuData synced_data;

        // 线性插值
        double front_scale = (back_data.time_stamp_ - sync_time) / (back_data.time_stamp_ - front_data.time_stamp_);
        double back_scale = (sync_time - front_data.time_stamp_) / (back_data.time_stamp_ - front_data.time_stamp_);

        synced_data.time_stamp_ = sync_time;
        
        synced_data.linear_acceleration_.x = front_data.linear_acceleration_.x * front_scale + back_data.linear_acceleration_.x * back_scale;
        synced_data.linear_acceleration_.y = front_data.linear_acceleration_.y * front_scale + back_data.linear_acceleration_.y * back_scale;
        synced_data.linear_acceleration_.z = front_data.linear_acceleration_.z * front_scale + back_data.linear_acceleration_.z * back_scale;

        synced_data.angular_velocity_.x = front_data.angular_velocity_.x * front_scale + back_data.angular_velocity_.x * back_scale;
        synced_data.angular_velocity_.y = front_data.angular_velocity_.y * front_scale + back_data.angular_velocity_.y * back_scale;
        synced_data.angular_velocity_.z = front_data.angular_velocity_.z * front_scale + back_data.angular_velocity_.z * back_scale;

        synced_data.orientation_.x = front_data.orientation_.x * front_scale + back_data.orientation_.x * back_scale;
        synced_data.orientation_.y = front_data.orientation_.y * front_scale + back_data.orientation_.y * back_scale;
        synced_data.orientation_.z = front_data.orientation_.z * front_scale + back_data.orientation_.z * back_scale;
        synced_data.orientation_.w = front_data.orientation_.w * front_scale + back_data.orientation_.w * back_scale;
        
        // 归一化
        synced_data.orientation_.Normlize();

        synced_data_buff.push_back(synced_data);

        return true;
    }

} // namespace robot_localization
