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
    bool ImuData::SyncData(
                           std::deque<ImuData> &unsynced_data_buff,
                           std::deque<ImuData> &synced_data_buff,
                           builtin_interfaces::msg::Time cloud_ros2_time,
                           double sync_time
                          )
    {   
        // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
        // 即找到与同步时间相邻的左右两个数据
        // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
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
        synced_data.ros2_time = cloud_ros2_time;// 同步到这个沟ros2时间戳格式，不能往涅
        
        synced_data.linear_acceleration_.x = front_data.linear_acceleration_.x * front_scale + back_data.linear_acceleration_.x * back_scale;
        synced_data.linear_acceleration_.y = front_data.linear_acceleration_.y * front_scale + back_data.linear_acceleration_.y * back_scale;
        synced_data.linear_acceleration_.z = front_data.linear_acceleration_.z * front_scale + back_data.linear_acceleration_.z * back_scale;

        synced_data.angular_velocity_.x = front_data.angular_velocity_.x * front_scale + back_data.angular_velocity_.x * back_scale;
        synced_data.angular_velocity_.y = front_data.angular_velocity_.y * front_scale + back_data.angular_velocity_.y * back_scale;
        synced_data.angular_velocity_.z = front_data.angular_velocity_.z * front_scale + back_data.angular_velocity_.z * back_scale;
        // 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
        // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
        synced_data.orientation_.x = front_data.orientation_.x * front_scale + back_data.orientation_.x * back_scale;
        synced_data.orientation_.y = front_data.orientation_.y * front_scale + back_data.orientation_.y * back_scale;
        synced_data.orientation_.z = front_data.orientation_.z * front_scale + back_data.orientation_.z * back_scale;
        synced_data.orientation_.w = front_data.orientation_.w * front_scale + back_data.orientation_.w * back_scale;
        
        // 线性插值之后要归一化
        synced_data.orientation_.Normlize();

        synced_data_buff.push_back(synced_data);

        return true;
    }

} // namespace robot_localization
