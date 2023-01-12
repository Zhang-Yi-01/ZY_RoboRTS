/*
 * @Description: 里程计端任务管理器
 * @Author: ZY
 * @Date: 
 */
#ifndef LIDAR_ODOM_END_FLOW
#define LIDAR_ODOM_END_FLOW

// ros
// #include <ros/package.h>
// glog
// #include <glog/logging.h>
// yaml
// subscriber
#include "../../subscriber/cloud_subscriber.hpp"
#include "../../subscriber/imu_subscriber.hpp"
// publisher
#include "../../publisher/odometry_publisher.hpp"
#include "../../publisher/cloud_publisher.hpp"
// tf listener
#include "../../tf/tf_listener.hpp"
// tf broadcaster
#include "../../tf/tf_broadcaster.hpp"
// sensor_data
// #include "../../sensor_data/imu_data.hpp"
// 里程计端算法
#include "lidar_odom_end.hpp"

namespace robot_localization
{
    class LidarOdomEndFlow
    {
    public:
        LidarOdomEndFlow(std::shared_ptr<rclcpp::Node>& node_);

        bool Run();

    private:
        bool ReadData();
        bool HasInited();
        bool HasData();

        bool HasIMUData(void) const
        {
            if (!imu_raw_data_buff_.empty())
            {
                double diff_filter_time = current_imu_raw_data_.time_stamp_ - lidar_odom_end_ptr_->GetTime();

                if (diff_filter_time <= 0.01)
                {
                    return true;
                }
            }

            return false;
        }

        bool HasLidarData(void) const
        {
            return (!cloud_data_buff_.empty() && !imu_synced_data_buff_.empty());
        }

        bool HasIMUComesFirst(void) const
        {
            return imu_raw_data_buff_.front().time_stamp_ < cloud_data_buff_.front().time_stamp_;
        }

        bool ValidIMUData();
        bool ValidLidarData();

        bool InitCalibration();
        bool InitLocalization();

        bool Predict();
        bool Correct();

        bool PublishGlobalMap();
        bool PublishLocalMap();
        bool PublishLidarOdom();
        bool PublishFusionOdom();

    private:
        // 订阅
        // 1.IMU raw data
        std::shared_ptr<ImuSubscriber> imu_raw_sub_ptr_;
        std::deque<ImuData> imu_raw_data_buff_;
        // 2.IMU synced data
        std::shared_ptr<ImuSubscriber> imu_synced_sub_ptr_;
        std::deque<ImuData> imu_synced_data_buff_;
        // 3.lidar
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::deque<CloudData> cloud_data_buff_;
        // 4.tf
        std::shared_ptr<TFListener> lidar_to_imu_ptr_;
        Eigen::Matrix4d lidar_to_imu_ = Eigen::Matrix4d::Identity();

        // 发布
        // 1.global-local map and current scan:
        std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
        std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
        std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
        // 2.odometry
        std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
        std::shared_ptr<OdometryPublisher> fused_odom_pub_ptr_;
        // 3.tf
        std::shared_ptr<TFBroadcaster> laser_tf_pub_ptr_;

        // 当前帧数据
        CloudData current_cloud_data_;
        ImuData current_imu_raw_data_;
        ImuData current_imu_synced_data_;

        // 里程计端算法
        std::shared_ptr<LidarOdomEnd> lidar_odom_end_ptr_;

        // 里程计信息
        Eigen::Matrix4d odometry_ = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d laser_pose_ = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d fused_pose_ = Eigen::Matrix4d::Identity();
        Eigen::Vector3d fused_vel_ = Eigen::Vector3d::Zero();

        bool if_odom_end_tf_broadcast = false;
    };

} // namespace robot_localization

#endif
