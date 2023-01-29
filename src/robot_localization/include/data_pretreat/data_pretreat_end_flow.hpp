/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等,也是对应感知的接口
 */
#ifndef DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_
#define DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_

#include "rclcpp/rclcpp.hpp"
// subscriber
#include "../subscriber/cloud_subscriber.hpp"
#include "../subscriber/imu_subscriber.hpp"


#include "../tf/tf_listener.hpp"

// publisher
// a. synced lidar measurement
#include "../publisher/cloud_publisher.hpp"
// b. synced IMU measurement
#include "../publisher/imu_publisher.hpp"
#include "../publisher/odometry_publisher.hpp"

#include "../subscriber/odometry_subscriber.hpp"
// 去畸变处理
#include "../models/scan_distortion_adjust/distortion_adjust.hpp"

namespace robot_localization {
class DataPretreatFlow {
  public:
    DataPretreatFlow(std::shared_ptr<rclcpp::Node>& node_);

    bool Run();

  private:
    bool ReadData();
    bool InitCalibration();

    bool HasData();
    bool ValidData();
    bool TransformData();
    bool PublishData();

  private:
    // subscriber
    // a. lidar odometry:
    std::shared_ptr<OdometrySubscriber> fused_odom_sub_ptr_;
    std::deque<PoseData> fused_odom_data_buff_;
    PoseData current_fused_odom_data_;

    std::shared_ptr<ImuSubscriber> imu_sub_ptr_;
    
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;

    std::shared_ptr<TFListener> lidar_to_imu_ptr_;
    // publisher
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<IMUPublisher> imu_pub_ptr_;
    
    // models
    std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;

    Eigen::Matrix4d lidar_to_imu_ = Eigen::Matrix4d::Identity();

    std::deque<CloudData> cloud_data_buff_;
    std::deque<ImuData> imu_data_buff_;

    CloudData current_cloud_data_;
    ImuData current_imu_data_;


};
}

#endif