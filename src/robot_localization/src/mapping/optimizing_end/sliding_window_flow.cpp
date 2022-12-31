/*
 * @Description: lio localization backend workflow, implementation
 * @Author: Genshin_Yi
 * @Date: 
 */
#include "../../../include/mapping/optimizing_end/sliding_window_flow.hpp"
#include "glog/logging.h"

#include "../../../include/tools/file_manager.hpp"
#include "../../../include/global_defination/global_defination.h.in"

namespace robot_localization {

SlidingWindowFlow::SlidingWindowFlow(ros::NodeHandle& nh) 
{
    std::string user_config_path = WORK_PACKAGE_PATH + "/config/user_setting.yaml";
    YAML::Node user_node = YAML::LoadFile(user_config_path);
    // 配置用户设置消息话题
    std::string undistrotion_pointcloud_topic;
    std::string lidar_link;
    std::string car_base_link;
    //
    // 优化端需要接收:

    // a. lidar odometry 激光雷达（紧耦合imu）里程计:
    laser_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/fused_odom", 100000);
    // b. map matching odometry 来自图匹配端的位姿:
    map_matching_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/map_matching_odom", 100000);
    // c. IMU measurement, for pre-integration:
    imu_raw_sub_ptr_ = std::make_shared<ImuSubscriber>(nh, "/imu", 1000000);
    imu_synced_sub_ptr_ = std::make_shared<ImuSubscriber>(nh, "/imu", 100000);
    // d. GNSS position:
    // gnss_pose_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);

    //  优化端发布:
    // a. current lidar key frame:
    // key_frame_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh, "/key_frame", "/map", 100);
    // b. current reference GNSS frame:
    // key_gnss_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh, "/key_gnss", "/map", 100);
    // c. optimized odometry 最终优化位姿:
    optimized_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/optimized_odometry", "/map", "/lidar", 100);
    // d. optimized trajectory:
    // optimized_trajectory_pub_ptr_ = std::make_shared<KeyFramesPublisher>(nh, "/optimized_trajectory", "/map", 100);
    // e. lidar frame
    // laser_tf_pub_ptr_ = std::make_shared<TFBroadcaster>("/map", "/velo_link");

    // 优化端任务管理器:
    sliding_window_ptr_ = std::make_shared<SlidingWindow>();
}

bool SlidingWindowFlow::Run() 
{
    // load messages into buffer:
    if ( !ReadData() )
        return false;
    
    while( HasData() ) 
    {
        // 确保所有的测量数据是同步的:
        if ( !ValidData() )
            continue;
        sliding_window_optimizing();
        PublishData();
    }

    return true;
}

bool SlidingWindowFlow::SaveOptimizedTrajectory() 
{
    sliding_window_ptr_ -> SaveOptimizedTrajectory();

    return true;
}

bool SlidingWindowFlow::ReadData() 
{
    // a. lidar odometry:
    laser_odom_sub_ptr_->ParseData(laser_odom_data_buff_);
    // b. map matching odometry:
    map_matching_odom_sub_ptr_->ParseData(map_matching_odom_data_buff_);
    // c. IMU measurement, for pre-integration:
    imu_raw_sub_ptr_->ParseData(imu_raw_data_buff_);
    imu_synced_sub_ptr_->ParseData(imu_synced_data_buff_);
    // d. GNSS position:
    // gnss_pose_sub_ptr_->ParseData(gnss_pose_data_buff_);

    return true;
}

bool SlidingWindowFlow::HasData() 
{
    if (
        laser_odom_data_buff_.empty() ||
        map_matching_odom_data_buff_.empty() ||
        imu_synced_data_buff_.empty()
    ) {
        return false;
    }

    return true;
}

bool SlidingWindowFlow::ValidData() 
{
    current_laser_odom_data_ = laser_odom_data_buff_.front();
    current_map_matching_odom_data_ = map_matching_odom_data_buff_.front();
    current_imu_data_ = imu_synced_data_buff_.front();
    // current_gnss_pose_data_ = gnss_pose_data_buff_.front();

    double diff_map_matching_odom_time = current_laser_odom_data_.time - current_map_matching_odom_data_.time;
    double diff_imu_time = current_laser_odom_data_.time - current_imu_data_.time_stamp_;

    if ( diff_map_matching_odom_time < -0.05 || diff_imu_time < -0.05 ) 
    {
        laser_odom_data_buff_.pop_front();
        return false;
    }

    if ( diff_map_matching_odom_time > 0.05 ) 
    {
        map_matching_odom_data_buff_.pop_front();
        return false;
    }

    if ( diff_imu_time > 0.05 ) 
    {
        imu_synced_data_buff_.pop_front();
        return false;
    }


    laser_odom_data_buff_.pop_front();
    map_matching_odom_data_buff_.pop_front();
    imu_synced_data_buff_.pop_front();
    // gnss_pose_data_buff_.pop_front();

    return true;
}

/**
 * @brief 更新IMU预积分
 * @note  取走的是imu_raw_data_buf中的第一个
 * @param imu_raw_data_buff_.front()
 **/
bool SlidingWindowFlow::UpdateIMUPreIntegration(void) 
{
    while (!imu_raw_data_buff_.empty() && 
            imu_raw_data_buff_.front().time_stamp_ < current_imu_data_.time_stamp_ && 
            sliding_window_ptr_->UpdateIMUPreIntegration(imu_raw_data_buff_.front())) 
    {
        imu_raw_data_buff_.pop_front();
    }

    return true;
}

bool SlidingWindowFlow::sliding_window_optimizing()
{
    static bool odometry_inited = false;
    static Eigen::Matrix4f odom_init_pose = Eigen::Matrix4f::Identity();

    if ( !odometry_inited ) 
    {
        // 地图帧中激光雷达里程计测量帧的到这里的第一次原点为初始姿态:
        // odom_init_pose = current_gnss_pose_data_.pose * current_laser_odom_data_.pose.inverse();
        odom_init_pose = current_laser_odom_data_.pose;
        // odom_init_pose = Eigen::Matrix4f::Identity();
        odometry_inited = true;
    }
    
    // update IMU pre-integration 更新imu预积分:
    UpdateIMUPreIntegration();
    
    // current lidar odometry in map frame:
    current_laser_odom_data_.pose = odom_init_pose * current_laser_odom_data_.pose;

    // optimization is carried out in map frame:
    return sliding_window_ptr_->Update(
                                        current_laser_odom_data_, 
                                        current_map_matching_odom_data_,
                                        current_imu_data_
                                        );
}


bool SlidingWindowFlow::PublishData() 
{
    if ( sliding_window_ptr_->HasNewKeyFrame() ) 
    {        
        KeyFrame key_frame;

        sliding_window_ptr_->GetLatestKeyFrame(key_frame);
        // key_frame_pub_ptr_->Publish(key_frame);

        sliding_window_ptr_->GetLatestKeyGNSS(key_frame);
        // key_gnss_pub_ptr_->Publish(key_frame);
    }

    if ( sliding_window_ptr_->HasNewOptimized() ) 
    {
        KeyFrame key_frame;
        sliding_window_ptr_->GetLatestOptimizedOdometry(key_frame);
        optimized_odom_pub_ptr_->Publish(key_frame.pose.cast<double>(), key_frame.time);

        // publish lidar TF:
        // laser_tf_pub_ptr_->SendTransform(key_frame.pose.cast<double>(), key_frame.time);
    }

    return true;
}

} // namespace robot_localization