/*
 * @Description: 地图匹配任务管理， 放在类里使代码更清晰
 * @Author: lsc
 * @Date: 
 */

#include "../../include/matching/matching_flow.hpp"
#include "glog/logging.h"
#include "../../include/global_defination/global_defination.h.in"

namespace  robot_localization {
MatchingFlow::MatchingFlow(ros::NodeHandle& nh) 
{   
    std::string config_file_path = ros::package::getPath("robot_localization") + "/config/user_setting.yaml";
    YAML::Node user_node = YAML::LoadFile(config_file_path);
    // 配置消息话题
    std::string imu_raw_data_topic;
    std::string undistrotion_pointcloud_topic;
    std::string imu_link;
    std::string lidar_link;
    std::string car_base_link;
    
    if(user_node["if_simulink"].as<bool>())
    {

    imu_raw_data_topic = user_node["sim_imu_topic"].as<std::string>();
    undistrotion_pointcloud_topic = user_node["sim_scan_pointcloud_topic"].as<std::string>();
    imu_link = user_node["sim_imu_link"].as<std::string>();
    lidar_link = user_node["sim_lidar_link"].as<std::string>();
    car_base_link = user_node["sim_car_base_link"].as<std::string>();
    }else
    {
    imu_raw_data_topic = user_node["imu_topic"].as<std::string>();
    undistrotion_pointcloud_topic = user_node["scan_pointcloud_topic"].as<std::string>();
    imu_link = user_node["imu_link"].as<std::string>();
    lidar_link = user_node["lidar_link"].as<std::string>();
    car_base_link = user_node["car_base_link"].as<std::string>();

    }


    // subscriber:
    // a. 已去畸变的点云（但其实这里还没有）: 
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, undistrotion_pointcloud_topic, 100000);
    // b. lidar pose in map frame:
    gnss_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);

    // publisher:
    // 1. global point cloud map:
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "global_map", "map", 100);
    // 2. local point cloud map:
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "local_map", "map", 100);
    // 3. current scan:
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "current_scan", "map", 100);
    // 4. estimated lidar pose in map frame:
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "laser_localization", "map", "lidar", 100);
    // 5. tf
    laser_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("map", car_base_link);

    matching_ptr_ = std::make_shared<Matching>();
}

bool MatchingFlow::Run() 
{
    if (matching_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers()) {
        CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
        matching_ptr_->GetGlobalMap(global_map_ptr);
        global_map_pub_ptr_->Publish(global_map_ptr);
    }

    if (matching_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers())
        local_map_pub_ptr_->Publish(matching_ptr_->GetLocalMap());

    ReadData();

    while(HasData()) {
        if (!ValidData()) {
            LOG(INFO) << "Invalid data. Skip matching" << std::endl;
            continue;
        }

        if (UpdateMatching()) {
            PublishData();
        }
    }

    return true;
}

bool MatchingFlow::ReadData() 
{
    // pipe lidar measurements and pose into buffer:
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);
    return true;
}

bool MatchingFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    
    if (matching_ptr_->HasInited())
        return true;
    
    if (gnss_data_buff_.size() == 0)
        return false;
        
    return true;
}

bool MatchingFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();

    if (matching_ptr_->HasInited()) {
        cloud_data_buff_.pop_front();
        gnss_data_buff_.clear();
        return true;
    }

    current_gnss_data_ = gnss_data_buff_.front();

    double diff_time = current_cloud_data_.time - current_gnss_data_.time;
    if (diff_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

bool MatchingFlow::UpdateMatching() {
    if (!matching_ptr_->HasInited()) {                //  第一帧点云数据
        //
        // TODO: implement global initialization here
        //
        // Hints: You can use SetGNSSPose & SetScanContextPose from matching.hpp
        //

        // naive implementation:
        /*地图原点初始化    初始化设置 init_pose  为单位阵*/
        // Eigen::Matrix4f init_pose = Eigen::Matrix4f::Identity();          
        
        // matching_ptr_->SetInitPose(init_pose);
        // matching_ptr_->SetInited();
        /*利用ScanContext 进行位姿初始化*/
        // matching_ptr_->SetScanContextPose(current_cloud_data_);
        /*利用GNSS 进行位姿初始化*/
        matching_ptr_->SetGNSSPose(current_gnss_data_.pose);
    }

    return matching_ptr_->Update(current_cloud_data_, laser_odometry_);
}

bool MatchingFlow::PublishData() 
{
    laser_tf_pub_ptr_->SendTransform(laser_odometry_, current_cloud_data_.time);
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);
    current_scan_pub_ptr_->Publish(matching_ptr_->GetCurrentScan());

    return true;
}
}