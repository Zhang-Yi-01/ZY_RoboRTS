/*
 * @Description: 地图匹配任务管理
 */
#include "../../../include/mapping/matching_end/matching_end_flow.hpp"
#include "glog/logging.h"

namespace  robot_localization 
{
MatchingFlow::MatchingFlow(std::shared_ptr<rclcpp::Node>& node_) 
{   
    std::string user_config_path = WORK_PACKAGE_PATH + "/config/user_setting.yaml";
    YAML::Node user_node = YAML::LoadFile(user_config_path);
    // 配置用户设置消息话题
    std::string undistrotion_pointcloud_topic;
    std::string lidar_link;
    std::string car_base_link;
    
    if(user_node["if_simulink"].as<bool>())
    {
 
    undistrotion_pointcloud_topic = user_node["sim_scan_pointcloud_topic"].as<std::string>();
    lidar_link = user_node["sim_lidar_link"].as<std::string>();
    car_base_link = user_node["sim_car_base_link"].as<std::string>();
    }else
    {
    undistrotion_pointcloud_topic = user_node["scan_pointcloud_topic"].as<std::string>();
    lidar_link = user_node["lidar_link"].as<std::string>();
    car_base_link = user_node["car_base_link"].as<std::string>();

    }
    if_matching_end_tf_broadcast=user_node["matching_end_send_tf"].as<bool>();

    // 订阅:
    // 已去畸变的点云（但其实这里还没有）: 
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(node_, undistrotion_pointcloud_topic, 100000);

    // 发布:
    // 1. global point cloud map:
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(node_, "global_map", "map", 100);
    // 2. local point cloud map:
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(node_, "local_map", "map", 100);
    // 3. current scan:
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(node_, "current_scan", "map", 100);
    // 4. estimated lidar pose in map frame:
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(node_, "map_matching_odom", "odom", "lidar", 100);
    // 5. tf
    laser_tf_pub_ptr_ = std::make_shared<TFBroadcaster>(node_,"odom", car_base_link);

    //matching任务管理器，构造函数会为其配置yaml参数内容，以及加载全局点云pcd文件并更新一次局部地图
    matching_ptr_ = std::make_shared<Matching>();
}

bool MatchingFlow::Run() 
{
    if (matching_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers()) 
    {
        CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
        matching_ptr_->GetGlobalMap(global_map_ptr);
        global_map_pub_ptr_->Publish(global_map_ptr);
    }

    if (matching_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers())
        local_map_pub_ptr_->Publish(matching_ptr_->GetLocalMap());

    ReadData();
    // yellow_info("ReadDate Done");
    while(HasData()) 
    {
        if (!ValidData())
        {
            LOG(INFO) << "Invalid data. Skip matching" << std::endl;
            continue;
        }
        // yellow_info("Begin UpdateMatching");

        if (UpdateMatching()) 
        {
            PublishData();
        }
    }

    return true;
}

bool MatchingFlow::ReadData() 
{
    // 将激光雷达测量结果送入缓冲区
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    // gnss_sub_ptr_->ParseData(gnss_data_buff_);
    return true;
}

bool MatchingFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    
    if (matching_ptr_->HasInited())
        return true;
    
        
    return true;
}

bool MatchingFlow::ValidData() 
{
    current_cloud_data_ = cloud_data_buff_.front();

    if (matching_ptr_->HasInited()) 
    {
        cloud_data_buff_.pop_front();

        return true;
    }

    // double diff_time = current_cloud_data_.time - current_gnss_data_.time;
    // if (diff_time < -0.05) 
    // {
    //     cloud_data_buff_.pop_front();
    //     return false;
    // }

    cloud_data_buff_.pop_front();

    return true;
}

/**
 * @brief 更新匹配结果
 * @param 
 **/
bool MatchingFlow::UpdateMatching() 
{
    if (!matching_ptr_->HasInited())    // 第一帧点云数据，在此处实现位姿全局初始化
    {                
        // TODO: implement global initialization here
        //
        // Hints: You can use SetScanContextPose from matching.hpp
        // 选用ScanContext或者原点进行位姿初始化：

        /*地图原点初始化，置 init_pose  为单位阵*/// 原注释此为天真（naive）的方法
        Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();     

        matching_ptr_->SetInitPose(init_pose);
        
        /*利用ScanContext 进行位姿初始化*/
        // matching_ptr_->SetScanContextPose(current_cloud_data_);

        matching_ptr_->SetInited();
    }

    return matching_ptr_->Update(current_cloud_data_, laser_odometry_);
}

bool MatchingFlow::PublishData() 
{   
    if(if_matching_end_tf_broadcast)    
        laser_tf_pub_ptr_->SendTransform(laser_odometry_, current_cloud_data_.ros2_time);

    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.ros2_time);
    current_scan_pub_ptr_->Publish(matching_ptr_->GetCurrentScan());

    return true;
}
}