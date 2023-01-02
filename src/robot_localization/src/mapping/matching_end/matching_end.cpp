/*
 * @Description: 里程计端里程计算法
 * @Author: Genshin_Yi
 * @Date: 
 */
#include "../../../include/mapping/matching_end/matching_end.hpp"
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"
// #include "../../../include/global_defination/global_defination.h.in"
#include "../../../include/models/registration/ndt_registration.hpp"
#include "../../../include/models/cloud_filter/voxel_filter.hpp"
#include "../../../include/models/cloud_filter/no_filter.hpp"
// #include "ros/package.h"

namespace  robot_localization {
    
/**
 * @brief 构造函数会为matching任务管理器配置yaml参数内容，以及加载全局点云pcd文件并更新一次局部地图
 * @note
 * @param
 **/
Matching::Matching()
        : global_map_ptr_  (new CloudData::CLOUD()),
          local_map_ptr_   (new CloudData::CLOUD()),
          current_scan_ptr_(new CloudData::CLOUD()) 
{
    
    InitWithConfig();

    InitGlobalMap();

    ResetLocalMap(0.00, 0.00, 0.00);
}

/**
 * @brief 由config/params/matching.yaml配置参数
 * @note
 * @param
 **/
bool Matching::InitWithConfig() 
{
    
    std::string config_file_path = WORK_PACKAGE_PATH + "/config/params/matching.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    LOG(INFO) << std::endl
              << "-----------------Init Matching With Config-------------------" 
              << std::endl;

    std::string pcd_map_path = WORK_PACKAGE_PATH + "/pcd_map/scans.pcd";

    InitDataPath(pcd_map_path);// pcd点云地图文件路径👈定

    // InitScanContextManager(config_node);//这个回环部分，是不是也可以要呢，我Genshin_Yi先注释了
    InitRegistration(registration_ptr_, config_node);

    // a. global map filter -- downsample point cloud map for visualization:
    InitFilter("global_map", global_map_filter_ptr_, config_node);
    // b. local map filter -- downsample & ROI filtering for scan-map matching:
    InitBoxFilter(config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    // c. scan filter -- 
    InitFilter("frame", frame_filter_ptr_, config_node);

    return true;
}


/**
 * @brief pcd点云地图文件路径设置
 * @param string pcd_map_path
 **/
bool Matching::InitDataPath(const std::string pcd_map_path) 
{   
    map_path_ = pcd_map_path;

    return true;
}

// bool Matching::InitScanContextManager(const YAML::Node& config_node) 
// {
//     // get loop closure config:
//     loop_closure_method_ = config_node["loop_closure_method"].as<std::string>();

//     // create instance:
//     scan_context_manager_ptr_ = std::make_shared<ScanContextManager>(config_node[loop_closure_method_]);

//     // load pre-built index:
//     scan_context_path_ = config_node["scan_context_path"].as<std::string>();
//     scan_context_manager_ptr_->Load(scan_context_path_);

//     return true;
// }

bool Matching::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    std::cout << "\tPoint Cloud Registration Method: " << registration_method << std::endl;

    if (registration_method == "NDT") 
    {
        registration_ptr = std::make_shared<NdtRegistration>(config_node[registration_method]);
    } else 
    {
        LOG(ERROR) << "Registration method " << registration_method << " NOT FOUND!";
        return false;
    }

    return true;
}

bool Matching::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    std::cout << "\tFilter Method for " << filter_user << ": " << filter_mothod << std::endl;

    if (filter_mothod == "voxel_filter") 
    {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else if (filter_mothod == "no_filter") 
    {
        filter_ptr = std::make_shared<NoFilter>();
    } else {
        LOG(ERROR) << "Filter method " << filter_mothod << " for " << filter_user << " NOT FOUND!";
        return false;
    }

    return true;
}

bool Matching::InitBoxFilter(const YAML::Node& config_node) 
{
    box_filter_ptr_ = std::make_shared<BoxFilter>(config_node);
    return true;
}

/**
 * @brief 读取点云pcd文件，获取全局地图
 * @note
 * @param map_path_
 **/
bool Matching::InitGlobalMap() 
{
    pcl::io::loadPCDFile(map_path_, *global_map_ptr_);
    LOG(INFO) << "Load global map, size:" << global_map_ptr_->points.size();

    // 由于用了scan-map匹配，所以这里将相同的过滤器应用于local map & scan:
    local_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);
    LOG(INFO) << "Filtered global map, size:" << global_map_ptr_->points.size();

    has_new_global_map_ = true;

    return true;
}

/**
 * @brief 更新局部地图，并将局部地图导入registration匹配模块
 * @note
 * @param 
 **/
bool Matching::ResetLocalMap(float x, float y, float z) 
{
    std::vector<float> origin = {x, y, z};

    // 使用ROI过滤进行局部地图分割：
    box_filter_ptr_->SetOrigin(origin);
    box_filter_ptr_->Filter(global_map_ptr_, local_map_ptr_);

    registration_ptr_->SetInputTarget(local_map_ptr_);

    has_new_local_map_ = true;

    std::vector<float> edge = box_filter_ptr_->GetEdge();
    LOG(INFO) << "New local map:" << edge.at(0) << ","
                                  << edge.at(1) << ","
                                  << edge.at(2) << ","
                                  << edge.at(3) << ","
                                  << edge.at(4) << ","
                                  << edge.at(5) << std::endl << std::endl;

    return true;
}

bool Matching::Update(const CloudData& cloud_data, Eigen::Matrix4d& cloud_pose) 
{
    static Eigen::Matrix4d step_pose = Eigen::Matrix4d::Identity();
    static Eigen::Matrix4d last_pose = init_pose_;
    static Eigen::Matrix4d predict_pose = init_pose_;

    // 删除nan值无效点:
    std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, indices);
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr_, *cloud_data.cloud_ptr_, indices);

    // 降采样
    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    frame_filter_ptr_->Filter(cloud_data.cloud_ptr_, filtered_cloud_ptr);


    // matching:
    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
    static Eigen::Matrix4d scan_match_result_pose = Eigen::Matrix4d::Identity();
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, scan_match_result_pose);
    cloud_pose=scan_match_result_pose;
    pcl::transformPointCloud(*cloud_data.cloud_ptr_, *current_scan_ptr_, cloud_pose);

    // update predicted pose:
    step_pose = last_pose.inverse() * cloud_pose;
    predict_pose = cloud_pose * step_pose;
    last_pose = cloud_pose;

    // 匹配之后判断是否需要更新局部地图,这一部分是图匹配的实现部分
    std::vector<float> edge = box_filter_ptr_->GetEdge();
    for (int i = 0; i < 3; i++) 
    {
        if ( fabs(cloud_pose(i, 3) - edge.at(2 * i)) > 50.0 &&
             fabs(cloud_pose(i, 3) - edge.at(2 * i + 1)) > 50.0) 
        {
            continue;
        }
            
        ResetLocalMap(cloud_pose(0,3), cloud_pose(1,3), cloud_pose(2,3));
        break;
    }

    return true;
}


/**
 * @brief  get init pose using scan context matching
 * @param  init_scan, init key scan
 * @return true if success otherwise false
 */
// TODO: understand this function
// bool Matching::SetScanContextPose(const CloudData& init_scan) {                      //    利用闭环检测，找到定位的初始位姿
//     // get init pose proposal using scan context match:
//     Eigen::Matrix4d init_pose =  Eigen::Matrix4d::Identity();                                             //    初始化位姿为单位阵

//     if (
//         !scan_context_manager_ptr_->DetectLoopClosure(init_scan, init_pose)
//     ) {
//         return false;
//     }

//     // set init pose:
//     SetInitPose(init_pose);
    
//     return true;
// }

// TODO: understand this function                   
bool Matching::SetInitPose(const Eigen::Matrix4d& init_pose) 
{                              // 设置定位的初始位姿，根据此位姿可以找到定位需要用到的局部地图；这个位姿可以通过回环检测得到
    init_pose_ = init_pose;
    ResetLocalMap(init_pose(0,3), init_pose(1,3), init_pose(2,3));                 //  定位需要用到的局部地图，通过获取(x,y,z) 更新local map
    
    return true;
}

bool Matching::SetInited(void) 
{
    has_inited_ = true;

    return true;
}

Eigen::Matrix4d Matching::GetInitPose(void) 
{
    return init_pose_;
}


void Matching::GetGlobalMap(CloudData::CLOUD_PTR& global_map) 
{
    // downsample global map for visualization:
    global_map_filter_ptr_->Filter(global_map_ptr_, global_map);

    has_new_global_map_ = false;
}

CloudData::CLOUD_PTR& Matching::GetLocalMap() 
{
    return local_map_ptr_;
}

CloudData::CLOUD_PTR& Matching::GetCurrentScan() 
{
    return current_scan_ptr_;
}

bool Matching::HasInited() 
{
    return has_inited_;
}

/**
 * @brief  判断全局地图是否是刚刚更新
 * @return true or false
 */
bool Matching::HasNewGlobalMap() 
{
    return has_new_global_map_;
}

bool Matching::HasNewLocalMap() 
{
    return has_new_local_map_;
}

}