/*
 * @Description: 地图匹配定位算法
 */
#ifndef LOCALIZATION_MATCHING_MATCHING_HPP_
#define LOCALIZATION_MATCHING_MATCHING_HPP_

#include <deque>

#include "../../models/registration/registration_interface.hpp" //包含CloudDate
// #include "../../models/cloud_filter/cloud_filter_interface.hpp"
#include "../../models/cloud_filter/box_filter.hpp"
#include "../../global_path_defination/global_path.h"

// #include "robot_localization/models/scan_context_manager/scan_context_manager.hpp"

namespace  robot_localization 
{
class Matching {
  public:
    Matching();

    bool Update(const CloudData& cloud_data, Eigen::Matrix4d& cloud_pose);

    // bool SetScanContextPose(const CloudData& init_scan); //回环要不先保留把

    bool SetInitPose(const Eigen::Matrix4d& init_pose);
    bool SetInited(void);

    Eigen::Matrix4d GetInitPose(void);
    void GetGlobalMap(CloudData::CLOUD_PTR& global_map);
    CloudData::CLOUD_PTR& GetLocalMap();
    CloudData::CLOUD_PTR& GetCurrentScan();
    bool HasInited();
    bool HasNewGlobalMap();
    bool HasNewLocalMap();

  private:
    bool InitWithConfig();
    bool InitDataPath(const std::string pcd_map_path);
    // bool InitScanContextManager(const YAML::Node& config_node);  //回环要不先保留把
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
    bool InitBoxFilter(const YAML::Node& config_node);

    bool InitGlobalMap();
    bool ResetLocalMap(float x, float y, float z);

  private:
    std::string scan_context_path_ = "";
    std::string map_path_ = "";

    std::string loop_closure_method_ = "";

    // std::shared_ptr<ScanContextManager> scan_context_manager_ptr_;
    std::shared_ptr<RegistrationInterface> registration_ptr_; 

    std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;

    std::shared_ptr<BoxFilter> box_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;

    CloudData::CLOUD_PTR global_map_ptr_;
    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR current_scan_ptr_;

    Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity();

    bool has_inited_ = false;
    bool has_new_global_map_ = false;
    bool has_new_local_map_ = false;
};
}

#endif