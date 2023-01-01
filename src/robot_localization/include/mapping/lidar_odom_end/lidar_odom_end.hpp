/*
 * @Description: 里程计端算法
 * @Author: ZY
 * @Date: 
 */
#ifndef LIDAR_ODOM_END_HPP
#define LIDAR_ODOM_END_HPP

// c++
#include <deque>
#include <unordered_map>
#include <string>
// eigen
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "glog/logging.h"
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

// 自定义点云数据类型
#include "../../sensor_data/cloud_data.hpp"
// 点云滤波虚继承接口
#include "../../models/cloud_filter/cloud_filter_interface.hpp"
// 点云匹配虚继承接口
#include "../../models/registration/registration_interface.hpp"
#include "../../sensor_data/imu_data.hpp"
#include "../../models/kalman_filter/kalman_filter_interface.hpp"
#include "../../models/kalman_filter/eskf.hpp"

namespace robot_localization
{
    
    class LidarOdomEnd
    {
    public:
        struct Frame
        {
            Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();
            CloudData cloud_data_;
        };

    public:
        LidarOdomEnd();

        bool Init(const Eigen::Matrix4d &init_pose,
                  const Eigen::Vector3d &init_vel,
                  const ImuData &init_imu_data);

        bool Predict(const ImuData &imu_data);

        bool Correct(const ImuData &imu_data,
                     const CloudData &cloud_data,
                     Eigen::Matrix4d &cloud_pose);

        bool HasInited() const { return has_inited_; }

        double GetTime(void) { return kalman_filter_ptr_->GetTime(); }
        CloudData::CLOUD_PTR &GetCurrentScan() { return current_scan_ptr_; }
        void GetOdometry(Eigen::Matrix4d &pose, Eigen::Vector3d &vel);

    private:
        bool ConfigFrame(const YAML::Node &config_node);
        bool ConfigRegistrationMethod(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &config_node);
        bool ConfigFilterMethod(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node);
        bool ConfigKalmanFilterMethod(std::shared_ptr<KalmanFilterInterface> &kalman_filter_ptr_, const YAML::Node &config_node);

        bool AddNewFrame(const Frame &new_key_frame);

    private:
        std::string data_path_ = "";

        std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
        std::shared_ptr<RegistrationInterface> registration_ptr_;
        std::shared_ptr<KalmanFilterInterface> kalman_filter_ptr_;

        std::deque<Frame> local_map_frames_;

        CloudData::CLOUD_PTR local_map_ptr_;
        CloudData::CLOUD_PTR current_scan_ptr_;

        Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d current_pose_ = Eigen::Matrix4d::Identity();
        Eigen::Vector3d current_vel_ = Eigen::Vector3d::Zero();
        Frame current_frame_;

        // 当前滤波器的观测数据
        KalmanFilterInterface::Measurement current_measurement_;

        bool has_inited_ = false;
        float key_frame_distance_ = 2.0;
        int local_frame_num_ = 20;
    };

} // namespace robot_localization

#endif