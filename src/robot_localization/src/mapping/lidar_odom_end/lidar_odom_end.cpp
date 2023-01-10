/*
 * @Description: 里程计端算法
 * @Author: ZY 、 Genshin_Yi
 * @Date: 2022.10.24
 */

#include "../../../include/mapping/lidar_odom_end/lidar_odom_end.hpp"

// ros
#include <ros/ros.h>
// #include <ros/package.h>
// pcl
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
// 匹配
#include "../../../include/models/registration/ndt_registration.hpp"
#include "../../../include/models/registration/icp_registration.hpp"
#include "../../../include/models/registration/icp_svd_registration.hpp"
#include "../../../include/models/registration/ndt_cpu/ndt_cpu_registration.hpp"
// 滤波
#include "../../../include/models/cloud_filter/voxel_filter.hpp"
// 融合
// #include "../../../include/models/kalman_filter/eskf.hpp"
// tools
#include "../../../include/tools/color_terminal.hpp"
// yaml
// #include <yaml-cpp/yaml.h>
// glog
// #include <glog/logging.h>

namespace robot_localization
{

    /**
     * @brief 里程计端流程控制初始化
     * @note 订阅点云信息 发布激光里程计
     * @todo
     **/
    LidarOdomEnd::LidarOdomEnd() : local_map_ptr_(new CloudData::CLOUD()), current_scan_ptr_(new CloudData::CLOUD())
    {
        // 读取YAML参数
        std::string config_file_path = WORK_PACKAGE_PATH + "/config/params/odom_end.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);
        // 参数配置
        ConfigFrame(config_node);
        ConfigRegistrationMethod(registration_ptr_, config_node);
        ConfigFilterMethod("local_map", local_map_filter_ptr_, config_node);
        ConfigFilterMethod("frame", frame_filter_ptr_, config_node);
        ConfigKalmanFilterMethod(kalman_filter_ptr_, config_node);
    }

    /**
     * @brief  关键帧配置
     * @note
     * @todo
     **/
    bool LidarOdomEnd::ConfigFrame(const YAML::Node &config_node)
    {
        key_frame_distance_ = config_node["key_frame_distance"].as<float>();
        local_frame_num_ = config_node["local_frame_num"].as<int>();
        return true;
    }

    bool LidarOdomEnd::ConfigRegistrationMethod(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &config_node)
    {
        std::string config_file_path = WORK_PACKAGE_PATH + "/config/user_setting.yaml";
        YAML::Node user_setting_node = YAML::LoadFile(config_file_path);
        std::string registration_method = user_setting_node["registration_method"].as<std::string>();

        if (registration_method == "NDT")
        {
            registration_ptr = std::make_shared<NdtRegistration>(config_node[registration_method]);
            LOG(INFO) << "[registration_method]" << std::endl
                      << registration_method << std::endl;
        }
        else if (registration_method == "ICP")
        {
            registration_ptr = std::make_shared<ICPRegistration>(config_node[registration_method]);
            LOG(INFO) << "[registration_method]" << std::endl
                      << registration_method << std::endl;
        }
        else if (registration_method == "ICP_SVD")
        {
            registration_ptr = std::make_shared<ICPSVDRegistration>(config_node[registration_method]);
            LOG(INFO) << "[registration_method]" << std::endl
                      << registration_method << std::endl;
        }
        else if (registration_method == "NDT_CPU")
        {
            registration_ptr = std::make_shared<NDTCPURegistration>(config_node[registration_method]);
            LOG(INFO) << "[registration_method]" << std::endl
                      << registration_method << std::endl;
        }
        else
        {
            LOG(ERROR) << "[无对应匹配方法]" << std::endl;
            ROS_BREAK();
        }
        return true;
    }

    bool LidarOdomEnd::ConfigFilterMethod(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node)
    {
        std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();

        if (filter_mothod == "voxel_filter")
        {
            filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
            LOG(INFO) << "[filter_mothod]" << std::endl
                      << filter_mothod << std::endl;
        }
        else
        {
            LOG(ERROR) << "[无对应滤波方法]" << std::endl;
            ROS_BREAK();
        }
        return true;
    }

    bool LidarOdomEnd::ConfigKalmanFilterMethod(std::shared_ptr<KalmanFilterInterface> &kalman_filter_ptr_, const YAML::Node &config_node)
    {
        std::string kalman_filter_method = config_node["kalman_filter_method"].as<std::string>();

        if (kalman_filter_method == "eskf")
        {
            kalman_filter_ptr_ = std::make_shared<ESKF>(config_node[kalman_filter_method]);
            LOG(INFO) << "[kalman_filter_method]" << std::endl
                      << kalman_filter_method << std::endl;
        }
        else
        {
            LOG(ERROR) << "[无对应卡尔曼融合方法]" << std::endl;
            ROS_BREAK();
        }

        return true;
    }

    bool LidarOdomEnd::Init(const Eigen::Matrix4d &init_pose,
                        const Eigen::Vector3d &init_vel,
                        const ImuData &init_imu_data)
    {
        init_pose_ = init_pose;
        current_vel_ = init_vel;

        kalman_filter_ptr_->Init(current_vel_, init_imu_data);

        has_inited_ = true;

        return true;
    }

    bool LidarOdomEnd::Predict(const ImuData &imu_data)
    {
        if (kalman_filter_ptr_->Update(imu_data))
        {
            kalman_filter_ptr_->GetOdometry(current_pose_, current_vel_);

            return true;
        }

        return false;
    }

    bool LidarOdomEnd::Correct(const ImuData &imu_data,
                           const CloudData &cloud_data,
                           Eigen::Matrix4d &cloud_pose)
    {
        // 定义用于运动递推的参数
        static Eigen::Matrix4d step_pose = Eigen::Matrix4d::Identity();
        static Eigen::Matrix4d last_pose = init_pose_;
        static Eigen::Matrix4d predict_pose = init_pose_;
        static Eigen::Matrix4d last_key_frame_pose = init_pose_;

        // 拷贝时间戳
        current_frame_.cloud_data_.time_stamp_ = cloud_data.time_stamp_;

        // 去除无效点
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr_, *current_frame_.cloud_data_.cloud_ptr_, indices);

        // 降采样
        CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
        frame_filter_ptr_->Filter(current_frame_.cloud_data_.cloud_ptr_, filtered_cloud_ptr);

        // 第一帧处理
        if (local_map_frames_.size() == 0)
        {
            current_frame_.pose_ = init_pose_;
            AddNewFrame(current_frame_);
            cloud_pose = current_frame_.pose_;
            return true;
        }

        // 匹配
        CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
        registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, current_frame_.pose_);
        cloud_pose = current_frame_.pose_;

        // 更新预测位姿
        // TODO:解释数学原理
        step_pose = last_pose.inverse() * current_frame_.pose_;
        predict_pose = current_frame_.pose_ * step_pose;
        last_pose = current_frame_.pose_;

        // 是否更新关键帧
        if ((fabs(last_key_frame_pose(0, 3) - current_frame_.pose_(0, 3)) +
             fabs(last_key_frame_pose(1, 3) - current_frame_.pose_(1, 3)) +
             fabs(last_key_frame_pose(2, 3) - current_frame_.pose_(2, 3))) >
            key_frame_distance_)
        {
            AddNewFrame(current_frame_);
            last_key_frame_pose = current_frame_.pose_;
        }

        // set lidar measurement:
        current_measurement_.time = cloud_data.time_stamp_;
        current_measurement_.T_nb = init_pose_.inverse() * cloud_pose;
        current_measurement_.w_b = Eigen::Vector3d(imu_data.angular_velocity_.x,
                                                   imu_data.angular_velocity_.y,
                                                   imu_data.angular_velocity_.z);

        // 卡尔曼更新
        if (kalman_filter_ptr_->Correct(imu_data, KalmanFilterInterface::MeasurementType::POSE, current_measurement_))
        {
            kalman_filter_ptr_->GetOdometry(current_pose_, current_vel_);
            return true;
        }

        return false;
    }

    void LidarOdomEnd::GetOdometry(Eigen::Matrix4d &pose, Eigen::Vector3d &vel)
    {
        pose = init_pose_ * current_pose_;
        vel = init_pose_.block<3, 3>(0, 0) * current_vel_;
    }

    /**
     * @brief  更新新关键帧
     * @note
     * @todo
     **/
    bool LidarOdomEnd::AddNewFrame(const Frame &new_key_frame)
    {
        /*深拷贝到本地*/
        Frame key_frame = new_key_frame;
        key_frame.cloud_data_.cloud_ptr_.reset(new CloudData::CLOUD(*new_key_frame.cloud_data_.cloud_ptr_));
        CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());

        /*更新局部地图*/
        local_map_frames_.push_back(key_frame);
        while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_))
        {
            local_map_frames_.pop_front();
        }
        local_map_ptr_.reset(new CloudData::CLOUD());
        for (size_t i = 0; i < local_map_frames_.size(); ++i)
        {
            pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data_.cloud_ptr_,
                                     *transformed_cloud_ptr,
                                     local_map_frames_.at(i).pose_);

            *local_map_ptr_ += *transformed_cloud_ptr;
        }

        /*更新目标点云*/
        if (local_map_frames_.size() < 10)
        {
            registration_ptr_->SetInputTarget(local_map_ptr_);
        }
        else
        {
            CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
            local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
            registration_ptr_->SetInputTarget(filtered_local_map_ptr);
        }
        return true;
    }
} // namespace robot_localization
