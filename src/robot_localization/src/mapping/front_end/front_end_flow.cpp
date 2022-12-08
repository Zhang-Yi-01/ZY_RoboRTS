/*
 * @Description: 前端任务管理器
 * @Author: ZY
 * @Date: 2022.10.24
 */

#include "../../../include/mapping/front_end/front_end_flow.hpp"
// tools
#include "../../../include/tools/color_terminal.hpp"

namespace robot_localization
{
    /**
     * @brief 前端流程控制初始化
     * @note 订阅点云信息 发布激光里程计
     * @todo
     **/
    FrontEndFlow::FrontEndFlow(ros::NodeHandle &nh)
    {
        // 读取YAML参数
        std::string config_file_path = ros::package::getPath("robot_localization") + "/config/front_end.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);
        // 配置消息话题
        std::string imu_raw_data_topic = config_node["topic"]["imu_raw_data_topic"].as<std::string>();
        std::string undistrotion_pointcloud_topic = config_node["topic"]["undistrotion_pointcloud_topic"].as<std::string>();
        std::string imu_link = config_node["topic"]["imu_link"].as<std::string>();
        std::string lidar_link = config_node["topic"]["lidar_link"].as<std::string>();

        // 订阅
        // 1.IMU原始数据
        imu_raw_sub_ptr_ = std::make_shared<ImuSubscriber>(nh, imu_raw_data_topic, 1000000);
        // 2.去畸变点云
        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, undistrotion_pointcloud_topic, 1000000);
        // 3.IMU 同步测量
        imu_synced_sub_ptr_ = std::make_shared<ImuSubscriber>(nh, imu_raw_data_topic, 1000000);
        // 4.lidar to imu tf
        lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, imu_link, lidar_link);

        // 发布
        // 1.全局点云地图
        global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "global_map", "map", 100);
        // 2.局部点云地图
        local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "local_map", "map", 100);
        // 3.当前帧雷达扫描
        current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "current_scan", "map", 100);
        // 4.estimated lidar pose in map frame
        laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "laser_localization", "map", "lidar", 100);
        // 5.fused psoe in map frame
        fused_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "fused_localization", "map", "lidar", 100);
        // 6.tf
        laser_tf_pub_ptr_ = std::make_shared<TFBroadcaster>("map", "base_link");

        // 前端算法
        front_end_ptr_ = std::make_shared<FrontEnd>();

        ColorTerminal::ColorFlowInfo("ESKF数据融合FLOW配置完成");
    }

    /**
     * @brief 前端流程运行
     * @note
     * @todo
     **/
    bool FrontEndFlow::Run()
    {
        if (!InitCalibration())
        {
            return false;
        }
        // PublishGlobalMap();
        // PublishLocalMap();

        ReadData();

        while (HasData())
        {
            // 初始化第一帧
            if (!HasInited())
            {
                if (ValidLidarData())
                {
                    InitLocalization();
                }
            }
            else
            {
                // 如果有雷达数据
                if (HasLidarData() && ValidLidarData())
                {
                    // 如果有IMU数据
                    if (HasIMUData())
                    {
                        // 当前帧雷达数据之前的惯性数据
                        while (HasIMUData() && ValidIMUData() && current_imu_raw_data_.time_stamp_ < current_cloud_data_.time_stamp_)
                        {
                            // 卡尔曼预测
                            Predict();
                        }

                        // 将取出的IMU数据放回队列
                        if (current_imu_raw_data_.time_stamp_ >= current_cloud_data_.time_stamp_)
                        {
                            imu_raw_data_buff_.push_back(current_imu_raw_data_);
                        }
                    }

                    // 卡尔曼更新
                    Correct();
                }

                // 如果只有IMU数据
                if (HasIMUData() && ValidIMUData())
                {
                    // 卡尔曼预测
                    Predict();
                }
            }
        }
        return true;
    }

    bool FrontEndFlow::ReadData()
    {
        // 将IMU原属数据存入缓存
        imu_raw_sub_ptr_->ParseData(imu_raw_data_buff_);

        while (HasInited() && HasIMUData() && imu_raw_data_buff_.front().time_stamp_ < front_end_ptr_->GetTime())
        {
            imu_raw_data_buff_.pop_front();
        }

        cloud_sub_ptr_->ParseData(cloud_data_buff_);

        imu_synced_sub_ptr_->ParseData(imu_synced_data_buff_);

        return true;
    }

    bool FrontEndFlow::HasInited(void)
    {
        return front_end_ptr_->HasInited();
    }

    bool FrontEndFlow::HasData()
    {
        if (!HasInited())
        {
            if (!HasLidarData())
            {
                return false;
            }
        }
        else
        {
            if (!HasIMUData() && !HasLidarData())
            {
                return false;
            }
        }

        return true;
    }

    bool FrontEndFlow::ValidIMUData()
    {
        current_imu_raw_data_ = imu_raw_data_buff_.front();

        imu_raw_data_buff_.pop_front();

        return true;
    }

    bool FrontEndFlow::ValidLidarData()
    {
        current_cloud_data_ = cloud_data_buff_.front();
        current_imu_synced_data_ = imu_synced_data_buff_.front();

        double diff_imu_time = current_cloud_data_.time_stamp_ - current_imu_synced_data_.time_stamp_;

        if (diff_imu_time < -0.05)
        {
            cloud_data_buff_.pop_front();
            return false;
        }

        if (diff_imu_time > 0.05)
        {
            imu_synced_data_buff_.pop_front();
            return false;
        }

        cloud_data_buff_.pop_front();
        imu_synced_data_buff_.pop_front();

        return true;
    }

    bool FrontEndFlow::InitCalibration()
    {
        // lookup imu pose in lidar frame:
        static bool calibration_received = false;

        if (!calibration_received)
        {
            if (lidar_to_imu_ptr_->LookUpData(lidar_to_imu_))
            {
                calibration_received = true;
            }
        }

        return calibration_received;
    }

    bool FrontEndFlow::InitLocalization(void)
    {
        Eigen::Vector3d init_vel = Eigen::Vector3d::Zero();
        Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();

        if (front_end_ptr_->Init(init_pose, init_vel, current_imu_synced_data_))
        {
            LOG(INFO) << " Localization Init Succeeded." << std::endl;
        }

        return true;
    }

    bool FrontEndFlow::Predict()
    {
        if (front_end_ptr_->Predict(current_imu_raw_data_))
        {
            PublishFusionOdom();
            return true;
        }

        return false;
    }

    bool FrontEndFlow::Correct()
    {
        bool is_fusion_succeeded = front_end_ptr_->Correct(current_imu_synced_data_,
                                                           current_cloud_data_,
                                                           laser_pose_);
        PublishLidarOdom();

        if (is_fusion_succeeded)
        {
            PublishFusionOdom();
            return true;
        }

        return false;
    }

    bool FrontEndFlow::PublishGlobalMap()
    {
        // if (front_end_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers())
        // {
        //     CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
        //     front_end_ptr_->GetGlobalMap(global_map_ptr);
        //     global_map_pub_ptr_->Publish(global_map_ptr);

        //     return true;
        // }

        return false;
    }

    bool FrontEndFlow::PublishLocalMap()
    {
        // if (front_end_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers())
        // {
        //     local_map_pub_ptr_->Publish(front_end_ptr_->GetLocalMap());

        //     return true;
        // }

        return false;
    }

    bool FrontEndFlow::PublishLidarOdom()
    {
        // 1. publish lidar odometry
        laser_odom_pub_ptr_->Publish(laser_pose_, current_cloud_data_.time_stamp_);
        // 2. publish current scan:
        current_scan_pub_ptr_->Publish(front_end_ptr_->GetCurrentScan());

        return true;
    }

    bool FrontEndFlow::PublishFusionOdom()
    {
        // get odometry from Kalman filter:
        front_end_ptr_->GetOdometry(fused_pose_, fused_vel_);
        // 1. publish tf:
        laser_tf_pub_ptr_->SendTransform(fused_pose_, current_imu_raw_data_.time_stamp_);
        // 2. publish fusion odometry:
        fused_odom_pub_ptr_->Publish(fused_pose_, fused_vel_, current_imu_raw_data_.time_stamp_);

        return true;
    }

} // namespace robot_localization
