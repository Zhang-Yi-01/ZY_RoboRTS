/*
 * @Description: 里程计端任务管理器
 * @Author: ZY 、 Genshin_Yi
 * @Date: 2022.10.24
 */

#include "../../../include/mapping/lidar_odom_end/lidar_odom_end_flow.hpp"
// tools
#include "../../../include/tools/color_terminal.hpp"

namespace robot_localization
{
    /**
     * @brief 里程计端流程控制初始化
     * @note 订阅点云信息 发布激光里程计
     * @todo
     **/
    LidarOdomEndFlow::LidarOdomEndFlow(std::shared_ptr<rclcpp::Node>& node_)
    {
        // 读取YAML参数
        std::string config_file_path = WORK_PACKAGE_PATH + "/config/user_setting.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);
        // 配置消息话题
        std::string imu_raw_data_topic;
        std::string undistrotion_pointcloud_topic;
        std::string imu_link;
        std::string lidar_link;
        std::string car_base_link;

        if(config_node["if_simulink"].as<bool>())
        {

        imu_raw_data_topic = config_node["sim_imu_topic"].as<std::string>();
        undistrotion_pointcloud_topic = config_node["sim_scan_pointcloud_topic"].as<std::string>();
        imu_link = config_node["sim_imu_link"].as<std::string>();
        lidar_link = config_node["sim_lidar_link"].as<std::string>();
        car_base_link = config_node["sim_car_base_link"].as<std::string>();
        }else
        {
        imu_raw_data_topic = config_node["imu_topic"].as<std::string>();
        undistrotion_pointcloud_topic = config_node["scan_pointcloud_topic"].as<std::string>();
        imu_link = config_node["imu_link"].as<std::string>();
        lidar_link = config_node["lidar_link"].as<std::string>();
        car_base_link = config_node["car_base_link"].as<std::string>();
        
        
        }
        
        coordinate_transformation = Eigen::Vector3d(
                                                    config_node["sim_init_position"][0].as<double>(),
                                                    config_node["sim_init_position"][1].as<double>(),
                                                    config_node["sim_init_position"][2].as<double>()
                                                   );
        if_odom_end_tf_broadcast = config_node["odom_end_send_tf"].as<bool>();

        // 订阅
        // 1.IMU原始数据
        imu_raw_sub_ptr_ = std::make_shared<ImuSubscriber>(node_, imu_raw_data_topic, 100000);
        // 2.去畸变点云
        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(node_, undistrotion_pointcloud_topic, 100000);
        // 3.IMU 同步测量
        imu_synced_sub_ptr_ = std::make_shared<ImuSubscriber>(node_, "/synced_imu", 100000);
        // 4.lidar to imu tf
        lidar_to_imu_ptr_ = std::make_shared<TFListener>(node_, imu_link, lidar_link);

        // 发布
        // 1.全局点云地图
        global_map_pub_ptr_ = std::make_shared<CloudPublisher>(node_, "global_map", "map", 100);
        // 2.局部点云地图
        local_map_pub_ptr_ = std::make_shared<CloudPublisher>(node_, "local_map", "map", 100);
        // 3.当前帧雷达扫描
        current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(node_, "current_scan", "map", 100);
        // 4.estimated lidar pose in map frame
        laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(node_, "laser_localization", "odom", "lidar", 100);
        // 5.fused psoe in map frame
        fused_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(node_, "fused_odom", "odom", "lidar", 100);
        // 6.tf
        laser_tf_pub_ptr_ = std::make_shared<TFBroadcaster>(node_, "odom", car_base_link);

        // 里程计端算法
        lidar_odom_end_ptr_ = std::make_shared<LidarOdomEnd>();

        ColorTerminal::ColorFlowInfo("ESKF数据融合FLOW配置完成");
    }

    /**
     * @brief 里程计端流程运行
     * @note
     * @todo
     **/
    bool LidarOdomEndFlow::Run()
    {
        if (!InitCalibration())
        {
            return false;
        }

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
                            yellow_info("predict 1 done");
                        }

                        // 如果imu原始数据大于lidar基准数据时间,将取出的IMU数据插回队列尾部
                        if (current_imu_raw_data_.time_stamp_ >= current_cloud_data_.time_stamp_)
                        {
                            imu_raw_data_buff_.push_back(current_imu_raw_data_);
                        }
                    }

                    // 卡尔曼更新
                    Correct();
                    green_info("correct done");
                }

                // 如果只有IMU数据
                if (HasIMUData() && ValidIMUData())
                {
                    // 卡尔曼预测
                    Predict();green_info("predict 2 done");
                }
            }
        }
        return true;
    }

    bool LidarOdomEndFlow::run_test_odom_match()
    {   
        cloud_sub_ptr_->ParseData(cloud_data_buff_);

        while(!cloud_data_buff_.empty())
        {
            current_cloud_data_ = cloud_data_buff_.front();
            bool is_fusion_succeeded = lidar_odom_end_ptr_->Correct(current_imu_synced_data_,
                                                           current_cloud_data_,
                                                           laser_pose_);
        }

    }
    bool LidarOdomEndFlow::ReadData()
    {
        // 将IMU原属数据存入缓存
        imu_raw_sub_ptr_->ParseData(imu_raw_data_buff_);
        
        while (HasInited() && HasIMUData() && imu_raw_data_buff_.front().time_stamp_ < lidar_odom_end_ptr_->GetTime())
        {
            imu_raw_data_buff_.pop_front();
        }

        cloud_sub_ptr_->ParseData(cloud_data_buff_);

        imu_synced_sub_ptr_->ParseData(imu_synced_data_buff_);

        return true;
    }

    bool LidarOdomEndFlow::HasInited(void)
    {
        return lidar_odom_end_ptr_->HasInited();
    }

    bool LidarOdomEndFlow::HasData()
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

    bool LidarOdomEndFlow::ValidIMUData()
    {
        current_imu_raw_data_ = imu_raw_data_buff_.front();
        // RCLCPP_INFO(this->get_logger(), "imu time : %.6lf\n",imu_data.time_stamp_);
        imu_raw_data_buff_.pop_front();

        return true;
    }

    bool LidarOdomEndFlow::ValidLidarData()
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

    // 标定初始化
    bool LidarOdomEndFlow::InitCalibration()
    {
        // lookup imu pose in lidar frame:
        static bool calibration_received = false;

        if (!calibration_received)
        {
            // lidar_to_imu_ 默认是 0,0,0即雷达和imu之间不需要标定
            if (lidar_to_imu_ptr_->LookUpData(lidar_to_imu_))
            {
                calibration_received = true;
            }
        }

        return calibration_received;
    }

    bool LidarOdomEndFlow::InitLocalization(void)
    {
        Eigen::Vector3d init_vel = Eigen::Vector3d::Zero();
        Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();
        // lidar_odom_end_ptr_ ->coordinate_transformation(init_pose ,coordinate_transformation);
        if (lidar_odom_end_ptr_->Init(init_pose, init_vel, current_imu_synced_data_))
        {
            LOG(INFO) << " Localization Init Succeeded." << std::endl;
        }

        return true;
    }

    bool LidarOdomEndFlow::Predict()
    {
        if (lidar_odom_end_ptr_->Predict(current_imu_raw_data_))
        {
            PublishFusionOdom();
            return true;
        }

        return false;
    }

    bool LidarOdomEndFlow::Correct()
    {
        bool is_fusion_succeeded = lidar_odom_end_ptr_->Correct(current_imu_synced_data_,
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

    bool LidarOdomEndFlow::PublishGlobalMap()
    {
        // if (lidar_odom_end_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers())
        // {
        //     CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
        //     lidar_odom_end_ptr_->GetGlobalMap(global_map_ptr);
        //     global_map_pub_ptr_->Publish(global_map_ptr);

        //     return true;
        // }

        return false;
    }

    bool LidarOdomEndFlow::PublishLocalMap()
    {
        // if (lidar_odom_end_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers())
        // {
        //     local_map_pub_ptr_->Publish(lidar_odom_end_ptr_->GetLocalMap());

        //     return true;
        // }

        return false;
    }

    bool LidarOdomEndFlow::PublishLidarOdom()
    {   
        // 1. publish lidar odometry
        laser_odom_pub_ptr_->Publish(laser_pose_, current_cloud_data_.ros2_time);
        // 2. publish current scan:
        current_scan_pub_ptr_->Publish(lidar_odom_end_ptr_->GetCurrentScan());

        return true;
    }

    bool LidarOdomEndFlow::PublishFusionOdom()
    {
        // get odometry from Kalman filter:
        lidar_odom_end_ptr_->GetOdometry(fused_pose_, fused_vel_);
        lidar_odom_end_ptr_ ->coordinate_transformation(fused_pose_ ,coordinate_transformation);
        // 1. publish tf:
        if(if_odom_end_tf_broadcast)
            laser_tf_pub_ptr_->SendTransform(fused_pose_, current_imu_raw_data_.ros2_time);
        // 2. publish fusion odometry:
        fused_odom_pub_ptr_->Publish(fused_pose_, fused_vel_, current_imu_raw_data_.ros2_time);

        return true;
    }

} // namespace robot_localization
