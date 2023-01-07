/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Genshin_Yi
 * @Date: 
 */
#include "../../include/data_pretreat/data_pretreat_end_flow.hpp"
#include "../../include/global_defination/global_defination.h.in"
#include "../../include/tools/color_terminal.hpp"

#include "glog/logging.h"

namespace robot_localization {

DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic) 
{   
    // 读取YAML参数
        std::string user_config_path = WORK_PACKAGE_PATH + "/config/user_setting.yaml";
        YAML::Node user_config = YAML::LoadFile(user_config_path);
        // 配置消息话题
        std::string imu_raw_data_topic="";
        std::string raw_pointcloud_topic="";
        std::string imu_link="";
        std::string lidar_link="";
        std::string car_base_link="";
        std::string undistorted_pointcloud_topic="";
        if(user_config["if_simulink"].as<bool>())
        {

        imu_raw_data_topic = user_config["sim_imu_topic"].as<std::string>();
        raw_pointcloud_topic = user_config["sim_scan_pointcloud_topic"].as<std::string>();
        imu_link = user_config["sim_imu_link"].as<std::string>();
        lidar_link = user_config["sim_lidar_link"].as<std::string>();
        car_base_link = user_config["sim_car_base_link"].as<std::string>();
        }else
        {
        imu_raw_data_topic = user_config["imu_topic"].as<std::string>();
        raw_pointcloud_topic = user_config["scan_pointcloud_topic"].as<std::string>();
        imu_link = user_config["imu_link"].as<std::string>();
        lidar_link = user_config["lidar_link"].as<std::string>();
        car_base_link = user_config["car_base_link"].as<std::string>();

        }
        undistorted_pointcloud_topic = user_config["undistorted_pointcloud_topic"].as<std::string>();

    // subscribers:
    // a. velodyne measurement:
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, raw_pointcloud_topic, 100000);
    // b. IMU:
    imu_sub_ptr_ = std::make_shared<ImuSubscriber>(nh, imu_raw_data_topic, 1000000);
    
    fused_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/fused_odom", 100000);

    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, lidar_link, car_base_link);

    // publishers:
    // cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, undistorted_pointcloud_topic, "/velo_link", 100);
    //                   "/velo_link" 这个位置在里程计端用的是 map，这个有点问题
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, undistorted_pointcloud_topic, car_base_link, 100);
    imu_pub_ptr_ = std::make_shared<IMUPublisher>(nh, "/synced_imu", imu_link, 100);
    // pos_vel_pub_ptr_ = std::make_shared<PosVelPublisher>(nh, "/synced_pos_vel", "/map", "/imu_link", 100);
    // gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "/map", "/map", 100);

    // motion compensation for lidar measurement:
    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
}

bool DataPretreatFlow::Run() 
{
    if (!ReadData())
        return false;

    if (!InitCalibration()) 
        return false;


    while(HasData()) 
    {
        if (!ValidData())
            continue;

        TransformData();
        PublishData();
    }

    return true;
}

bool DataPretreatFlow::ReadData() 
{
    static std::deque<ImuData> unsynced_imu_;

    // a. lidar odometry:
    fused_odom_sub_ptr_->ParseData(fused_odom_data_buff_);
    // fetch lidar measurements from buffer:
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    imu_sub_ptr_->ParseData(unsynced_imu_);

    if (cloud_data_buff_.size() == 0)
        return false;

    // use timestamp of lidar measurement as reference:
    double cloud_time = cloud_data_buff_.front().time_stamp_;
    // sync IMU, velocity and GNSS with lidar measurement:
    // find the two closest measurement around lidar measurement time
    // then use linear interpolation to generate synced measurement:
    bool valid_imu = ImuData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);

    // only mark lidar as 'inited' when all the three sensors are synced:
    static bool sensor_inited = false;
    if (!sensor_inited) 
    {
        if (!valid_imu ) 
        {
            cloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }

    return true;
}

bool DataPretreatFlow::InitCalibration()//标定初始化 
{
    // lookup imu pose in lidar frame:
    static bool calibration_received = false;
    if (!calibration_received) 
    {
        if (lidar_to_imu_ptr_->LookUpData(lidar_to_imu_.cast<double>())) 
        {
            calibration_received = true;
        }
    }

    return calibration_received;
}


bool DataPretreatFlow::HasData() 
{
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;
        

    return true;
}

bool DataPretreatFlow::ValidData() 
{   
    current_fused_odom_data_ = fused_odom_data_buff_.front();
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();

    double diff_imu_time = current_cloud_data_.time_stamp_ - current_imu_data_.time_stamp_;
    //
    // this check assumes the frequency of lidar is 10Hz:
    //
    if (diff_imu_time < -0.05  ) 
    {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_imu_time > 0.05) 
    {
        imu_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();

    return true;
}

bool DataPretreatFlow::TransformData() 
{
    // c. motion compensation for lidar measurements:
    // current_velocity_data_.TransformCoordinate(lidar_to_imu_);
    // distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);

    // 把里程计端给的线速度，imu给的角速度，扔去做机械式雷达畸变矫正
    distortion_adjust_ptr_->SetMotionInfo(0.1, current_imu_data_,current_fused_odom_data_.vel.v);

    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr_, current_cloud_data_.cloud_ptr_);

    return true;
}

bool DataPretreatFlow::PublishData() 
{
    // take lidar measurement time as synced timestamp:
    const double &timestamp_synced = current_cloud_data_.time_stamp_;

    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr_, timestamp_synced);
    imu_pub_ptr_->Publish(current_imu_data_, timestamp_synced);

    
    return true;
}

}