/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等,也是对应感知的接口
 * @Author: Genshin_Yi
 * @Date:  
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include "../../include/global_defination/global_defination.h.in"
#include "../../include/data_pretreat/data_pretreat_end_flow.hpp"


using namespace robot_localization;

int main(int argc, char *argv[]) 
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_PACKAGE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "data_pretreat_end_node");
    ros::NodeHandle nh;

    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");

    // 接收
    // 原始点云信息、感知部分给到的动态障碍物信息

    // 发布
    // a. undistorted Velodyne measurement
    // b. lidar pose in map frame
    std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::make_shared<DataPretreatFlow>(nh, cloud_topic);

    // pre-process lidar point cloud at 100Hz:
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        data_pretreat_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}