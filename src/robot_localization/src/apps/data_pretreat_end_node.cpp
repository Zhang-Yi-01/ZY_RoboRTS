/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等,也是对应感知的接口
 */
#include "rclcpp/rclcpp.hpp"
#include "glog/logging.h"

#include "../../include/global_path_defination/global_path.h"
#include "../../include/data_pretreat/data_pretreat_end_flow.hpp"

using namespace robot_localization;

int main(int argc, char *argv[]) 
{   
    setlocale(LC_ALL,"");
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr ros2_node = nullptr;
    ros2_node = rclcpp::Node::make_shared("data_pretreat_end_node");
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_PACKAGE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;


    // 接收
    // 原始点云信息、感知部分给到的动态障碍物信息

    // 发布
    // a. undistorted Velodyne measurement
    // b. lidar pose in map frame
    std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::make_shared<DataPretreatFlow>(ros2_node);

    while (rclcpp::ok()) 
    {

        data_pretreat_flow_ptr->Run();
        rclcpp::spin_some(ros2_node);

    }
    rclcpp::shutdown();

    return 0;
}