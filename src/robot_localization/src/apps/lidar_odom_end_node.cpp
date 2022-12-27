/*
 * @Description: 前段里程计
 * @Author: ZY
 * @Date: 2022.10.24
 */

// ros
#include <ros/ros.h>
#include <ros/package.h>
// glog
#include <glog/logging.h>
// tools
#include "../../include/tools/color_terminal.hpp"
// 前段数据处理流程控制
#include "../../include/mapping/lidar_odom_end/lidar_odom_end_flow.hpp"


using namespace robot_localization;

int main(int argc, char *argv[])
{
    // ros
    ros::init(argc, argv, "lidar_odom_end_node");
    ros::NodeHandle nh;

    // 彩色终端
    ColorTerminal::ColorNodeInfo("lidar_odom_end_node节点启动");

    // glog配置
    google::InitGoogleLogging(argv[0]);
    std::string path = ros::package::getPath("robot_localization");
    FLAGS_log_dir = path + "/log";
    FLAGS_alsologtostderr = 1; // 记录Log到本地 & 在终端中显示

    // 前端任务管理器
    std::shared_ptr<LidarOdomEndFlow> lidar_odom_end_flow_ptr = std::make_shared<LidarOdomEndFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        lidar_odom_end_flow_ptr->Run();
        rate.sleep();
    }

    return 0;
}
