/*
 * @Description: 基于图优化之滑动窗口模型的优化端
 * @Author: ZY、genshin_zy
 * @Date: 2022.10.24
 */

// ros
#include <ros/ros.h>
#include <ros/package.h>
// glog
#include <glog/logging.h>
// tools
#include "../../include/tools/color_terminal.hpp"
// 后端数据处理流程控制

using namespace robot_localization;

int main(int argc, char *argv[])
{
    // ros
    ros::init(argc, argv, "sliding_window_optimization_node");
    ros::NodeHandle nh;

    //彩色终端
    ColorTerminal::ColorNodeInfo("sliding_window_optimization_node start ->");

    // glog
    google::InitGoogleLogging(argv[0]);
    std::string path = ros::package::getPath("robot_localization");
    FLAGS_log_dir = path + "/log";
    FLAGS_alsologtostderr = 1; //记录Log到本地 & 在终端中显示

    //优化端任务管理器

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}
