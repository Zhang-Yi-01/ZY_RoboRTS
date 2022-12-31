/*
 * @Description: 基于图优化之滑动窗口模型的优化端
 * @Author: ZY 、 Genshin_Yi
 * @Date: 2022.10.24
 */

// ros
#include <ros/ros.h>
#include <ros/package.h>
// glog
#include <glog/logging.h>
// tools
#include "../../include/tools/color_terminal.hpp"
#include "../../include/mapping/optimizing_end/sliding_window_flow.hpp"

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

    //优化端工作流任务管理器
    std::shared_ptr<SlidingWindowFlow> sliding_window_flow_ptr = std::make_shared<SlidingWindowFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        sliding_window_flow_ptr->Run();
        rate.sleep();
    }

    return 0;
}
