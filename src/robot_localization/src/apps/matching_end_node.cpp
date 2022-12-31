/*
 * @Description: scan to map 地图匹配定位端
 * @Author: Genshin_Yi
 * @Date: 
 */
// ros
#include <ros/ros.h>
#include <ros/package.h>
// glog
#include <glog/logging.h>
// 彩色文字终端
#include "../../include/tools/color_terminal.hpp"
#include "../../include/mapping/matching_end/matching_end_flow.hpp"

using namespace robot_localization;

int main(int argc, char *argv[]) 
{
    // glog配置
    google::InitGoogleLogging(argv[0]);
    std::string path = ros::package::getPath("robot_localization");
    FLAGS_log_dir = path + "/log";
    FLAGS_alsologtostderr = 1; // 记录Log到本地 & 在终端中显示

    ros::init(argc, argv, "matching_node");
    ros::NodeHandle nh;

    // 匹配端任务管理器
    std::shared_ptr<MatchingFlow> matching_flow_ptr = std::make_shared<MatchingFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) 
    {
        ros::spinOnce();

        matching_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}