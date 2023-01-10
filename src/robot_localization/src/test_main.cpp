#include "../include/tf/tf_broadcaster.hpp"

// ros2
#include "rclcpp/rclcpp.hpp"
// glog
#include <glog/logging.h>
// tools
#include "../include/tools/color_terminal.hpp"
// 前段数据处理流程控制

using namespace robot_localization;

int main(int argc, char *argv[])
{
    // ros2
    setlocale(LC_ALL,"");
    rclcpp::init(argc,argv);

    rclcpp::Node::SharedPtr ros2_node = nullptr;
    ros2_node = rclcpp::Node::make_shared("lidar_odom_end_node");
    // 彩色终端
    ColorTerminal::ColorNodeInfo("lidar_odom_end_node节点启动");

    // glog配置
    google::InitGoogleLogging(argv[0]);
    std::string path = WORK_PACKAGE_PATH;
    FLAGS_log_dir = path + "/log";
    FLAGS_alsologtostderr = 1; // 记录Log到本地 & 在终端中显示

    // 激光雷达里程计端任务管理器    
    rclcpp::Rate loop_rate(110);
    
    while (rclcpp::ok())
    {   
        
        rclcpp::spin_some(ros2_node);
        // lidar_odom_end_flow_ptr->Run();
        loop_rate.sleep();
    }

    return 0;
}
