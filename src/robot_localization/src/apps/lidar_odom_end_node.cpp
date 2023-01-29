/*
 * @Description: 激光雷达里程计端
 */

// ros2
#include "rclcpp/rclcpp.hpp"
// glog
// #include <glog/logging.h>
// tools
#include "../../include/tools/color_terminal.hpp"
// 前段数据处理流程控制
#include "../../include/mapping/lidar_odom_end/lidar_odom_end_flow.hpp"

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
    std::shared_ptr<LidarOdomEndFlow> lidar_odom_end_flow_ptr = std::make_shared<LidarOdomEndFlow>(ros2_node);
    
    rclcpp::WallRate loop_rate(110); //别用这个，无论是rate还是wallrate，跟spinsome是有内涵的
    
    while (rclcpp::ok())
    {           
        lidar_odom_end_flow_ptr->run_test_odom_match();
        // lidar_odom_end_flow_ptr->Run();

        rclcpp::spin_some(ros2_node);
    }
    rclcpp::shutdown();

    return 0;
}
