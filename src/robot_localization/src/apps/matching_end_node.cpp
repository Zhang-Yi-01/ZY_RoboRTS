/*
 * @Description: scan to map 地图匹配定位端
 */

// glog
#include <glog/logging.h>
// 彩色文字终端
// #include "../../include/tools/color_terminal.hpp"
#include "../../include/mapping/matching_end/matching_end_flow.hpp"

using namespace robot_localization;

int main(int argc, char *argv[]) 
{   
     // ros2
    setlocale(LC_ALL,"");
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr ros2_node = nullptr;
    ros2_node = rclcpp::Node::make_shared("pcd_matching_end_node");

    // glog配置
    google::InitGoogleLogging(argv[0]);
    std::string path = WORK_PACKAGE_PATH;
    FLAGS_log_dir = path + "/log";
    FLAGS_alsologtostderr = 1; // 记录Log到本地 & 在终端中显示

    // 匹配端任务管理器
    std::shared_ptr<MatchingFlow> matching_flow_ptr = std::make_shared<MatchingFlow>(ros2_node);

    rclcpp::Rate loop_rate(100);
    while (rclcpp::ok()) 
    {
        rclcpp::spin_some(ros2_node);

        matching_flow_ptr->Run();

        // loop_rate.sleep();
    }
    rclcpp::shutdown();

    return 0;
}