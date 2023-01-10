/*
 * @Description: 点云发布
 */

// glog
#include <glog/logging.h>
// 头文件
#include "../../include/publisher/cloud_publisher.hpp"
#include "pcl_msgs/msg/point_indices.hpp"
namespace robot_localization
{
    CloudPublisher::CloudPublisher(
                                    std::shared_ptr<rclcpp::Node> &node_,
                                    std::string topic_name,
                                    std::string frame_id,
                                    size_t buff_size
                                  )
        : ros2_node_(node_), frame_id_(frame_id)
    {   
        publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name,buff_size);

    }

    void CloudPublisher::Publish(CloudData::CLOUD_PTR &cloud_ptr_input, double time)
    {   
        // double 转换 builtin_interfaces::msg::Time找不到方法！！！！！！
        builtin_interfaces::msg::Time ros2_time = ros2_node_->now();
        // std::chrono::seconds(time);
        time = time*1;
        // builtin_interfaces::msg::Time ros2_time =  rclcpp::Time(time);//测试，这个只能把double硬转为int类型的nanosec并令sec=0
        // ros::Time ros_time(time);

        PublishData(cloud_ptr_input, ros2_time);
    }

    void CloudPublisher::Publish(CloudData::CLOUD_PTR &cloud_ptr_input)
    {
        builtin_interfaces::msg::Time time = ros2_node_->now();
        PublishData(cloud_ptr_input, time);
    }

    void CloudPublisher::PublishData(CloudData::CLOUD_PTR &cloud_ptr_input, builtin_interfaces::msg::Time time)
    {
        sensor_msgs::msg::PointCloud2 cloud_ptr_output;
        pcl::toROSMsg(*cloud_ptr_input, cloud_ptr_output);

        cloud_ptr_output.header.stamp = time;
        cloud_ptr_output.header.frame_id = frame_id_;

        publisher_->publish(cloud_ptr_output);
    }

    bool CloudPublisher::HasSubscribers()
    {
        // ros2 好像没有这个函数了,留待后面吧
        // return publisher_.getNumSubscribers() != 0;
        return true;
    }

} // namespace robot_localization
