/*
 * @Description: 自定义点云数据
 */
#ifndef CLOUD_DATA_HPP
#define CLOUD_DATA_HPP

// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "rclcpp/time.hpp"

namespace robot_localization
{
    class CloudData
    {
    public:
        using POINT = pcl::PointXYZ;
        using CLOUD = pcl::PointCloud<pcl::PointXYZ>;
        using CLOUD_PTR = pcl::PointCloud<pcl::PointXYZ>::Ptr;

    public:
        CloudData();

    public:
        builtin_interfaces::msg::Time ros2_time = rclcpp::Time(0,0);
        double time_stamp_ = 0.0;
        CLOUD_PTR cloud_ptr_;
    };

} // namespace robot_localization

#endif
