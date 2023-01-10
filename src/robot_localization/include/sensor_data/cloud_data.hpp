/*
 * @Description: 自定义点云数据
 */
#ifndef CLOUD_DATA_HPP
#define CLOUD_DATA_HPP

// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

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
        double time_stamp_ = 0.0;
        CLOUD_PTR cloud_ptr_;
    };

} // namespace robot_localization

#endif
