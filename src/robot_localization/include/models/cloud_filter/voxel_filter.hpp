/*
 * @Description: 体素滤波器
 * @Author: ZY
 * @Date: 2022.10.24
 */

#ifndef VOXEL_FILTER_HPP
#define VOXEL_FILTER_HPP

#include "cloud_filter_interface.hpp"
// glog
#include <glog/logging.h>
// pcl
#include <pcl/filters/voxel_grid.h>

namespace robot_localization
{
    class VoxelFilter : public CloudFilterInterface
    {
    public:
        VoxelFilter(const YAML::Node &node);
        VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);
        bool Filter(const CloudData::CLOUD_PTR &input_cloud_ptr, CloudData::CLOUD_PTR &filter_cloud_ptr) override;

    private:
        bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    private:
        pcl::VoxelGrid<CloudData::POINT> voxel_filter_;
    };

} // namespace robot_localization

#endif
