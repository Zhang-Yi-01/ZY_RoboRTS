/*
 * @Description: 体素滤波器
 * @Author: ZY
 * @Date: 2022.10.24
 */

#include "../../../include/models/cloud_filter/voxel_filter.hpp"

namespace robot_localization
{
    /**
     * @brief 体素滤波器初始化
     * @note YAML设定参数
     * @todo
     **/
    VoxelFilter::VoxelFilter(const YAML::Node &node)
    {
        float leaf_size_x = node["leaf_size"][0].as<float>();
        float leaf_size_y = node["leaf_size"][1].as<float>();
        float leaf_size_z = node["leaf_size"][2].as<float>();

        SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
    }

    /**
     * @brief 体素滤波器初始化
     * @note 形参传入方式设定参数
     * @todo
     **/
    VoxelFilter::VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z)
    {
        SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
    }

    /**
     * @brief 设置滤波参数
     * @note
     * @todo
     **/
    bool VoxelFilter::SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z)
    {
        voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
        LOG(INFO) << "[VoxelFilter_param]" << std::endl
                  << leaf_size_x << "," << leaf_size_y << "," << leaf_size_z << std::endl;
        return true;
    }

    /**
     * @brief 点云滤波
     * @note
     * @todo
     **/
    bool VoxelFilter::Filter(const CloudData::CLOUD_PTR &input_cloud_ptr, CloudData::CLOUD_PTR &filtered_cloud_ptr)
    {
        voxel_filter_.setInputCloud(input_cloud_ptr);
        voxel_filter_.filter(*filtered_cloud_ptr);

        return true;
    }

} // namespace robot_localization
