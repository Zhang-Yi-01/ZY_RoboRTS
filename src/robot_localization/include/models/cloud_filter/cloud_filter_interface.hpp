/*
 * @Description: 点云滤波接口
 * @Author: ZY
 * @Date: 2022.10.24
 */
#ifndef CLOUD_FILTER_INTERFACE_HPP
#define CLOUD_FILTER_INTERFACE_HPP

// 自定义点云消息类型
#include "../../../include/sensor_data/cloud_data.hpp"
// yaml
#include <yaml-cpp/yaml.h>

namespace robot_localization
{
    class CloudFilterInterface
    {
    public:
        virtual ~CloudFilterInterface() = default;
        virtual bool Filter(const CloudData::CLOUD_PTR &input_cloud_ptr, CloudData::CLOUD_PTR &filtered_cloud_ptr) = 0;
    };

} // namespace robot_localization

#endif