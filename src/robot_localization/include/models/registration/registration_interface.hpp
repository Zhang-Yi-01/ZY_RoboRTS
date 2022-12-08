/*
 * @Description: 点云匹配接口
 * @Author: ZY
 * @Date: 2022.10.24
 */
#ifndef REGISTRATION_INTERFACE_HPP
#define REGISTRATION_INTERFACE_HPP

// 自定义点云消息类型
#include "../../../include/sensor_data/cloud_data.hpp"
// yaml
#include <yaml-cpp/yaml.h>
// eigen
#include <Eigen/Dense>

namespace robot_localization
{
    class RegistrationInterface
    {
    public:
        virtual ~RegistrationInterface() = default;
        virtual bool SetInputTarget(const CloudData::CLOUD_PTR &input_cloud) = 0;
        virtual bool ScanMatch(const CloudData::CLOUD_PTR &input_cloud_ptr,
                               const Eigen::Matrix4d &predict_pose,
                               CloudData::CLOUD_PTR &result_cloud_ptr,
                               Eigen::Matrix4d &result_pose) = 0;
    };

} // namespace robot_localization

#endif