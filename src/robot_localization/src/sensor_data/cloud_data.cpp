/*
 * @Description: 自定义点云数据
 */
#include "../../include/sensor_data/cloud_data.hpp"
namespace robot_localization
{
    /**
     * @brief 激光雷达数据类型封装构造
     * @note 为cloud_ptr_分配内存空间
     * @todo
     **/
    CloudData::CloudData() : cloud_ptr_(new CLOUD())
    {
    }

} // namespace robot_localization
