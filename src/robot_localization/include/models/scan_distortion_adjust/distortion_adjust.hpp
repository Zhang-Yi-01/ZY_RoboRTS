/*
 * @Description: 点云畸变补偿
 * @Author: Genshin_Yi
 * @Date: 
 */

#ifndef MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_
#define MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "glog/logging.h"


#include "lidar_localization/sensor_data/velocity_data.hpp"
#include "../../sensor_data/cloud_data.hpp"


namespace robot_localization {
  
class DistortionAdjust 
{
  public:
    void SetMotionInfo(float scan_period, VelocityData velocity_data);//那里程计端的速度来做畸变矫正
    bool AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr);

  private:
    inline Eigen::Matrix3f UpdateMatrix(float real_time);

  private:
    float scan_period_;
    Eigen::Vector3f velocity_;
    Eigen::Vector3f angular_rate_;
};
} // namespace lidar_slam
#endif