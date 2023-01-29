/*
 * @Description: 点云畸变补偿
 */
#include "../../../include/models/scan_distortion_adjust/distortion_adjust.hpp"

#include "glog/logging.h"

namespace robot_localization {

void DistortionAdjust::SetMotionInfo(float scan_period, ImuData imu_data_, Eigen::Vector3d fused_vel_) 
{
    scan_period_ = scan_period;
    // velocity_ <<
    angular_rate_ << imu_data_.angular_velocity_.x,
                     imu_data_.angular_velocity_.y,
                     imu_data_.angular_velocity_.z;
    velocity_ = fused_vel_;
    
}
bool DistortionAdjust::AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr) 
{
    CloudData::CLOUD_PTR origin_cloud_ptr(new CloudData::CLOUD(*input_cloud_ptr));
    output_cloud_ptr.reset(new CloudData::CLOUD());

    float orientation_space = 2.0 * M_PI;
    float delete_space = 5.0 * M_PI / 180.0;
    float start_orientation = atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);

    Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3d rotate_matrix = t_V.matrix().cast<double>();
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block<3,3>(0,0) = rotate_matrix.inverse().cast<float>();
    pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);

    velocity_ = rotate_matrix * velocity_;
    angular_rate_ = rotate_matrix * angular_rate_;

    for (size_t point_index = 1; point_index < origin_cloud_ptr->points.size(); ++point_index) 
    {
        float orientation = atan2(origin_cloud_ptr->points[point_index].y, origin_cloud_ptr->points[point_index].x);
        if (orientation < 0.0)
            orientation += 2.0 * M_PI;
        
        if (orientation < delete_space || 2.0 * M_PI - orientation < delete_space)
            continue;

        float real_time = fabs(orientation) / orientation_space * scan_period_ - scan_period_ / 2.0;

        Eigen::Vector3f origin_point(origin_cloud_ptr->points[point_index].x,
                                     origin_cloud_ptr->points[point_index].y,
                                     origin_cloud_ptr->points[point_index].z);

        Eigen::Matrix3f current_matrix = UpdateMatrix(real_time);
        Eigen::Vector3f rotated_point = current_matrix * origin_point;
        Eigen::Vector3d adjusted_point = rotated_point.cast<double>() + velocity_ * real_time;
        CloudData::POINT point;
        point.x = adjusted_point(0);
        point.y = adjusted_point(1);
        point.z = adjusted_point(2);
        output_cloud_ptr->points.push_back(point);
    }

    pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());
    return true;
}

Eigen::Matrix3f DistortionAdjust::UpdateMatrix(float real_time) 
{
    Eigen::Vector3f angle = angular_rate_.cast<float>() * real_time;
    Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf t_V;
    t_V = t_Vz * t_Vy * t_Vx;
    return t_V.matrix();
}
} // namespace lidar_localization