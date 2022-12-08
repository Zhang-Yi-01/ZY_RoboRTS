/*
 * @Description: tf监听模块
 * @Author: ZY
 * @Date: 2022-12-03 16:01:21
 */

#include "../../include/tf/tf_listener.hpp"

// eigen
#include <Eigen/Geometry>

namespace robot_localization
{
    TFListener::TFListener(ros::NodeHandle &nh, std::string base_frame_id, std::string child_frame_id)
        : nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id) {}

    bool TFListener::LookUpData(Eigen::Matrix4d &transform_matrix)
    {
        try
        {
            tf::StampedTransform transform;
            listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0), transform);
            TransformMatrix(transform, transform_matrix);
        }
        catch (tf::TransformException &e)
        {
            return false;
        }

        return true;
    }

    bool TFListener::TransformMatrix(const tf::StampedTransform &transform, Eigen::Matrix4d &transform_matrix)
    {
        Eigen::Translation3d tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());

        double roll, pitch, yaw;
        tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
        Eigen::AngleAxisd rot_x_btol(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rot_y_btol(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rot_z_btol(yaw, Eigen::Vector3d::UnitZ());

        // 此矩阵为 child_frame_id 到 base_frame_id 的转换矩阵
        transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

        return true;
    }

} // namespace robot_localization
