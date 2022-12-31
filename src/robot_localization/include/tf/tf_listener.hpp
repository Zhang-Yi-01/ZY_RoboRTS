/*
 * @Description: tf监听模块
 * @Author: ZY
 * @Date: 2022-12-03 16:01:21
 */
#ifndef TF_LISTENER_HPP_
#define TF_LISTENER_HPP_

// c++
#include <string>
// ros
#include <ros/ros.h>
// tf
#include <tf/transform_listener.h>
// eigen
#include <Eigen/Dense>

namespace robot_localization
{
    class TFListener
    {
    public:
        TFListener(ros::NodeHandle &nh, std::string base_frame_id, std::string child_frame_id);
        TFListener() = default;

        bool LookUpData(Eigen::Matrix4d &transform_matrix);

    private:
        bool TransformMatrix(const tf::StampedTransform &transform, Eigen::Matrix4d &transform_matrix);

    private:
        ros::NodeHandle nh_;
        tf::TransformListener listener_;
        std::string base_frame_id_;
        std::string child_frame_id_;
    };

} // namespace robot_localization

#endif