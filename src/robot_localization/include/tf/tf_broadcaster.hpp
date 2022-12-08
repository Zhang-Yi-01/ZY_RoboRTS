/*
 * @Description: tf广播
 * @Author: ZY
 * @Date: 2022-12-03 15:23:26
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_TF_BROADCASTER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_TF_BROADCASTER_HPP_

// c++
#include <string>
// ros
#include <ros/ros.h>
// tf
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
// eigen
#include <Eigen/Dense>

namespace robot_localization
{
    class TFBroadcaster
    {
    public:
        TFBroadcaster(std::string frame_id, std::string child_frame_id);
        TFBroadcaster() = default;

        void SendTransform(Eigen::Matrix4d pose, double time);

    protected:
        tf::StampedTransform transform_;
        tf::TransformBroadcaster broadcaster_;
    };

} // namespace robot_localization

#endif