/*
 * @Description: tf广播
 * @Author: ZY
 * @Date: 2022-12-03 15:23:26
 */
#include "../../include/tf/tf_broadcaster.hpp"

namespace robot_localization
{
    TFBroadcaster::TFBroadcaster(std::string frame_id, std::string child_frame_id)
    {
        transform_.frame_id_ = frame_id;
        transform_.child_frame_id_ = child_frame_id;
    }

    void TFBroadcaster::SendTransform(Eigen::Matrix4d pose, double time)
    {
        Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
        ros::Time ros_time(time);
        transform_.stamp_ = ros_time;
        transform_.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
        transform_.setOrigin(tf::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
        broadcaster_.sendTransform(transform_);
    }

} // namespace robot_localization
