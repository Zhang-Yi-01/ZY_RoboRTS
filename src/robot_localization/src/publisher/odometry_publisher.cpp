/*
 * @Description: 里程计发布
 * @Author: ZY
 * @Date: 2022.10.24
 */

#include "../../include/publisher/odometry_publisher.hpp"

namespace robot_localization
{
    /**
     * @brief 里程计发布初始化
     * @note
     * @todo
     **/
    OdometryPublisher::OdometryPublisher(ros::NodeHandle &nh,
                                         std::string topic_name,
                                         std::string base_frame_id,
                                         std::string child_frame_id,
                                         size_t buff_size) : nh_(nh)
    {

        publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
        odometry_.header.frame_id = base_frame_id;
        odometry_.child_frame_id = child_frame_id;
    }

    /**
     * @brief 里程计发布信息
     * @note
     * @todo
     **/
    void OdometryPublisher::Publish(const Eigen::Matrix4d &transform_matrix, double time)
    {
        ros::Time ros_time((float)time);
        PublishData(transform_matrix, ros_time);
    }

    void OdometryPublisher::Publish(const Eigen::Matrix4d &transform_matrix)
    {
        PublishData(transform_matrix, ros::Time::now());
    }

    void OdometryPublisher::Publish(const Eigen::Matrix4d &transform_matrix, const Eigen::Vector3d &vel, double time)
    {
        ros::Time ros_time(time);
        PublishData(transform_matrix, vel, ros_time);
    }
    /**
     * @brief 里程数据发布
     * @note
     * @todo
     **/
    void OdometryPublisher::PublishData(const Eigen::Matrix4d &transform_matrix, ros::Time time)
    {
        odometry_.header.stamp = time;

        odometry_.pose.pose.position.x = transform_matrix(0, 3);
        odometry_.pose.pose.position.y = transform_matrix(1, 3);
        odometry_.pose.pose.position.z = transform_matrix(2, 3);

        Eigen::Quaterniond q;
        q = transform_matrix.block<3, 3>(0, 0);
        odometry_.pose.pose.orientation.x = q.x();
        odometry_.pose.pose.orientation.y = q.y();
        odometry_.pose.pose.orientation.z = q.z();
        odometry_.pose.pose.orientation.w = q.w();

        publisher_.publish(odometry_);
    }

    void OdometryPublisher::PublishData(const Eigen::Matrix4d &transform_matrix, const Eigen::Vector3d &vel, ros::Time time)
    {
        odometry_.header.stamp = time;

        odometry_.pose.pose.position.x = transform_matrix(0, 3);
        odometry_.pose.pose.position.y = transform_matrix(1, 3);
        odometry_.pose.pose.position.z = transform_matrix(2, 3);

        Eigen::Quaterniond q;
        q = transform_matrix.block<3, 3>(0, 0);
        odometry_.pose.pose.orientation.x = q.x();
        odometry_.pose.pose.orientation.y = q.y();
        odometry_.pose.pose.orientation.z = q.z();
        odometry_.pose.pose.orientation.w = q.w();
        
        odometry_.twist.twist.linear.x = vel.x();
        odometry_.twist.twist.linear.y = vel.y();
        odometry_.twist.twist.linear.z = vel.z();

        publisher_.publish(odometry_);
    }

    /**
     * @brief 是否被订阅
     * @note
     * @todo
     **/
    bool OdometryPublisher::HasSubscriber()
    {
        return publisher_.getNumSubscribers() != 0;
    }

} // namespace robot_localization
