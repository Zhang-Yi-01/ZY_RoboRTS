/*
 * @Description: 里程计发布
 * @Author: ZY
 * @Date: 2022.10.24
 */
#ifndef ODOMETRY_PUBLISHER_HPP
#define ODOMETRY_PUBLISHER_HPP

// ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
// c++
#include <string>
// eigen
#include <Eigen/Dense>

namespace robot_localization
{
    class OdometryPublisher
    {
    public:
        OdometryPublisher(ros::NodeHandle &nh,
                          std::string topic_name,
                          std::string base_frame_id,
                          std::string child_frame_id,
                          size_t buff_size);
        OdometryPublisher() = default;

        void Publish(const Eigen::Matrix4d &transform_matrix, double time);
        void Publish(const Eigen::Matrix4d &transform_matrix);
        void Publish(const Eigen::Matrix4d &transform_matrix, const Eigen::Vector3d &vel, double time);

        bool HasSubscriber();

    private:
        void PublishData(const Eigen::Matrix4d &transform_matrix, ros::Time time);
        void PublishData(const Eigen::Matrix4d &transform_matrix, const Eigen::Vector3d &vel, ros::Time time);

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        nav_msgs::Odometry odometry_;
    };

} // namespace robot_localization

#endif