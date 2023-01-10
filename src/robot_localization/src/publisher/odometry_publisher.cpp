/*
 * @Description: 里程计发布
 */

#include "../../include/publisher/odometry_publisher.hpp"

namespace robot_localization
{
    /**
     * @brief 里程计发布初始化
     * @note
     * @todo
     **/
    OdometryPublisher::OdometryPublisher(
                                         std::shared_ptr<rclcpp::Node>& node_,
                                         std::string topic_name,
                                         std::string base_frame_id,
                                         std::string child_frame_id,
                                         size_t buff_size
                                        ): ros2_node_(node_)
    {

        publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(topic_name,buff_size);

        odometry_.header.frame_id = base_frame_id;
        odometry_.child_frame_id = child_frame_id;
    }

    /**
     * @brief 里程计发布信息
     * @note
     * @todo
     **/
    // void OdometryPublisher::Publish(const Eigen::Matrix4d &transform_matrix, double time)
    // {
    //     ros::Time ros_time((float)time);
    //     PublishData(transform_matrix, ros_time);
    // }

    void OdometryPublisher::Publish(const Eigen::Matrix4d &transform_matrix)
    {
        PublishData(transform_matrix, ros2_node_->now());
    }

    // void OdometryPublisher::Publish(
    //                                 const Eigen::Matrix4d &transform_matrix, 
    //                                 const Eigen::Vector3d &vel, 
    //                                 double time
    //                                 )
    // {
    //     ros::Time ros_time(time);
    //     PublishData(transform_matrix, vel, ros_time);
    // }
    /**
     * @brief 里程数据发布
     * @note
     * @todo
     **/
    void OdometryPublisher::PublishData(const Eigen::Matrix4d &transform_matrix, builtin_interfaces::msg::Time time)
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

        publisher_->publish(odometry_);
    }

    void OdometryPublisher::PublishData(
                                        const Eigen::Matrix4d &transform_matrix,
                                        const Eigen::Vector3d &vel,
                                        builtin_interfaces::msg::Time time
                                       )
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

        publisher_->publish(odometry_);
    }

    /**
     * @brief 是否被订阅
     * @note
     * @todo
     **/
    bool OdometryPublisher::HasSubscriber()
    {
        // return publisher_.getNumSubscribers() != 0;
        // ros2 好像没有这个函数了,留待后面吧
        return true;
    }

} // namespace robot_localization
