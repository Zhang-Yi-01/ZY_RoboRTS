/*
 * @Description: 订阅odometry数据
 */
#include "../../include/subscriber/odometry_subscriber.hpp"
#include "glog/logging.h"

using std::placeholders::_1;

namespace robot_localization{

OdometrySubscriber::OdometrySubscriber ( std::shared_ptr<rclcpp::Node>& node_,
                                         std::string topic_name,
                                         size_t buff_size
                                       )
{   
    subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
                        topic_name,
                        buff_size,
                        std::bind(&OdometrySubscriber::msg_callback, this, _1)
                                                                     );
}

void OdometrySubscriber::msg_callback(const nav_msgs::msg::Odometry& odom_msg)
{
    buff_mutex_.lock();
    PoseData pose_data;

    // ROS2的时间类型 builtin_interfaces::msg::Time 需要把秒和纳秒相加才能表示当前时间
    // 如果获取的是msg时间戳，恢复成完整表达如下
    // double now_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    // auto now_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    pose_data.time = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec* 1e-9;

    //set the position
    pose_data.pose(0,3) = odom_msg.pose.pose.position.x;
    pose_data.pose(1,3) = odom_msg.pose.pose.position.y;
    pose_data.pose(2,3) = odom_msg.pose.pose.position.z;

    Eigen::Quaternionf q;
    q.x() = odom_msg.pose.pose.orientation.x;
    q.y() = odom_msg.pose.pose.orientation.y;
    q.z() = odom_msg.pose.pose.orientation.z;
    q.w() = odom_msg.pose.pose.orientation.w;
    pose_data.pose.block<3,3>(0,0) = q.matrix();

    pose_data.vel.v = Eigen::Vector3f(odom_msg.twist.twist.linear.x,
                                    odom_msg.twist.twist.linear.y,
                                    odom_msg.twist.twist.linear.z
                                    );
    
    pose_data.vel.w = Eigen::Vector3f(odom_msg.twist.twist.angular.x,
                                    odom_msg.twist.twist.angular.y,
                                    odom_msg.twist.twist.angular.z
                                    );

    new_pose_data_.push_back(pose_data);
    buff_mutex_.unlock();
}

void OdometrySubscriber::ParseData(std::deque<PoseData>& pose_data_buff) 
{
    buff_mutex_.lock();
    if (new_pose_data_.size() > 0) 
    {
        pose_data_buff.insert(pose_data_buff.end(), new_pose_data_.begin(), new_pose_data_.end());
        new_pose_data_.clear();
    }
    buff_mutex_.unlock();
}
}