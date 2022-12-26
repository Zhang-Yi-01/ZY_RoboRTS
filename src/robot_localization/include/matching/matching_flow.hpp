/*
 * @Description: matching 模块任务管理， 放在类里使代码更清晰
 * @Author: lsc
 * @Date: 
 */
#ifndef LOCALIZATION_MATCHING_MATCHING_FLOW_HPP_
#define LOCALIZATION_MATCHING_MATCHING_FLOW_HPP_

#include <ros/ros.h>
#include <ros/package.h>
#include "../tools/color_terminal.hpp"
// subscriber
#include "../subscriber/cloud_subscriber.hpp"
#include "../subscriber/odometry_subscriber.hpp"
// publisher
#include "../publisher/cloud_publisher.hpp"
#include "../publisher/odometry_publisher.hpp"
#include "../tf/tf_broadcaster.hpp"
// matching
#include "matching.hpp"

namespace  robot_localization 
{
class MatchingFlow {
  public:
    MatchingFlow(ros::NodeHandle& nh);
    bool Run();

  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateMatching();
    bool PublishData();

  private:
    // subscriber 
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;

    // publisher
    std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    std::shared_ptr<TFBroadcaster> laser_tf_pub_ptr_;
    // matching
    std::shared_ptr<Matching> matching_ptr_;

    std::deque<CloudData> cloud_data_buff_;

    CloudData current_cloud_data_;

    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
};
}

#endif