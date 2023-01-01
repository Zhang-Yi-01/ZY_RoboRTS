/*
 * @Description: 优化端工作流，接口
 * @Author: Genshin_Yi
 * @Date: 
 */
#ifndef OPTIMIZING_END_SLIDING_WINDOW_FLOW_HPP_
#define OPTIMIZING_END_SLIDING_WINDOW_FLOW_HPP_

#include <ros/ros.h>

//
// subscribers:
//
// a. lidar odometry, map matching pose reference position:
#include "../../subscriber/odometry_subscriber.hpp"
// b. IMU measurement, for pre-integration:
#include "../../subscriber/imu_subscriber.hpp"

#include "../../publisher/cloud_publisher.hpp"
#include "../../publisher/odometry_publisher.hpp"
#include "../../tf/tf_broadcaster.hpp"

// #include "../../publisher/key_frame_publisher.hpp"
// #include "../../publisher/key_frames_publisher.hpp"


#include "../../mapping/optimizing_end/sliding_window.hpp"


namespace robot_localization {

class SlidingWindowFlow {
public:
    SlidingWindowFlow(ros::NodeHandle& nh);

    bool Run();
    bool SaveOptimizedTrajectory();
    
  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateIMUPreIntegration(void);
    bool sliding_window_optimizing();
    bool PublishData();

  private:
    //
    // subscribers:
    //
    // a. lidar odometry:
    std::shared_ptr<OdometrySubscriber> laser_odom_sub_ptr_;
    std::deque<PoseData> laser_odom_data_buff_;
    // b. map matching odometry:
    std::shared_ptr<OdometrySubscriber> map_matching_odom_sub_ptr_;
    std::deque<PoseData> map_matching_odom_data_buff_;
    // c. IMU measurement, for pre-integration:
    std::shared_ptr<ImuSubscriber> imu_raw_sub_ptr_;
    std::deque<ImuData> imu_raw_data_buff_;
    std::shared_ptr<ImuSubscriber> imu_synced_sub_ptr_;
    std::deque<ImuData> imu_synced_data_buff_;
    // d. GNSS position:
    std::shared_ptr<OdometrySubscriber> gnss_pose_sub_ptr_;
    std::deque<PoseData> gnss_pose_data_buff_;

    //
    // publishers:
    //
    // std::shared_ptr<KeyFramePublisher> key_frame_pub_ptr_;
    // std::shared_ptr<KeyFramePublisher> key_gnss_pub_ptr_;
    std::shared_ptr<OdometryPublisher> optimized_odom_pub_ptr_;
    // std::shared_ptr<KeyFramesPublisher> optimized_trajectory_pub_ptr_;
    std::shared_ptr<TFBroadcaster> laser_tf_pub_ptr_;

    //
    // backend:
    //
    std::shared_ptr<SlidingWindow> sliding_window_ptr_;

    //
    // synced data:
    //
    PoseData current_laser_odom_data_;
    PoseData current_map_matching_odom_data_;
    ImuData current_imu_data_;
    PoseData current_gnss_pose_data_;

    bool if_sliding_window_tf_broadcast = false;
};

} // namespace robot_localization

#endif // MATCHING_BACK_END_SLIDING_WINDOW_FLOW_HPP_