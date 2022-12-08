/*
 * @Description: 点云发布
 * @Author: ZY
 * @Date: 2020-02-05 02:27:30
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_

// ros
#include <sensor_msgs/PointCloud2.h>
// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// sensor_data
#include "../sensor_data/cloud_data.hpp"

namespace robot_localization
{
    class CloudPublisher
    {
    public:
        CloudPublisher(ros::NodeHandle &nh,
                       std::string topic_name,
                       std::string frame_id,
                       size_t buff_size);
        CloudPublisher() = default;

        void Publish(CloudData::CLOUD_PTR &cloud_ptr_input, double time);
        void Publish(CloudData::CLOUD_PTR &cloud_ptr_input);

        bool HasSubscribers();

    private:
        void PublishData(CloudData::CLOUD_PTR &cloud_ptr_input, ros::Time time);

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_;
    };

} // namespace robot_localization

#endif