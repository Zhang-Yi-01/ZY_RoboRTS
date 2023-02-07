/*
 * @Description: tf监听模块
 */

#include "../../include/tf/tf_listener.hpp"

// eigen
#include <Eigen/Geometry>

namespace robot_localization
{
    TFListener::TFListener(
                            std::shared_ptr<rclcpp::Node>& node_, 
                            std::string base_frame_id, 
                            std::string child_frame_id
                          ):ros2_node_(node_), 
                            base_frame_id_(base_frame_id), 
                            child_frame_id_(child_frame_id) 
    {
        // tf2_ros::Buffer 需要用 rclcpp::Clock::SharedPtr clock初始化，而不是ros2_node_->now()
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(ros2_node_->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    bool TFListener::LookUpData(Eigen::Matrix4d &transform_matrix)
    {
        try
        {   
            tf2::Transform tf2_transform;
            geometry_msgs::msg::TransformStamped transform;
            transform = tf_buffer_ ->lookupTransform(
                                                      base_frame_id_,
                                                      child_frame_id_,
                                                      tf2::TimePointZero
                                                    );//得到的是 child_frame_id_在 base_frame_id_坐标系下的位置

            // listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0), transform);
            TransformMatrix(transform, transform_matrix);
        }
        catch (tf2::TransformException &e)
        {   
            //Could not transform %s to %s: %s"//转换失败
            return false;
        }

        return true;
    }

    bool TFListener::TransformMatrix(
                                      const geometry_msgs::msg::TransformStamped &transform,
                                      Eigen::Matrix4d &transform_matrix
                                    )
    {   
        Eigen::Translation3d tl_btol(
                                        transform.transform.translation.x,
                                        transform.transform.translation.y,
                                        transform.transform.translation.z
                                    );
        // Eigen::Translation3d tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());

        
        double roll, pitch, yaw;
        tf2::Quaternion rotation_ ;
        rotation_.setX(transform.transform.rotation.x);
        rotation_.setY(transform.transform.rotation.y);
        rotation_.setZ(transform.transform.rotation.z);
        rotation_.setW(transform.transform.rotation.w);

        tf2::Matrix3x3(rotation_).getEulerYPR(yaw, pitch, roll);
        
        Eigen::AngleAxisd rot_x_btol(roll,  Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rot_y_btol(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rot_z_btol(yaw,   Eigen::Vector3d::UnitZ());

        // 此矩阵为 child_frame_id 到 base_frame_id 的转换矩阵
        transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

        return true;
    }

} // namespace robot_localization
