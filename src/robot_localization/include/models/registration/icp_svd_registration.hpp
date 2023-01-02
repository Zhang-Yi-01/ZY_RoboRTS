/*
 * @Description: ICP SVD lidar odometry
 * @Author: ZY
 * @Date:  
 */
#ifndef ROBOT_LOCALIZATION_MODELS_REGISTRATION_ICP_SVD_REGISTRATION_HPP_
#define ROBOT_LOCALIZATION_MODELS_REGISTRATION_ICP_SVD_REGISTRATION_HPP_

#include "registration_interface.hpp"
// pcl
#include <pcl/kdtree/kdtree_flann.h>


namespace robot_localization
{

  class ICPSVDRegistration : public RegistrationInterface
  {
  public:
    ICPSVDRegistration(const YAML::Node &node);
    ICPSVDRegistration(float max_corr_dist,
                       float trans_eps,
                       float euc_fitness_eps,
                       int max_iter);

    bool SetInputTarget(const CloudData::CLOUD_PTR &input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR &input_source,
                   const Eigen::Matrix4d &predict_pose,
                   CloudData::CLOUD_PTR &result_cloud_ptr,
                   Eigen::Matrix4d &result_pose) override;

  private:
    bool SetRegistrationParam(float max_corr_dist,
                              float trans_eps,
                              float euc_fitness_eps,
                              int max_iter);

  private:
    size_t GetCorrespondence(const CloudData::CLOUD_PTR &input_source,
                             std::vector<Eigen::Vector3d> &xs,
                             std::vector<Eigen::Vector3d> &ys);

    void GetTransform(const std::vector<Eigen::Vector3d> &xs,
                      const std::vector<Eigen::Vector3d> &ys,
                      Eigen::Matrix4d &transformation_);

    bool IsSignificant(const Eigen::Matrix4d &transformation,
                       const float trans_eps);

    float max_corr_dist_;
    float trans_eps_;
    float euc_fitness_eps_;
    int max_iter_;

    CloudData::CLOUD_PTR input_target_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr input_target_kdtree_;
    CloudData::CLOUD_PTR input_source_;

    Eigen::Matrix4d transformation_;
  };

}

#endif