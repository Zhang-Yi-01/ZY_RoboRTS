/*
 * @Description: ICP 匹配模块
 */
#ifndef ROBOT_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_
#define ROBOT_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_

#include "registration_interface.hpp"
// pcl
#include <pcl/registration/icp.h>

namespace robot_localization
{
  class ICPRegistration : public RegistrationInterface
  {
  public:
    ICPRegistration(const YAML::Node &node);
    ICPRegistration(
        float max_corr_dist,
        float trans_eps,
        float euc_fitness_eps,
        int max_iter);

    bool SetInputTarget(const CloudData::CLOUD_PTR &input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR &input_source,
                   const Eigen::Matrix4d &predict_pose,
                   CloudData::CLOUD_PTR &result_cloud_ptr,
                   Eigen::Matrix4d &result_pose) override;

  private:
    bool SetRegistrationParam(
        float max_corr_dist,
        float trans_eps,
        float euc_fitness_eps,
        int max_iter);

  private:
    pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>::Ptr icp_ptr_;
  };
}

#endif