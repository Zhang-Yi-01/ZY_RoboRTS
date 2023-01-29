/*
 * @Description: SICP registration
 */
#ifndef MODELS_REGISTRATION_SICP_REGISTRATION_HPP_
#define MODELS_REGISTRATION_SICP_REGISTRATION_HPP_
#include <pcl/common/transforms.h>
#include "../registration_interface.hpp"
#include "../sicp/ICP.h"
namespace robot_localization
{

  class SICPRegistration : public RegistrationInterface
  {
  public:
    SICPRegistration(const YAML::Node &node);

    bool SetInputTarget(const CloudData::CLOUD_PTR &input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR &input_source,
                   const Eigen::Matrix4d &predict_pose,
                   CloudData::CLOUD_PTR &result_cloud_ptr,
                   Eigen::Matrix4d &result_pose) override;

  private:
    CloudData::CLOUD_PTR input_target_;
    CloudData::CLOUD_PTR input_source_;

    Eigen::Matrix4d transformation_;
    SICP::Parameters params_;
  };

} // namespace robot_localization

#endif