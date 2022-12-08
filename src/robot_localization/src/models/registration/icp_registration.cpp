/*
 * @Description: ICP 匹配模块
 * @Author: ZY
 * @Date: 2020-10-24 21:46:45
 */
#include "../../../include/models/registration/icp_registration.hpp"
// glog
#include "glog/logging.h"

namespace robot_localization
{

    ICPRegistration::ICPRegistration(const YAML::Node &node) : icp_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>())
    {

        float max_corr_dist = node["max_corr_dist"].as<float>();
        float trans_eps = node["trans_eps"].as<float>();
        float euc_fitness_eps = node["euc_fitness_eps"].as<float>();
        int max_iter = node["max_iter"].as<int>();

        SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
    }

    ICPRegistration::ICPRegistration(float max_corr_dist,
                                     float trans_eps,
                                     float euc_fitness_eps,
                                     int max_iter)
        : icp_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>())
    {

        SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
    }

    bool ICPRegistration::SetRegistrationParam(float max_corr_dist,
                                               float trans_eps,
                                               float euc_fitness_eps,
                                               int max_iter)
    {
        icp_ptr_->setMaxCorrespondenceDistance(max_corr_dist);
        icp_ptr_->setTransformationEpsilon(trans_eps);
        icp_ptr_->setEuclideanFitnessEpsilon(euc_fitness_eps);
        icp_ptr_->setMaximumIterations(max_iter);

        LOG(INFO) << "ICP params:" << std::endl
                  << "max_corr_dist: " << max_corr_dist << ", "
                  << "trans_eps: " << trans_eps << ", "
                  << "euc_fitness_eps: " << euc_fitness_eps << ", "
                  << "max_iter: " << max_iter
                  << std::endl
                  << std::endl;

        return true;
    }

    bool ICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR &input_target)
    {
        icp_ptr_->setInputTarget(input_target);

        return true;
    }

    bool ICPRegistration::ScanMatch(const CloudData::CLOUD_PTR &input_source,
                                    const Eigen::Matrix4d &predict_pose,
                                    CloudData::CLOUD_PTR &result_cloud_ptr,
                                    Eigen::Matrix4d &result_pose)
    {
        icp_ptr_->setInputSource(input_source);           //  输入待配准点云
        icp_ptr_->align(*result_cloud_ptr, predict_pose.cast<float>()); // 配准
        result_pose = icp_ptr_->getFinalTransformation().cast<double>(); // 获取变换矩阵

        return true;
    }

}