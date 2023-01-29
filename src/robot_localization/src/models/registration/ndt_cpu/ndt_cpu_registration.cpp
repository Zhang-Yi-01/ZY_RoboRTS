/*
 * @Description: NDT CPU lidar odometry
 * @Author: ZY
 * @Date: 
 */

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "glog/logging.h"
#include "../../../../include/tools/tic_toc.hpp"

#include "../../../../include/models/registration/ndt_cpu/ndt_cpu_registration.hpp"

namespace robot_localization
{

    NDTCPURegistration::NDTCPURegistration(const YAML::Node &node)
    {
        float res = node["res"].as<float>();
        float step_size = node["step_size"].as<float>();
        float trans_eps = node["trans_eps"].as<float>();
        int max_iter = node["max_iter"].as<int>();

        SetRegistrationParam(res, step_size, trans_eps, max_iter);
    }

    bool NDTCPURegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter)
    {
        ndt_cpu_.setResolution(res);    //网格大小设置
        ndt_cpu_.setStepSize(step_size);    //牛顿法优化的最大步长
        ndt_cpu_.setTransformationEpsilon(trans_eps);   //连续变换之间允许的最大差值
        ndt_cpu_.setMaximumIterations(max_iter);    //迭代最大次数

        LOG(INFO) << "NDT params:" << std::endl
                  << "res: " << res << ", "
                  << "step_size: " << step_size << ", "
                  << "trans_eps: " << trans_eps << ", "
                  << "max_iter: " << max_iter
                  << std::endl
                  << std::endl;

        return true;
    }

    bool NDTCPURegistration::SetInputTarget(const CloudData::CLOUD_PTR &input_target)
    {
        ndt_cpu_.setInputTarget(input_target);

        return true;
    }

    bool NDTCPURegistration::ScanMatch(const CloudData::CLOUD_PTR &input_source,
                                       const Eigen::Matrix4d &predict_pose,
                                       CloudData::CLOUD_PTR &result_cloud_ptr,
                                       Eigen::Matrix4d &result_pose)
    {   
        // TicToc example1;
        ndt_cpu_.setInputSource(input_source);
        // printf("TicToc setInputSource耗时 %.6lf ms\n",example1.toc());
        // TicToc example2;
        ndt_cpu_.align(*result_cloud_ptr, predict_pose.cast<float>());  // 配准用的float，阿这
        // printf("TicToc align耗时 %.6lf ms\n",example2.toc());
        // TicToc example3;
        result_pose = ndt_cpu_.getFinalTransformation().cast<double>(); // 匹配后的点云
        // printf("TicToc getFinalTransformation耗时 %.6lf ms\n",example3.toc());

        return true;
    }

}