/*
 * @Description: 卡尔曼滤波接口MatrixXd
 */

// c++
#include <string>
#include <vector>
// SVD
#include <Eigen/SVD>
// glog
#include <glog/logging.h>
// global_defination
#include "../../../include/global_path_defination/global_path.h"
// tools
#include "../../../include/tools/CSVWriter.hpp"
// 头文件
#include "../../../include/models/kalman_filter/kalman_filter_interface.hpp"

namespace robot_localization
{
    void KalmanFilterInterface::AnalyzeQ(const int DIM_STATE, const double &time,
                                         const Eigen::MatrixXd &Q, const Eigen::VectorXd &Y,
                                         std::vector<std::vector<double>> &data)
    {
        // SVD分析，矩形矩阵
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Q, Eigen::ComputeThinU | Eigen::ComputeThinV);

        // add record
        std::vector<double> record;

        // record timestamp
        record.push_back(time);

        // record singular value
        for (int i = 0; i < DIM_STATE; ++i)
        {
            record.push_back(svd.singularValues()(i, 0));
        }

        // record degree of obsvervability
        Eigen::MatrixXd X = (svd.matrixV() * svd.singularValues().asDiagonal().inverse()) *
                            (svd.matrixU().transpose() * Y).asDiagonal();

        Eigen::VectorXd degree_of_observability = Eigen::VectorXd::Zero(DIM_STATE);

        for (int i = 0; i < DIM_STATE; ++i)
        {
            // 求X的最大幅值响应
            Eigen::MatrixXd::Index sv_index;
            X.col(i).cwiseAbs().maxCoeff(&sv_index);

            // 与相应的奇异值相关联
            degree_of_observability(sv_index) = svd.singularValues()(i);
        }

        // nomalize
        degree_of_observability = 1.0 / svd.singularValues().maxCoeff() * degree_of_observability;

        for (int i = 0; i < DIM_STATE; ++i)
        {
            record.push_back(degree_of_observability(i));
        }

        // add to data:
        data.push_back(record);
    }

    void KalmanFilterInterface::WriteAsCsv(const int DIM_STATE,
                                           const std::vector<std::vector<double>> &data,
                                           const std::string filename)
    {
        // init:
        CSVWriter csv(",");
        csv.enableAutoNewRow(1 + 2 * DIM_STATE);

        // write header:
        csv << "T";
        for (int i = 0; i < DIM_STATE; ++i)
        {
            csv << ("sv" + std::to_string(i + 1));
        }
        for (int i = 0; i < DIM_STATE; ++i)
        {
            csv << ("doo" + std::to_string(i + 1));
        }

        // write contents:
        for (const auto &record : data)
        {
            // cast timestamp to int:
            csv << static_cast<int>(record.at(0));

            for (size_t i = 1; i < record.size(); ++i)
            {
                csv << std::fabs(record.at(i));
            }
        }

        // save to persistent storage:
        csv.writeToFile(filename);
    }

} // namespace robot_localization
