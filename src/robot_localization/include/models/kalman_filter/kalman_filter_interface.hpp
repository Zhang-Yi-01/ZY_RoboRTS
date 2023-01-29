/*
 * @Description: 卡尔曼滤波接口
 * @Author: ZY 、Genshin才是最🐮的好吧
 * @Date: 2022.10.24
 */
#ifndef KALMAN_FILTER_INTERFACE_
#define KALMAN_FILTER_INTERFACE_

// c++
// #include <deque>
// yaml
#include <yaml-cpp/yaml.h>
// eigen
#include <Eigen/Core>
// #include <Eigen/Dense>
// 自定义imu数据类型
#include "../../sensor_data/imu_data.hpp"

namespace robot_localization
{
    class KalmanFilterInterface
    {
    public:
        /**
         * @class MeasurementType
         * @brief 观测类型 枚举
         */
        enum MeasurementType
        {
            POSE = 0,
            POSE_VEL,
            POSI,
            POSI_VEL,
            POSI_MAG,
            POSI_VEL_MAG,
            NUM_TYPES
        };

        /**
         * @class Measurement
         * @brief 卡尔曼滤波器测量数据
         */
        struct Measurement
        {
            double time;
            // 激光雷达里程计端位姿观测
            Eigen::Matrix4d T_nb;
            // 机器人基坐标速度观测，里程计给出
            Eigen::Vector3d v_b;
            // 机器人基坐标角度观测，由运动约束给出
            Eigen::Vector3d w_b;
            // 磁强计
            Eigen::Vector3d B_b;
        };

        /**
         * @class 协方差矩阵
         * @brief 卡尔曼过程协方差数据
         */
        struct Cov
        {
            struct
            {
                double x;
                double y;
                double z;
            } posi;
            struct
            {
                double x;
                double y;
                double z;
            } vel;
            struct
            {
                double w;
                double x;
                double y;
                double z;
            } ori;
            struct
            {
                double x;
                double y;
                double z;
            } gyro_bias;
            struct
            {
                double x;
                double y;
                double z;
            } accel_bias;
        };

        /**
         * @brief  初始化滤波器
         * @param  imu_data, input IMU measurements
         * @return
         */
        virtual void Init(const Eigen::Vector3d &vel, const ImuData &imu_data) = 0;

        /**
         * @brief  卡尔曼滤波器预测过程：更新状态变量&协方差估计
         * @param  imu_data, input IMU measurements
         * @return
         */
        virtual bool Update(const ImuData &imu_data) = 0;

        /**
         * @brief  卡尔曼滤波器预测过程：更新状态变量&协方差估计
         * @param  measurement_type, input measurement type
         * @param  measurement, input measurement
         * @return
         */
        virtual bool Correct(const ImuData &imu_data,
                             const MeasurementType &measurement_type,
                             const Measurement &measurement) = 0;

        /**
         * @brief  得到最近的滤波时间
         * @return filter time as double
         */
        double GetTime(void) const { return time_; }

        /**
         * @brief  获得里程计估计
         * @param  pose, output pose
         * @param  vel, output vel
         * @return void
         */
        virtual void GetOdometry(Eigen::Matrix4d &pose, Eigen::Vector3d &vel) = 0;

        /**
         * @brief  获得协方差矩阵估计
         * @param  cov, output covariance
         * @return void
         */
        virtual void GetCovariance(Cov &cov) = 0;

        /**
         * @brief  更新可观测性分析
         * @param  time, measurement time
         * @param  measurement_type, measurement type
         * @return void
         */
        virtual void UpdateObservabilityAnalysis(const double &time, const MeasurementType &measurement_type) = 0;

        /**
         * @brief  保存可观测性分析
         * @param  measurement_type, measurement type
         * @return void
         */
        virtual bool SaveObservabilityAnalysis(const MeasurementType &measurement_type) = 0;

    protected:
        KalmanFilterInterface() {}

        static void AnalyzeQ(const int DIM_STATE, const double &time,
                             const Eigen::MatrixXd &Q, const Eigen::VectorXd &Y,
                             std::vector<std::vector<double>> &data);

        static void WriteAsCsv(const int DIM_STATE,
                               const std::vector<std::vector<double>> &data,
                               const std::string filename);

        double time_;

        // imu数据缓存
        std::deque<ImuData> imu_data_buff_;

        // 地球常量
        Eigen::Vector3d g_;
        Eigen::Vector3d w_;
        Eigen::Vector3d b_;

        // 可观测性分析
        struct
        {
            std::vector<double> time_;
            std::vector<Eigen::MatrixXd> Q_;
            std::vector<Eigen::VectorXd> Y_;
        } observability;

        // 超参数
        // 地球常量
        struct
        {
            double GRAVITY_MAGNITUDE;
            double ROTATION_SPEED;
            double LATITUDE;
            double LONGITUDE;
            struct
            {
                double B_E;
                double B_N;
                double B_U;
            } MAG;
        } EARTH;

        // 先验状态协方差 & 过程噪声 & 测量噪声
        struct
        {
            struct
            {
                double POSI;
                double VEL;
                double ORI;
                double EPSILON;
                double DELTA;
            } PRIOR; // 预测
            struct
            {
                double GYRO;
                double ACCEL;
                double BIAS_ACCEL;
                double BIAS_GYRO;
            } PROCESS; // 过程噪声
            struct
            {
                struct
                {
                    double POSI;
                    double ORI;
                } POSE; // 观测
                double POSI;
                double VEL;
                double ORI;
                double MAG;
            } MEASUREMENT; //测量
        } COV;
        // 运动约束
        struct
        {
            bool ACTIVATED;
            double W_B_THRESH;
        } MOTION_CONSTRAINT;
    };

} // namespace robot_localization

#endif