/*
 * @Description: 误差状态卡尔曼滤波：算法实现
 * @Author: ZY 、 Genshin_Yi
 * @Date: 2022.10.24
 */

// c++
#include <cstdlib>
#include <limits>
// #include <cmath>
#include <fstream>
#include <iostream>
#include <ostream>
// sophus
#include <sophus/so3.hpp>
// glog
#include "glog/logging.h"
// tools
#include "../../../include/tools/color_terminal.hpp"
// global_defination
#include "../../../include/global_path_defination/global_path.h"
// 头文件
#include "../../../include/models/kalman_filter/eskf.hpp"

namespace robot_localization
{
    ESKF::ESKF(const YAML::Node &node)
    {
        // 读取YAML文件参数
        // 地球约束
        EARTH.GRAVITY_MAGNITUDE = node["earth"]["gravity_magnitude"].as<double>();
        EARTH.LATITUDE = node["earth"]["latitude"].as<double>(); // altitude  纬度
        EARTH.LATITUDE *= M_PI / 180.0;
        // 先验方差
        COV.PRIOR.POSI = node["covariance"]["prior"]["pos"].as<double>();
        COV.PRIOR.VEL = node["covariance"]["prior"]["vel"].as<double>();
        COV.PRIOR.ORI = node["covariance"]["prior"]["ori"].as<double>();
        COV.PRIOR.EPSILON = node["covariance"]["prior"]["epsilon"].as<double>(); // bias_accel 随机游走
        COV.PRIOR.DELTA = node["covariance"]["prior"]["delta"].as<double>();     // bias_gyro  随机游走
        // 过程噪声（器件噪声）
        COV.PROCESS.ACCEL = node["covariance"]["process"]["accel"].as<double>();
        COV.PROCESS.GYRO = node["covariance"]["process"]["gyro"].as<double>();
        COV.PROCESS.BIAS_ACCEL = node["covariance"]["process"]["bias_accel"].as<double>();
        COV.PROCESS.BIAS_GYRO = node["covariance"]["process"]["bias_gyro"].as<double>();
        // 观测噪声
        COV.MEASUREMENT.POSE.POSI = node["covariance"]["measurement"]["pose"]["pos"].as<double>(); // 平移 pos
        COV.MEASUREMENT.POSE.ORI = node["covariance"]["measurement"]["pose"]["ori"].as<double>();  // 旋转 ori
        // glog
        LOG(INFO) << std::endl
                  << "Error-State Kalman Filter params:" << std::endl
                  << "\tgravity magnitude: " << EARTH.GRAVITY_MAGNITUDE << std::endl
                  << "\tlatitude: " << EARTH.LATITUDE << std::endl
                  << std::endl
                  << "\tprior cov. pos.: " << COV.PRIOR.POSI << std::endl
                  << "\tprior cov. vel.: " << COV.PRIOR.VEL << std::endl
                  << "\tprior cov. ori: " << COV.PRIOR.ORI << std::endl
                  << "\tprior cov. epsilon.: " << COV.PRIOR.EPSILON << std::endl
                  << "\tprior cov. delta.: " << COV.PRIOR.DELTA << std::endl
                  << std::endl
                  << "\tprocess noise gyro.: " << COV.PROCESS.GYRO << std::endl
                  << "\tprocess noise accel.: " << COV.PROCESS.ACCEL << std::endl
                  << std::endl
                  << "\tmeasurement noise pose.: " << std::endl
                  << "\t\tpos: " << COV.MEASUREMENT.POSE.POSI
                  << ", ori.: " << COV.MEASUREMENT.POSE.ORI << std::endl
                  << std::endl
                  << std::endl;

        // 初始化滤波器
        // 地球约束
        g_ = Eigen::Vector3d(0.0, 0.0, EARTH.GRAVITY_MAGNITUDE);

        // 先验
        ResetState();      // 状态量清零
        ResetCovariance(); // 初始化方差

        // 过程噪声
        Q_.block<3, 3>(kIndexNoiseAccel, kIndexNoiseAccel) = COV.PROCESS.ACCEL * Eigen::Matrix3d::Identity();
        Q_.block<3, 3>(kIndexNoiseGyro, kIndexNoiseGyro) = COV.PROCESS.GYRO * Eigen::Matrix3d::Identity();
        Q_.block<3, 3>(kIndexNoiseBiasAccel, kIndexNoiseBiasAccel) = COV.PROCESS.BIAS_ACCEL * Eigen::Matrix3d::Identity();
        Q_.block<3, 3>(kIndexNoiseBiasGyro, kIndexNoiseBiasGyro) = COV.PROCESS.BIAS_GYRO * Eigen::Matrix3d::Identity();

        // 观测噪声
        RPose_.block<3, 3>(0, 0) = COV.MEASUREMENT.POSE.POSI * Eigen::Matrix3d::Identity();
        RPose_.block<3, 3>(3, 3) = COV.MEASUREMENT.POSE.ORI * Eigen::Matrix3d::Identity();

        // 状态方程  状态转移矩阵   F:15*15    B:15*12
        F_.block<3, 3>(kIndexErrorPos, kIndexErrorVel) = Eigen::Matrix3d::Identity();
        F_.block<3, 3>(kIndexErrorOri, kIndexErrorGyro) = -Eigen::Matrix3d::Identity();
        B_.block<3, 3>(kIndexErrorOri, kIndexNoiseGyro) = Eigen::Matrix3d::Identity();
        B_.block<3, 3>(kIndexErrorAccel, kIndexNoiseBiasAccel) = Eigen::Matrix3d::Identity();
        B_.block<3, 3>(kIndexErrorGyro, kIndexNoiseBiasGyro) = Eigen::Matrix3d::Identity();

        // 观测方程  状态转移矩阵   G:6*15  C:6*6
        GPose_.block<3, 3>(0, kIndexErrorPos) = Eigen::Matrix3d::Identity();
        GPose_.block<3, 3>(3, kIndexErrorOri) = Eigen::Matrix3d::Identity();
        CPose_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        CPose_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

        // init soms：
        QPose_.block<kDimMeasurementPose, kDimState>(0, 0) = GPose_;
    }

    /**
     * @brief  init filter
     * @param  pose, init pose
     * @param  vel, init vel
     * @param  imu_data, init IMU measurements
     * @return true if success false otherwise
     */
    void ESKF::Init(const Eigen::Vector3d &vel, const ImuData &imu_data)
    {
        // 初始化里程计
        Eigen::Matrix3d C_nb = imu_data.OrientationToMatrix().cast<double>(); //  body2nav
        // a. init C_nb using IMU estimation:
        pose_.block<3, 3>(0, 0) = C_nb;
        // b. convert flu velocity into navigation frame:
        vel_ = C_nb * vel;

        // save init pose:
        init_pose_ = pose_;

        // init IMU data buffer:
        imu_data_buff_.clear();
        imu_data_buff_.push_back(imu_data); //   获取IMU数据

        // init filter time:
        time_ = imu_data.time_stamp_;

        // set process equation in case of one step prediction & correction:
        Eigen::Vector3d linear_acc_init(imu_data.linear_acceleration_.x,
                                        imu_data.linear_acceleration_.y,
                                        imu_data.linear_acceleration_.z);
        Eigen::Vector3d angular_vel_init(imu_data.angular_velocity_.x,
                                         imu_data.angular_velocity_.y,
                                         imu_data.angular_velocity_.z);
        // covert to navigation frame: 把IMU的 velocity angular（flu系）转换到导航 下
        linear_acc_init = linear_acc_init - accl_bias_;                   // body 系下
        angular_vel_init = GetUnbiasedAngularVel(angular_vel_init, C_nb); // body 系下
        // init process equation, in case of direct correct step:
        UpdateProcessEquation(linear_acc_init, angular_vel_init);

        LOG(INFO) << std::endl
                  << "Kalman Filter Inited at " << static_cast<int>(time_)
                  << std::endl
                  << "Init Position: " << pose_(0, 3) << ", " << pose_(1, 3) << ", " << pose_(2, 3)
                  << std::endl
                  << "Init Velocity: " << vel_.x() << ", " << vel_.y() << ", " << vel_.z()
                  << std::endl;
    }

    /**
     * @brief  Kalman update
     * @param  imu_data, input IMU measurements
     * @return true if success false otherwise
     */
    bool ESKF::Update(const ImuData &imu_data)
    {
        // update IMU buff:
        if (time_ < imu_data.time_stamp_)
        {
            // update IMU odometry:
            Eigen::Vector3d linear_acc_mid;
            Eigen::Vector3d angular_vel_mid;
            imu_data_buff_.push_back(imu_data);
            UpdateOdomEstimation(linear_acc_mid, angular_vel_mid); // 更新名义值，惯性解算
            imu_data_buff_.pop_front();

            // update error estimation:
            double T = imu_data.time_stamp_ - time_;                   // 滤波周期
            UpdateErrorEstimation(T, linear_acc_mid, angular_vel_mid); // 更新误差估计

            // move forward:
            time_ = imu_data.time_stamp_;

            return true;
        }

        return false;
    }

    /**
     * @brief  Kalman correction, pose measurement and other measurement in body
     * frame
     * @param  measurement_type, input measurement type
     * @param  measurement, input measurement
     * @return
     */
    bool ESKF::Correct(const ImuData &imu_data,
                       const MeasurementType &measurement_type,
                       const Measurement &measurement)
    {

        static Measurement measurement_;

        // get time delta:
        double time_delta = measurement.time - time_;

        // 时间对齐
        if (time_delta > -0.05)
        {
            // perform Kalman prediction:
            if (time_ < measurement.time)
            {
                Update(imu_data);
            }

            // get observation in navigation frame:
            measurement_ = measurement;
            measurement_.T_nb = init_pose_ * measurement_.T_nb;

            // correct error estimation:
            CorrectErrorEstimation(measurement_type, measurement_);

            // eliminate error:
            EliminateError(); // 更新名义值

            // reset error state:
            ResetState(); // 清零误差值，方差保留

            return true;
        }

        LOG(INFO) << "ESKF Correct: Observation is not synced with filter. Skip, "
                  << (int)measurement.time << " <-- " << (int)time_ << " @ "
                  << time_delta << std::endl;

        return false;
    }

    /**
     * @brief  get odometry estimation
     * @param  pose, init pose
     * @param  vel, init vel
     * @return void
     */
    void ESKF::GetOdometry(Eigen::Matrix4d &pose, Eigen::Vector3d &vel)
    {
        // init:
        Eigen::Matrix4d pose_double = pose_;
        Eigen::Vector3d vel_double = vel_;

        // eliminate error:
        // a. position:
        pose_double.block<3, 1>(0, 3) = pose_double.block<3, 1>(0, 3) - X_.block<3, 1>(kIndexErrorPos, 0);
        // b. velocity:
        vel_double = vel_double - X_.block<3, 1>(kIndexErrorVel, 0);
        // c. orientation:
        Eigen::Matrix3d C_nn = Sophus::SO3d::exp(X_.block<3, 1>(kIndexErrorOri, 0)).matrix();
        pose_double.block<3, 3>(0, 0) = C_nn * pose_double.block<3, 3>(0, 0);

        // finally:
        pose_double = init_pose_.inverse() * pose_double;
        vel_double = init_pose_.block<3, 3>(0, 0).transpose() * vel_double;

        pose = pose_double;
        vel = vel_double;
    }

    /**
     * @brief  get covariance estimation
     * @param  cov, covariance output
     * @return void
     */
    void ESKF::GetCovariance(Cov &cov)
    {
        static int OFFSET_X = 0;
        static int OFFSET_Y = 1;
        static int OFFSET_Z = 2;

        // a. delta position:
        cov.posi.x = P_(kIndexErrorPos + OFFSET_X, kIndexErrorPos + OFFSET_X);
        cov.posi.y = P_(kIndexErrorPos + OFFSET_Y, kIndexErrorPos + OFFSET_Y);
        cov.posi.z = P_(kIndexErrorPos + OFFSET_Z, kIndexErrorPos + OFFSET_Z);

        // b. delta velocity:
        cov.vel.x = P_(kIndexErrorVel + OFFSET_X, kIndexErrorVel + OFFSET_X);
        cov.vel.y = P_(kIndexErrorVel + OFFSET_Y, kIndexErrorVel + OFFSET_Y);
        cov.vel.z = P_(kIndexErrorVel + OFFSET_Z, kIndexErrorVel + OFFSET_Z);

        // c. delta orientation:
        cov.ori.x = P_(kIndexErrorOri + OFFSET_X, kIndexErrorOri + OFFSET_X);
        cov.ori.y = P_(kIndexErrorOri + OFFSET_Y, kIndexErrorOri + OFFSET_Y);
        cov.ori.z = P_(kIndexErrorOri + OFFSET_Z, kIndexErrorOri + OFFSET_Z);

        // d. gyro. bias:
        cov.gyro_bias.x = P_(kIndexErrorGyro + OFFSET_X, kIndexErrorGyro + OFFSET_X);
        cov.gyro_bias.y = P_(kIndexErrorGyro + OFFSET_Y, kIndexErrorGyro + OFFSET_Y);
        cov.gyro_bias.z = P_(kIndexErrorGyro + OFFSET_Z, kIndexErrorGyro + OFFSET_Z);

        // e. accel bias:
        cov.accel_bias.x = P_(kIndexErrorAccel + OFFSET_X, kIndexErrorAccel + OFFSET_X);
        cov.accel_bias.y = P_(kIndexErrorAccel + OFFSET_Y, kIndexErrorAccel + OFFSET_Y);
        cov.accel_bias.z = P_(kIndexErrorAccel + OFFSET_Z, kIndexErrorAccel + OFFSET_Z);
    }

    /**
     * @brief  get unbiased angular velocity in body frame
     * @param  angular_vel, angular velocity measurement
     * @param  R, corresponding orientation of measurement
     * @return unbiased angular velocity in body frame
     */
    inline Eigen::Vector3d ESKF::GetUnbiasedAngularVel(const Eigen::Vector3d &angular_vel, const Eigen::Matrix3d &R)
    {
        return angular_vel - gyro_bias_;
    }

    /**
     * @brief  get unbiased linear acceleration in navigation frame
     * @param  linear_acc, linear acceleration measurement
     * @param  R, corresponding orientation of measurement
     * @return unbiased linear acceleration in navigation frame
     */
    inline Eigen::Vector3d ESKF::GetUnbiasedLinearAcc(const Eigen::Vector3d &linear_acc, const Eigen::Matrix3d &R)
    {
        return R * (linear_acc - accl_bias_) - g_;
    }

    /**
     * @brief  get angular delta
     * @param  index_curr, current imu measurement buffer index
     * @param  index_prev, previous imu measurement buffer index
     * @param  angular_delta, angular delta output
     * @return true if success false otherwise
     */
    bool ESKF::GetAngularDelta(const size_t index_curr,
                               const size_t index_prev,
                               Eigen::Vector3d &angular_delta,
                               Eigen::Vector3d &angular_vel_mid)
    {
        if (index_curr <= index_prev || imu_data_buff_.size() <= index_curr)
        {
            return false;
        }

        const ImuData &imu_data_curr = imu_data_buff_.at(index_curr);
        const ImuData &imu_data_prev = imu_data_buff_.at(index_prev);

        double delta_t = imu_data_curr.time_stamp_ - imu_data_prev.time_stamp_;

        Eigen::Vector3d angular_vel_curr = Eigen::Vector3d(imu_data_curr.angular_velocity_.x,
                                                           imu_data_curr.angular_velocity_.y,
                                                           imu_data_curr.angular_velocity_.z);
        Eigen::Matrix3d R_curr = imu_data_curr.OrientationToMatrix().cast<double>();
        angular_vel_curr = GetUnbiasedAngularVel(angular_vel_curr, R_curr);

        Eigen::Vector3d angular_vel_prev = Eigen::Vector3d(imu_data_prev.angular_velocity_.x,
                                                           imu_data_prev.angular_velocity_.y,
                                                           imu_data_prev.angular_velocity_.z);
        Eigen::Matrix3d R_prev = imu_data_prev.OrientationToMatrix().cast<double>();
        angular_vel_prev = GetUnbiasedAngularVel(angular_vel_prev, R_prev);

        angular_delta = 0.5 * delta_t * (angular_vel_curr + angular_vel_prev);

        angular_vel_mid = 0.5 * (angular_vel_curr + angular_vel_prev);
        return true;
    }

    /**
     * @brief  update orientation with effective rotation angular_delta
     * @param  angular_delta, effective rotation
     * @param  R_curr, current orientation
     * @param  R_prev, previous orientation
     * @return void
     */
    void ESKF::UpdateOrientation(const Eigen::Vector3d &angular_delta, Eigen::Matrix3d &R_curr, Eigen::Matrix3d &R_prev)
    {
        // magnitude:
        double angular_delta_mag = angular_delta.norm();
        // direction:
        Eigen::Vector3d angular_delta_dir = angular_delta.normalized();

        // build delta q:
        double angular_delta_cos = cos(angular_delta_mag / 2.0);
        double angular_delta_sin = sin(angular_delta_mag / 2.0);
        Eigen::Quaterniond dq(angular_delta_cos,
                              angular_delta_sin * angular_delta_dir.x(),
                              angular_delta_sin * angular_delta_dir.y(),
                              angular_delta_sin * angular_delta_dir.z());
        Eigen::Quaterniond q(pose_.block<3, 3>(0, 0));

        // update:
        q = q * dq;

        // write back:
        R_prev = pose_.block<3, 3>(0, 0);
        pose_.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
        R_curr = pose_.block<3, 3>(0, 0);
    }

    /**
     * @brief  get velocity delta
     * @param  index_curr, current imu measurement buffer index
     * @param  index_prev, previous imu measurement buffer index
     * @param  R_curr, corresponding orientation of current imu measurement
     * @param  R_prev, corresponding orientation of previous imu measurement
     * @param  velocity_delta, velocity delta output
     * @param  linear_acc_mid, mid-value unbiased linear acc
     * @return true if success false otherwise
     */
    bool ESKF::GetVelocityDelta(const size_t index_curr, const size_t index_prev,
                                const Eigen::Matrix3d &R_curr, const Eigen::Matrix3d &R_prev, double &T,
                                Eigen::Vector3d &velocity_delta, Eigen::Vector3d &linear_acc_mid)
    {
        if (index_curr <= index_prev || imu_data_buff_.size() <= index_curr)
        {
            return false;
        }

        const ImuData &imu_data_curr = imu_data_buff_.at(index_curr);
        const ImuData &imu_data_prev = imu_data_buff_.at(index_prev);

        T = imu_data_curr.time_stamp_ - imu_data_prev.time_stamp_;

        Eigen::Vector3d linear_acc_curr = Eigen::Vector3d(imu_data_curr.linear_acceleration_.x,
                                                          imu_data_curr.linear_acceleration_.y,
                                                          imu_data_curr.linear_acceleration_.z);
        Eigen::Vector3d a_curr = GetUnbiasedLinearAcc(linear_acc_curr, R_curr); //  w系下的a_curr
        Eigen::Vector3d linear_acc_prev = Eigen::Vector3d(imu_data_prev.linear_acceleration_.x,
                                                          imu_data_prev.linear_acceleration_.y,
                                                          imu_data_prev.linear_acceleration_.z);
        Eigen::Vector3d a_prev = GetUnbiasedLinearAcc(linear_acc_prev, R_prev); //  w系下的a_prev
        // mid-value acc can improve error state prediction accuracy:
        linear_acc_mid = 0.5 * (a_curr + a_prev); //  w 系下的linear_acc_mid , 用于更新pos_w 和 vel_w
        velocity_delta = T * linear_acc_mid;

        linear_acc_mid = 0.5 * (linear_acc_curr + linear_acc_prev) - accl_bias_; //  b 系下的linear_acc_mid

        return true;
    }

    /**
     * @brief  update orientation with effective velocity change velocity_delta
     * @param  T, timestamp delta
     * @param  velocity_delta, effective velocity change
     * @return void
     */
    void ESKF::UpdatePosition(const double &T, const Eigen::Vector3d &velocity_delta)
    {
        pose_.block<3, 1>(0, 3) += T * vel_ + 0.5 * T * velocity_delta;
        vel_ += velocity_delta;
    }

    /**
     * @brief  update IMU odometry estimation 更新名义值
     * @param  linear_acc_mid, output mid-value unbiased linear acc
     * @return void
     */
    void ESKF::UpdateOdomEstimation(Eigen::Vector3d &linear_acc_mid, Eigen::Vector3d &angular_vel_mid)
    {
        // get deltas:
        size_t index_curr_ = 1;
        size_t index_prev_ = 0;
        Eigen::Vector3d angular_delta = Eigen::Vector3d::Zero();
        GetAngularDelta(index_curr_, index_prev_, angular_delta, angular_vel_mid); // 获取等效旋转矢量,保存角速度中值

        // update orientation:
        Eigen::Matrix3d R_curr_ = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d R_prev_ = Eigen::Matrix3d::Identity();
        UpdateOrientation(angular_delta, R_curr_, R_prev_); // 更新四元数

        // get velocity delta:
        double delta_t_;
        Eigen::Vector3d velocity_delta_;
        GetVelocityDelta(index_curr_, index_prev_, R_curr_, R_prev_, delta_t_, velocity_delta_, linear_acc_mid); // 获取速度差值，保存线加速度中值
        // save mid-value unbiased linear acc for error-state update:

        // update position:
        UpdatePosition(delta_t_, velocity_delta_);
    }

    /**
     * @brief  set process equation   更新状态方程  F矩阵
     * @param  C_nb, rotation matrix, body frame -> navigation frame
     * @param  f_n, accel measurement in navigation frame
     * @return void
     */
    void ESKF::SetProcessEquation(const Eigen::Matrix3d &C_nb,
                                  const Eigen::Vector3d &f_b,
                                  const Eigen::Vector3d &w_b)
    {
        // a. set process equation for delta vel:
        F_.setZero();
        F_.block<3, 3>(kIndexErrorPos, kIndexErrorVel) = Eigen::Matrix3d::Identity();
        F_.block<3, 3>(kIndexErrorVel, kIndexErrorOri) = -C_nb * Sophus::SO3d::hat(f_b).matrix();
        F_.block<3, 3>(kIndexErrorVel, kIndexErrorAccel) = -C_nb;
        F_.block<3, 3>(kIndexErrorOri, kIndexErrorOri) = -Sophus::SO3d::hat(w_b).matrix();
        F_.block<3, 3>(kIndexErrorOri, kIndexErrorGyro) = -Eigen::Matrix3d::Identity();
        // b. set process equation for delta ori:
        B_.setZero();
        B_.block<3, 3>(kIndexErrorVel, kIndexNoiseGyro) = C_nb;
        B_.block<3, 3>(kIndexErrorOri, kIndexNoiseGyro) = Eigen::Matrix3d::Identity();
        // 判断是否考虑随机游走
        B_.block<3, 3>(kIndexErrorAccel, kIndexNoiseBiasAccel) = Eigen::Matrix3d::Identity();
        B_.block<3, 3>(kIndexErrorGyro, kIndexNoiseBiasGyro) = Eigen::Matrix3d::Identity();
    }

    /**
     * @brief  update process equation
     * @param  imu_data, input IMU measurement
     * @param  T, output time delta
     * @return void
     */
    void ESKF::UpdateProcessEquation(const Eigen::Vector3d &linear_acc_mid, const Eigen::Vector3d &angular_vel_mid)
    {
        // set linearization point:
        Eigen::Matrix3d C_nb = pose_.block<3, 3>(0, 0); //  b2n   转换矩阵
        Eigen::Vector3d f_b = linear_acc_mid;           //  + g_  加速度
        Eigen::Vector3d w_b = angular_vel_mid;          //        角速度

        // set process equation:
        SetProcessEquation(C_nb, f_b, w_b);
    }

    /**
     * @brief  update error estimation 更新误差值
     * @param  linear_acc_mid, input mid-value unbiased linear acc
     * @return void
     */
    void ESKF::UpdateErrorEstimation(const double &T, const Eigen::Vector3d &linear_acc_mid, const Eigen::Vector3d &angular_vel_mid)
    {
        static MatrixF F_1st;
        static MatrixF F_2nd;
        // 更新状态方程
        UpdateProcessEquation(linear_acc_mid, angular_vel_mid);
        // 非线性化
        F_1st = F_ * T; //  T kalman 周期
        MatrixF F = MatrixF::Identity() + F_1st;
        MatrixB B = MatrixB::Zero();
        B.block<3, 3>(kIndexErrorVel, kIndexNoiseGyro) = B_.block<3, 3>(kIndexErrorVel, kIndexNoiseGyro) * T;
        B.block<3, 3>(kIndexErrorOri, kIndexNoiseGyro) = B_.block<3, 3>(kIndexErrorOri, kIndexNoiseGyro) * T;
        B.block<3, 3>(kIndexErrorAccel, kIndexNoiseBiasAccel) = B_.block<3, 3>(kIndexErrorAccel, kIndexNoiseBiasAccel) * sqrt(T);
        B.block<3, 3>(kIndexErrorGyro, kIndexNoiseBiasGyro) = B_.block<3, 3>(kIndexErrorGyro, kIndexNoiseBiasGyro) * sqrt(T);

        // perform Kalman prediction
        X_ = F * X_;
        P_ = F * P_ * F.transpose() + B * Q_ * B.transpose(); // 只有方差进行了计算
    }

    /**
     * @brief  correct error estimation using pose measurement 计算  Y ，G  ，K
     * @param  T_nb, input pose measurement
     * @return void
     */
    void ESKF::CorrectErrorEstimationPose(const Eigen::Matrix4d &T_nb, Eigen::VectorXd &Y, Eigen::MatrixXd &G, Eigen::MatrixXd &K)
    {
        // 计算观测 delta pos 、delta  ori
        Eigen::Vector3d dp = pose_.block<3, 1>(0, 3) - T_nb.block<3, 1>(0, 3);
        Eigen::Matrix3d dR = T_nb.block<3, 3>(0, 0).transpose() * pose_.block<3, 3>(0, 0);
        // set measurement equation:
        Eigen::Vector3d dtheta = Sophus::SO3d::vee(dR - Eigen::Matrix3d::Identity());
        YPose_.block<3, 1>(0, 0) = dp;     // delta  position
        YPose_.block<3, 1>(3, 0) = dtheta; // 失准角
        Y = YPose_;
        // set measurement  G
        GPose_.setZero();
        GPose_.block<3, 3>(0, kIndexErrorPos) = Eigen::Matrix3d::Identity();
        GPose_.block<3, 3>(3, kIndexErrorOri) = Eigen::Matrix3d::Identity();
        G = GPose_;
        // set measurement  C
        CPose_.setZero();
        CPose_.block<3, 3>(0, kIndexNoiseAccel) = Eigen::Matrix3d::Identity();
        CPose_.block<3, 3>(3, kIndexNoiseGyro) = Eigen::Matrix3d::Identity();
        Eigen::MatrixXd C = CPose_;
        // set Kalman gain:
        Eigen::MatrixXd R = RPose_; // 观测噪声
        K = P_ * G.transpose() * (G * P_ * G.transpose() + C * RPose_ * C.transpose()).inverse();
    }

    /**
     * @brief  correct error estimation
     * @param  measurement_type, measurement type
     * @param  measurement, input measurement
     * @return void
     */
    void ESKF::CorrectErrorEstimation(const MeasurementType &measurement_type, const Measurement &measurement)
    {
        Eigen::VectorXd Y;
        Eigen::MatrixXd G, K;
        switch (measurement_type)
        {
        case MeasurementType::POSE:
            CorrectErrorEstimationPose(measurement.T_nb, Y, G, K);
            break;
        default:
            break;
        }

        // Tperform Kalman correct:
        P_ = (MatrixP::Identity() - K * G) * P_; //  后验方差
        X_ = X_ + K * (Y - G * X_);              //  更新后的状态量
    }

    /**
     * @brief  eliminate error
     * @param  void
     * @return void
     */
    void ESKF::EliminateError(void)
    {
        // 误差状态量  X_  :   15*1
        // correct state estimation using the state of ESKF
        // a. position:
        pose_.block<3, 1>(0, 3) -= X_.block<3, 1>(kIndexErrorPos, 0); // 减去error
        // b. velocity:
        vel_ -= X_.block<3, 1>(kIndexErrorVel, 0);
        // c. orientation:
        Eigen::Matrix3d dtheta_cross = Sophus::SO3d::hat(X_.block<3, 1>(kIndexErrorOri, 0)); // 失准角的反对称矩阵
        pose_.block<3, 3>(0, 0) = pose_.block<3, 3>(0, 0) * (Eigen::Matrix3d::Identity() - dtheta_cross);
        Eigen::Quaterniond q_tmp(pose_.block<3, 3>(0, 0));
        q_tmp.normalize(); // 为了保证旋转矩阵是正定的
        pose_.block<3, 3>(0, 0) = q_tmp.toRotationMatrix();

        // d. gyro bias:
        if (IsCovStable(kIndexErrorGyro))
        {
            gyro_bias_ -= X_.block<3, 1>(kIndexErrorGyro, 0); // 判断gyro_bias_error是否可观
        }

        // e. accel bias:
        if (IsCovStable(kIndexErrorAccel))
        {
            accl_bias_ -= X_.block<3, 1>(kIndexErrorAccel, 0); // 判断accel_bias_error是否可观
        }
    }

    /**
     * @brief  is covariance stable
     * @param  INDEX_OFSET, state index offset
     * @param  THRESH, covariance threshold, defaults to 1.0e-5
     * @return void
     */
    bool ESKF::IsCovStable(const int INDEX_OFSET, const double THRESH)
    {
        for (int i = 0; i < 3; ++i)
        {
            if (P_(INDEX_OFSET + i, INDEX_OFSET + i) > THRESH)
            {
                return false;
            }
        }

        return true;
    }

    /**
     * @brief  reset filter state
     * @param  void
     * @return void
     */
    void ESKF::ResetState(void)
    {
        // reset current state:
        X_ = VectorX::Zero();
    }

    /**
     * @brief  reset filter covariance
     * @param  void
     * @return void
     */
    void ESKF::ResetCovariance(void)
    {
        P_ = MatrixP::Zero();

        P_.block<3, 3>(kIndexErrorPos, kIndexErrorPos) = COV.PRIOR.POSI * Eigen::Matrix3d::Identity();
        P_.block<3, 3>(kIndexErrorVel, kIndexErrorVel) = COV.PRIOR.VEL * Eigen::Matrix3d::Identity();
        P_.block<3, 3>(kIndexErrorOri, kIndexErrorOri) = COV.PRIOR.ORI * Eigen::Matrix3d::Identity();
        P_.block<3, 3>(kIndexErrorGyro, kIndexErrorGyro) = COV.PRIOR.EPSILON * Eigen::Matrix3d::Identity();
        P_.block<3, 3>(kIndexErrorAccel, kIndexErrorAccel) = COV.PRIOR.DELTA * Eigen::Matrix3d::Identity();
    }

    /**
     * @brief  get Q analysis for pose measurement
     * @param  void
     * @return void
     */
    void ESKF::GetQPose(Eigen::MatrixXd &Q, Eigen::VectorXd &Y)
    {
        // build observability matrix for position measurement:
        Y = Eigen::VectorXd::Zero(kDimState * kDimMeasurementPose);
        Y.block<kDimMeasurementPose, 1>(0, 0) = YPose_;
        for (int i = 1; i < kDimState; ++i)
        {
            QPose_.block<kDimMeasurementPose, kDimState>(i * kDimMeasurementPose, 0) =
                (QPose_.block<kDimMeasurementPose, kDimState>((i - 1) * kDimMeasurementPose, 0) * F_);

            Y.block<kDimMeasurementPose, 1>(i * kDimMeasurementPose, 0) = YPose_;
        }

        Q = QPose_;
    }

    /**
     * @brief  update observability analysis
     * @param  measurement_type, measurement type
     * @return void
     */
    void ESKF::UpdateObservabilityAnalysis(const double &time, const MeasurementType &measurement_type)
    {
        // get Q:
        Eigen::MatrixXd Q;
        Eigen::VectorXd Y;
        switch (measurement_type)
        {
        case MeasurementType::POSE:
            GetQPose(Q, Y);
            break;
        default:
            break;
        }

        observability.time_.push_back(time);
        observability.Q_.push_back(Q);
        observability.Y_.push_back(Y);
    }

    /**
     * @brief  save observability analysis to persistent storage
     * @param  measurement_type, measurement type
     * @return void
     */
    bool ESKF::SaveObservabilityAnalysis(const MeasurementType &measurement_type)
    {
        // get fusion strategy:
        std::string type;
        switch (measurement_type)
        {
        case MeasurementType::POSE:
            type = std::string("pose");
            break;
        case MeasurementType::POSE_VEL:
            type = std::string("pose_velocity");
            break;
        case MeasurementType::POSI:
            type = std::string("position");
            break;
        case MeasurementType::POSI_VEL:
            type = std::string("position_velocity");
            break;
        default:
            return false;
            break;
        }

        // build Q_so:
        const int N = observability.Q_.at(0).rows();

        std::vector<std::vector<double>> q_data, q_so_data;

        Eigen::MatrixXd Qso(observability.Q_.size() * N, kDimState);
        Eigen::VectorXd Yso(observability.Y_.size() * N);

        for (size_t i = 0; i < observability.Q_.size(); ++i)
        {
            const double &time = observability.time_.at(i);

            const Eigen::MatrixXd &Q = observability.Q_.at(i);
            const Eigen::VectorXd &Y = observability.Y_.at(i);

            Qso.block(i * N, 0, N, kDimState) = Q;
            Yso.block(i * N, 0, N, 1) = Y;

            KalmanFilterInterface::AnalyzeQ(kDimState, time, Q, Y, q_data);

            if (0 < i && (0 == i % 10))
            {
                KalmanFilterInterface::AnalyzeQ(kDimState, observability.time_.at(i - 5),
                                                Qso.block((i - 10), 0, 10 * N, kDimState),
                                                Yso.block((i - 10), 0, 10 * N, 1), q_so_data);
            }
        }

        std::string q_data_csv =
            WORK_SPACE_PATH + "/slam_data/observability/" + type + ".csv";
        std::string q_so_data_csv =
            WORK_SPACE_PATH + "/slam_data/observability/" + type + "_som.csv";

        KalmanFilterInterface::WriteAsCsv(kDimState, q_data, q_data_csv);
        KalmanFilterInterface::WriteAsCsv(kDimState, q_so_data, q_so_data_csv);

        return true;
    }

} // namespace robot_localization
