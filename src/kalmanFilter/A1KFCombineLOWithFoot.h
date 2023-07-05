#pragma once

#include <mutex>

#include <Eigen/Dense>
#include <casadi/casadi.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include "A1KF.hpp"
#include "../legKinematics/A1Kinematics.h"

/*
   Baseline 3
   This filter follows http://www.roboticsproceedings.org/rss08/p03.pdf
   We do not consider bias here

   state = [x, y, z, vx, vy, vz, euler_roll, euler_pitch, euler_yaw, foot1, foot2, foot3, foot4, tk]
           [0, 1, 2,  3,  4,  5,  6,        7,        8,    9:11, 12:14, 15:17. 18:20    21]

   ctrl = [acc_x, acc_y, acc_z, ang_vel_x, ang_vel_y, ang_vel_z, dt]
            [0,    1,    2,     3,        4,        5,        6]

   observation = [foot1 pos, foot2 pos, foot3 pos, foot4 pos, foot1 vel, foot2 vel, foot3 vel, foot4 vel]
                     [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23]

 */
#define EKF_STATE_SIZE 22 // 状态向量的维度
#define CONTROL_SIZE 7 // 控制向量的维度
#define OBSERVATION_SIZE 24 // 观测向量的维度

#define OPTI_OBSERVATION_SIZE 6 // 使用OptiTrack系统进行观测时的观测向量的维度大小 observe position and velocity using optitrack
#define UPDATE_DT 0.002 // 滤波器的更新时间间隔（以秒为单位）

class A1KFCombineLOWithFoot : public A1KF
{
public:
    A1KFCombineLOWithFoot();
    void init_filter(A1SensorData data, Eigen::Vector3d _init_pos = Eigen::Vector3d(0, 0, 0.15));
    void update_filter(A1SensorData data);
    void update_filter_with_opti(A1SensorData data);

    Eigen::Matrix<double, EKF_STATE_SIZE, 1> get_state() { return curr_state; }
    Eigen::Matrix<double, NUM_LEG, 1> get_contacts() { return estimated_contact; }

private:
    void load_casadi_functions();
    void process(Eigen::Matrix<double, EKF_STATE_SIZE, 1> state,
                 Eigen::Matrix<double, CONTROL_SIZE, 1> prev_ctrl,
                 Eigen::Matrix<double, CONTROL_SIZE, 1> ctrl, double dt);

    void measure(Eigen::Matrix<double, EKF_STATE_SIZE, 1> state,
                 Eigen::Matrix<double, 3, 1> w,
                 Eigen::Matrix<double, 12, 1> joint_ang,
                 Eigen::Matrix<double, 12, 1> joint_vel);
    Eigen::Matrix<double, EKF_STATE_SIZE, 1> curr_state; // 当前状态向量
    Eigen::Matrix<double, EKF_STATE_SIZE, EKF_STATE_SIZE> curr_covariance; // 当前状态协方差矩阵，表示状态向量中各个元素之间的协方差

    Eigen::Matrix<double, EKF_STATE_SIZE, 1> x01;                           // 中间状态向量，用于存储EKF更新过程中的临时状态 intermediate state
    Eigen::Matrix<double, EKF_STATE_SIZE, EKF_STATE_SIZE> process_jacobian; // 中间协方差矩阵，用于存储EKF更新过程中的临时协方差 intermediate covariance
    Eigen::Matrix<double, EKF_STATE_SIZE, EKF_STATE_SIZE> P01;              // 中间协方差矩阵，表示中间状态向量的协方差 intermediate covariance

    Eigen::Matrix<double, OBSERVATION_SIZE, 1> measurement; // 观测向量
    Eigen::Matrix<double, OBSERVATION_SIZE, EKF_STATE_SIZE> measurement_jacobian; // 观测向量的雅可比矩阵，用于将状态向量映射到观测向量

    Eigen::Matrix<double, CONTROL_SIZE, 1> prev_ctrl; // 上一个控制向量，包含7个元素，表示上一个时刻的控制输入
    Eigen::Matrix<double, CONTROL_SIZE, 1> curr_ctrl; // 当前控制向量，包含7个元素，表示当前时刻的控制输入

    Eigen::Matrix<double, EKF_STATE_SIZE, EKF_STATE_SIZE> process_noise; // 过程噪声协方差矩阵，表示系统模型中的噪声
    Eigen::Matrix<double, OBSERVATION_SIZE, OBSERVATION_SIZE> measure_noise; // 观测噪声协方差矩阵，表示传感器观测中的噪声

    // optitrack related
    Eigen::Matrix<double, OPTI_OBSERVATION_SIZE, EKF_STATE_SIZE> opti_jacobian; // OptiTrack观测向量的雅可比矩阵，用于将状态向量映射到OptiTrack观测向量
    Eigen::Matrix<double, OPTI_OBSERVATION_SIZE, OPTI_OBSERVATION_SIZE> opti_noise; // OptiTrack观测噪声协方差矩阵

    std::mutex update_mutex; // 互斥锁，用于保护滤波器的更新过程

    /* CasADi函数，用于计算EKF的过程模型和观测模型以及它们的雅可比矩阵 */
    casadi::Function process_func;
    casadi::Function process_jac_func;
    casadi::Function measure_func;
    casadi::Function measure_jac_func;

    A1Kinematics kinematics; // A1机器人的运动学模型，用于计算关节角度和脚部位置等
    /* 四条腿的在机器人身体坐标系下的偏移量 */
    double leg_offset_x[4] = {};
    double leg_offset_y[4] = {};
    // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
    /* 四条腿的电机偏移量，表示电机与机器人身体坐标系之间的偏移 */
    double motor_offset[4] = {};
    /* 四条腿的上腿长度和下腿长度 */
    double upper_leg_length[4] = {};
    double lower_leg_length[4] = {};
    /* 用于存储固定腿和OptiTrack腿的位置和速度 */
    std::vector<Eigen::VectorXd> rho_fix_list;
    std::vector<Eigen::VectorXd> rho_opt_list;

    // estimated contact (see which foot velocity agrees with state_v)
    /* 估计的腿部接触状态，用于确定哪个脚部的速度与状态向量中的速度一致 */
    Eigen::Matrix<double, NUM_LEG, 1> estimated_contact;

    // helper matrices
    Eigen::Matrix<double, 3, 3> eye3; // 3x3 identity
    Eigen::Matrix<double, OBSERVATION_SIZE, OBSERVATION_SIZE> S; // 观测残差协方差矩阵，用于计算卡尔曼增益
};