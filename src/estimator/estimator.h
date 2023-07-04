#include <thread>
#include <iomanip>
#include <mutex>
#include <algorithm>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <unordered_map>
#include <vector>
#include <deque>
#include <queue>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "../utils/visualization.h"
#include "../utils/utility.h"
#include "../factor/imu_factor.h"
#include "../factor/imu_leg_factor.h"

class Estimator
{ 
public:
    Estimator();
    ~Estimator();
    
    void setParameter();

        // most important
    void processMeasurements();

    void inputIMU(double t, const Eigen::Vector3d &linearAcceleration, const Eigen::Vector3d &angularVelocity);
    void inputLeg(double t, const Eigen::Ref<const Vector_dof> &jointAngles,
                         const Eigen::Ref<const Vector_dof> &jointVels, const Eigen::Ref<const Vector_leg> &contact_flags);
    void processIMU(double t, double dt, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity);
    void processIMULeg(double t, double dt,
                       const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity,
                       const Eigen::Ref<const Vector_dof> &joint_angle, const Eigen::Ref<const Vector_dof> &joint_velocity,
                       const Eigen::Ref<const Vector_leg> &foot_contact);


    void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity);
    void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);

    bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector,
                        vector<pair<double, Eigen::Vector3d>> &gyrVector);
    bool getIMUAndLegInterval(double t0, double t1,
                              vector<pair<double, Eigen::Vector3d>> &accVector,
                              vector<pair<double, Eigen::Vector3d>> &gyrVector,
                              vector<pair<double, Vector_dof>> &jointAngVector,
                              vector<pair<double, Vector_dof>> &jointVelVector,
                              vector<pair<double, Vector_leg>> &contactFlagVector);
    bool IMUAvailable(double t);



    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    std::mutex mProcess;
    std::mutex mBuf;
    std::mutex mPropagate;

    std::thread processThread;

    double prevTime, curTime;

    // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
    double leg_offset_x[4] = {};
    double leg_offset_y[4] = {};
    // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
    double motor_offset[4] = {};
    double upper_leg_length[4] = {};
    double lower_leg_length[4] = {};

    // the first param is time
    std::queue<std::pair<double, Eigen::Vector3d>> accBuf;
    std::queue<std::pair<double, Eigen::Vector3d>> gyrBuf;

    std::queue<std::pair<double, Vector_dof>> legAngBufList;
    std::queue<std::pair<double, Vector_dof>> legAngVelBufList;
    std::queue<std::pair<double, Vector_leg>> contactFlagBufList;

    SolverFlag solver_flag;

    double latest_time; // used in fastPredictIMU to get dt from last function call
    // used in fastPredictIMU and updateLatestStates
    Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0;
    Eigen::Quaterniond latest_Q;

    Eigen::Vector3d g;

    Eigen::Vector3d acc_0, gyr_0;

    int leg_msg_counter;

    bool first_imu;

    bool initFirstPoseFlag;
    bool initThreadFlag;

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    IMULegIntegrationBase *il_pre_integrations[(WINDOW_SIZE + 1)];

    IntegrationBase *tmp_pre_integration;
    IMULegIntegrationBase *tmp_il_pre_integration;

    int frame_count;

    Eigen::Vector3d Bas[(WINDOW_SIZE + 1)];
    Eigen::Vector3d Bgs[(WINDOW_SIZE + 1)];
    Eigen::Vector3d Ps[(WINDOW_SIZE + 1)];
    Eigen::Vector3d Vs[(WINDOW_SIZE + 1)];
    Eigen::Matrix3d Rs[(WINDOW_SIZE + 1)];

    std::vector<double> dt_buf[(WINDOW_SIZE + 1)];
    std::vector<Eigen::Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    std::vector<Eigen::Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];
    std::vector<Vector_dof> joint_angle_buf[(WINDOW_SIZE + 1)];
    std::vector<Vector_dof> joint_velocity_buf[(WINDOW_SIZE + 1)];
    std::vector<Vector_leg> foot_contact_buf[(WINDOW_SIZE + 1)];

    Vector_dof phi_0, dphi_0;
    Vector_leg c_0;

    // 创建 RHO_OPT_SIZE X 1 矩阵
    Eigen::Matrix<double, RHO_OPT_SIZE, 1> Rho1[(WINDOW_SIZE + 1)];
    Eigen::Matrix<double, RHO_OPT_SIZE, 1> Rho2[(WINDOW_SIZE + 1)];
    Eigen::Matrix<double, RHO_OPT_SIZE, 1> Rho3[(WINDOW_SIZE + 1)];
    Eigen::Matrix<double, RHO_OPT_SIZE, 1> Rho4[(WINDOW_SIZE + 1)];

    std::vector<Eigen::VectorXd> rho_fix_list;

    // add leg kinematics
    // the leg kinematics is relative to body frame, which is the center of the robot
    // following are some parameters that defines the transformation between IMU frame(b) and robot body frame(r)
    Eigen::Vector3d p_br; // IMU和机器人主体坐标系之间的平移偏移量
    Eigen::Matrix3d R_br; // IMU和机器人主体坐标系之间的旋转矩阵（旋转偏移量）

};