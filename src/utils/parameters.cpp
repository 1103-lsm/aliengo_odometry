/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "yaml-cpp/yaml.h"
#include "parameters.h"
#include "utility.h"


double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_N_Z, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string DATASET_NAME;
std::string VILO_RESULT_PATH;
std::string OUTPUT_FOLDER;
std::string IMU_TOPIC;

std::string POSE_TOPIC;
std::string TWIST_TOPIC;

int ROW, COL;
double TD;
int NUM_OF_CAM;
int STEREO;
int USE_IMU;
int USE_LEG;
int MULTIPLE_THREAD;
map<int, Eigen::Vector3d> pts_gt;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;

std::string ROBOT_TYPE;

std::string LEG_TOPIC;
int OPTIMIZE_LEG_BIAS;
int CONTACT_SENSOR_TYPE;
// temporarily write some parameters here
double PHI_N;
double DPHI_N;
double RHO_C_N;
double RHO_NC_N;

double V_N_MIN_XY;
double V_N_MIN_Z;
double V_N_MIN;
double V_N_MAX;

double V_N_FORCE_THRES_RATIO;
double V_N_TERM1_STEEP;
double V_N_TERM2_VAR_RESCALE;
double V_N_TERM3_DISTANCE_RESCALE;

double VILO_LOWER_LEG_LENGTH;

int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

/******************从配置文件读取参数************************/
void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(), "r");
    if (fh == NULL)
    {
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;
    }
    fclose(fh);

    // 创建一个 YAML 对象
    YAML::Node fsSettings;

    try{
        fsSettings = YAML::LoadFile(config_file);
    } catch(YAML::BadFile &e) {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    POSE_TOPIC = fsSettings["pose_topic"].as<string>();
    TWIST_TOPIC = fsSettings["twist_topic"].as<string>();

    MULTIPLE_THREAD = fsSettings["multiple_thread"].as<int>();

    /**********IMU配置**************/
    // fsSettings["imu_topic"] >> IMU_TOPIC;
    IMU_TOPIC = fsSettings["imu_topic"].as<string>();
    printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
    ACC_N = fsSettings["acc_n"].as<double>();
    ACC_N_Z = fsSettings["acc_n_z"].as<double>();
    ACC_W = fsSettings["acc_w"].as<double>();
    GYR_N = fsSettings["gyr_n"].as<double>();
    GYR_W = fsSettings["gyr_w"].as<double>();
    G.z() = fsSettings["g_norm"].as<double>();

    /**********腿部里程计配置**************/
    // fsSettings["robot_type"] >> ROBOT_TYPE;
    ROBOT_TYPE = fsSettings["robot_type"].as<string>();

    // fsSettings["leg_topic"] >> LEG_TOPIC;
    LEG_TOPIC = fsSettings["leg_topic"].as<string>();
    // fsSettings["dataset_name"] >> DATASET_NAME;
    DATASET_NAME = fsSettings["dataset_name"].as<string>();
    printf("LEG_TOPIC: %s\n", LEG_TOPIC.c_str());

    OPTIMIZE_LEG_BIAS = fsSettings["optimize_leg_bias"].as<int>();
    printf("leg optimize_leg_bias %d\n", OPTIMIZE_LEG_BIAS);

    CONTACT_SENSOR_TYPE = fsSettings["contact_sensor_type"].as<int>();
    printf("contact sensor type %d\n", CONTACT_SENSOR_TYPE);

    PHI_N = fsSettings["joint_angle_n"].as<double>();
    DPHI_N = fsSettings["joint_velocity_n"].as<double>();
    RHO_C_N = fsSettings["leg_bias_c_n"].as<double>();
    RHO_NC_N = fsSettings["leg_bias_nc_n"].as<double>();

    V_N_MIN_XY = fsSettings["v_n_min_xy"].as<double>();
    V_N_MIN_Z = fsSettings["v_n_min_z"].as<double>();
    V_N_MIN = fsSettings["v_n_min"].as<double>();
    V_N_MAX = fsSettings["v_n_max"].as<double>();
    V_N_FORCE_THRES_RATIO = fsSettings["v_n_force_thres_ratio"].as<double>();
    V_N_TERM1_STEEP = fsSettings["v_n_term1_steep"].as<double>();
    V_N_TERM2_VAR_RESCALE = fsSettings["v_n_term2_var_rescale"].as<double>();
    V_N_TERM3_DISTANCE_RESCALE = fsSettings["v_n_term3_distance_rescale"].as<double>();

    VILO_LOWER_LEG_LENGTH = fsSettings["lower_leg_length"].as<double>();
    /**************************************************/

    SOLVER_TIME = fsSettings["max_solver_time"].as<double>();
    NUM_ITERATIONS = fsSettings["max_num_iterations"].as<int>();
    MIN_PARALLAX = fsSettings["keyframe_parallax"].as<double>();
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    // fsSettings["output_path"] >> OUTPUT_FOLDER;
    OUTPUT_FOLDER = fsSettings["output_path"].as<string>();

    if (OPTIMIZE_LEG_BIAS)
    {
        VILO_RESULT_PATH = OUTPUT_FOLDER + "/vilo_wb" + Utility::GetCurrentTimeForFileName() +
                           "-" + DATASET_NAME + "-"
                                                "-c-" +
                           to_string(CONTACT_SENSOR_TYPE) + ".csv";
    }
    else
    {
        VILO_RESULT_PATH = OUTPUT_FOLDER + "/vilo_wob" + Utility::GetCurrentTimeForFileName() +
                           "-" + DATASET_NAME + "-"
                                                "-c-" +
                           to_string(CONTACT_SENSOR_TYPE) + ".csv";
    }
    std::cout << "result path " << VILO_RESULT_PATH << std::endl;
    std::ofstream fout(VILO_RESULT_PATH, std::ios::out);
    fout.close();

    // int pn = config_file.find_last_of('/');

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    // fsSettings.release();
}
