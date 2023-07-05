#include "ros/ros.h"
#include "utils/parameters.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
// #include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "kalmanFilter/A1KFCombineLOWithFoot.h"
#include "estimator/estimator.h"

// kf for estimating contact
// sensor process to filter imu and leg data
A1SensorData sensor_data;
Estimator estimator;
A1KFCombineLOWithFoot kf; // Kalman filter Baseline 3 with foot
double curr_t;
bool first_sensor_received = false;

// debug print filtered data
ros::Publisher filterd_imu_pub;
ros::Publisher filterd_joint_pub;
ros::Publisher filterd_pos_pub;

ros::Publisher pub_truth_path;
ros::Publisher pub_truth_odometry;

// truth pose
nav_msgs::Path truth_path;
nav_msgs::Odometry truth_odometry;

void sensor_callback(const sensor_msgs::Imu::ConstPtr &imu_msg, const sensor_msgs::JointState::ConstPtr &joint_msg);
void geometry_callback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg, const geometry_msgs::TwistStamped::ConstPtr& twistMsg);

int main(int argc, char *argv[])
{
    // 执行 ros 节点初始化
    ros::init(argc,argv,"ilo");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    // 输入两个参数
    if (argc != 2)
    {
        printf("please intput: rosrun ilo ilo [config file] \n"
               "for example: rosrun ilo ilo src/go/config/aliengo_config/aliengo_ilo_config.yaml");
        return 1;
    }

    // 读取 yaml 文件参数
    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);
    readParameters(config_file);

    estimator.setParameter();

    /***sync IMU and leg, we assume that IMU and leg, although come as two separate topics, are actually has the same time stamp***/
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
    message_filters::Subscriber<sensor_msgs::JointState> joint_state_sub;
    imu_sub.subscribe(n, IMU_TOPIC, 200);
    joint_state_sub.subscribe(n, LEG_TOPIC, 200);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::JointState> imu_leg_SyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<imu_leg_SyncPolicy> imu_leg_sync(imu_leg_SyncPolicy(30), imu_sub, joint_state_sub);

    imu_leg_sync.registerCallback(boost::bind(&sensor_callback, _1, _2));
    /****************************************************************************************************************************/

    /************************************************** truth data *****************************************************************/
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(n, POSE_TOPIC, 200); // 200 为队列深度
    message_filters::Subscriber<geometry_msgs::TwistStamped> twist_sub(n, TWIST_TOPIC, 200); 

    // 定义同步策略（近似时间同步）
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> pose_twist_SyncPolicy;
    message_filters::Synchronizer<pose_twist_SyncPolicy> pose_twist_sync(pose_twist_SyncPolicy(30), pose_sub, twist_sub);

    pose_twist_sync.registerCallback(boost::bind(&geometry_callback, _1, _2));

    /********************************************************************************************************************************/

    /* publishers */
    filterd_imu_pub = n.advertise<sensor_msgs::Imu>("/a1_filterd_imu", 30);
    filterd_joint_pub = n.advertise<sensor_msgs::JointState>("/a1_filterd_joint", 30);
    filterd_pos_pub = n.advertise<nav_msgs::Odometry>("/a1_filterd_pos", 30);
    // registerPub(n);

    pub_truth_path = n.advertise<nav_msgs::Path>("truth_path", 1000);
    pub_truth_odometry = n.advertise<nav_msgs::Odometry>("truth_odometry", 1000);

    // 循环处理消息
    ros::spin();

    return 0;
}


// we assume IMU and leg have the same timestamp
void sensor_callback(const sensor_msgs::Imu::ConstPtr &imu_msg, const sensor_msgs::JointState::ConstPtr &joint_msg)
{
    static int counter = 0;
    counter++;
    double t = imu_msg->header.stamp.toSec();

    // assemble sensor data
    Eigen::Vector3d acc = Eigen::Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
    Eigen::Vector3d ang_vel = Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);

    Eigen::Matrix<double, NUM_DOF, 1> joint_pos;
    Eigen::Matrix<double, NUM_DOF, 1> joint_vel;
    Eigen::Matrix<double, NUM_LEG, 1> plan_contacts;
    Eigen::Matrix<double, NUM_LEG, 1> foot_force_sensor_readings;
    for (int i = 0; i < NUM_DOF; ++i)
    {
        joint_pos[i] = joint_msg->position[i];
        joint_vel[i] = joint_msg->velocity[i];
    }
    for (int i = 0; i < NUM_LEG; ++i)
    {
        plan_contacts[i] = joint_msg->velocity[NUM_DOF + i];
        foot_force_sensor_readings[i] = joint_msg->effort[NUM_DOF + i];
    }

    double dt;
    sensor_data.input_imu(acc, ang_vel);
    // TODO: 这里 foot force sensor data 的输入为什么是 plan_contacts ？？？
    sensor_data.input_leg(joint_pos, joint_vel, plan_contacts);

    if (!kf.is_inited() && first_sensor_received == false)
    {
        // the callback is called the first time, filter may not be inited
        dt = 0;
        curr_t = t;
        sensor_data.input_dt(dt);
        // init the filter
        kf.init_filter(sensor_data);
    }
    else if (!kf.is_inited())
    {
        // filter may not be inited even after the callback is called multiple times
        dt = t - curr_t;
        sensor_data.input_dt(dt);
        curr_t = t;
    }
    else
    {
        dt = t - curr_t;

        sensor_data.input_dt(dt);
        kf.update_filter(sensor_data);
        curr_t = t;
    }

    // get filtered data from data and kf to estimator
    estimator.inputIMU(t, sensor_data.acc, sensor_data.ang_vel);

    if (CONTACT_SENSOR_TYPE == 0)
    {
        estimator.inputLeg(t, sensor_data.joint_pos, sensor_data.joint_vel, kf.get_contacts());
    }
    else if (CONTACT_SENSOR_TYPE == 1)
    {
        estimator.inputLeg(t, sensor_data.joint_pos, sensor_data.joint_vel, sensor_data.plan_contacts);
    }
    else if (CONTACT_SENSOR_TYPE == 2)
    {
        estimator.inputLeg(t, sensor_data.joint_pos, sensor_data.joint_vel, foot_force_sensor_readings);
    }

    // debug print filtered data
    sensor_msgs::Imu filterd_imu_msg;
    sensor_msgs::JointState filterd_joint_msg;
    filterd_imu_msg.header.stamp = ros::Time::now();
    filterd_imu_msg.linear_acceleration.x = sensor_data.acc[0];
    filterd_imu_msg.linear_acceleration.y = sensor_data.acc[1];
    filterd_imu_msg.linear_acceleration.z = sensor_data.acc[2];

    filterd_imu_msg.angular_velocity.x = sensor_data.ang_vel[0];
    filterd_imu_msg.angular_velocity.y = sensor_data.ang_vel[1];
    filterd_imu_msg.angular_velocity.z = sensor_data.ang_vel[2];

    filterd_joint_msg.header.stamp = ros::Time::now();

    // use joint names in urdf so we can visualize the robot
    filterd_joint_msg.name = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                              "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                              "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
                              "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                              "FL_foot", "FR_foot", "RL_foot", "RR_foot"};
    filterd_joint_msg.position.resize(NUM_DOF + NUM_LEG);
    filterd_joint_msg.velocity.resize(NUM_DOF + NUM_LEG);
    filterd_joint_msg.effort.resize(NUM_DOF + NUM_LEG);
    for (int i = 0; i < NUM_DOF; ++i)
    {
        filterd_joint_msg.position[i] = sensor_data.joint_pos[i];
        filterd_joint_msg.velocity[i] = sensor_data.joint_vel[i];
    }
    Eigen::Vector4d estimated_contact = kf.get_contacts();
    for (int i = 0; i < NUM_LEG; ++i)
    {
        if (CONTACT_SENSOR_TYPE == 0)
        {
            filterd_joint_msg.velocity[NUM_DOF + i] = estimated_contact[i];
        }
        else if (CONTACT_SENSOR_TYPE == 1)
        {
            filterd_joint_msg.velocity[NUM_DOF + i] = sensor_data.plan_contacts[i];
        }
        else if (CONTACT_SENSOR_TYPE == 2)
        {
            filterd_joint_msg.velocity[NUM_DOF + i] = foot_force_sensor_readings[i];
        }
    }
    filterd_imu_pub.publish(filterd_imu_msg);
    filterd_joint_pub.publish(filterd_joint_msg);

    Eigen::Matrix<double, EKF_STATE_SIZE, 1> kf_state = kf.get_state();
    nav_msgs::Odometry filterd_pos_msg;
    filterd_pos_msg.header.stamp = ros::Time::now();
    filterd_pos_msg.child_frame_id = "world";
    filterd_pos_msg.pose.pose.position.x = kf_state[0];
    filterd_pos_msg.pose.pose.position.y = kf_state[1];
    filterd_pos_msg.pose.pose.position.z = kf_state[2];
    filterd_pos_msg.twist.twist.linear.x = kf_state[3];
    filterd_pos_msg.twist.twist.linear.y = kf_state[4];
    filterd_pos_msg.twist.twist.linear.z = kf_state[5];

    filterd_pos_pub.publish(filterd_pos_msg);

    first_sensor_received = true;
    return;
}

// 处理pose和twist消息的回调函数
void geometry_callback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg, const geometry_msgs::TwistStamped::ConstPtr& twistMsg)
{ 
    // 处理pose和twist消息的回调函数
    ROS_INFO("Received pose message:");

    // 设置path消息的头部
    truth_path.header = poseMsg->header;
    //TODO:frame_id 改为 world_truth 为什么不行？
    truth_path.header.frame_id = "world";
    // 将pose_stamped消息添加到path消息的poses字段中
    truth_path.poses.push_back(*poseMsg);
    // 发布path消息到pub_path话题
    pub_truth_path.publish(truth_path);

    // 设置odometry消息的头部
    truth_odometry.header = poseMsg->header;
    truth_odometry.child_frame_id = "world";
    // 将pose_stamped消息添加到path消息的poses字段中
    truth_odometry.pose.pose = poseMsg->pose;
    // 将twist_stamped消息添加到path消息的poses字段中
    truth_odometry.twist.twist = twistMsg->twist;
    // 发布odometry消息到pub_odometry话题
    pub_truth_odometry.publish(truth_odometry);
}