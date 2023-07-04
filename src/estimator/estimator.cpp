
#include "estimator.h"

Estimator::Estimator()
{
    // ROS_INFO("init begins");
    // initThreadFlag = false;
    // clearState();
}

Estimator::~Estimator()
{
    // if (MULTIPLE_THREAD)
    // {
    //     processThread.join();
    //     printf("join thread \n");
    // }
}


// in MULTIPLE_THREAD mode, this function will be called periodically
void Estimator::processMeasurements()
{
    while (true)
    {
        //        printf("process measurments\n");
        // pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> feature;

        // Vector3d 3维向量 线性加速度和角速度
        std::vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
 
        // vector of time index, Vector12d for ang, vel Vector4d for footForce
        // 12维向量 关节角度和速度
        vector<pair<double, Vector_dof>> jointAngVector, jointVelVector;
        // 四维向量
        vector<pair<double, Vector_leg>> contactFlagVector;

        // if (!featureBuf.empty())
        if(1)
        {
            // feature = featureBuf.front();
            // curTime = feature.first + td;
            // wait until IMU is available
            // while (1)
            // {
            //     if ((!USE_IMU || IMUAvailable(feature.first + td)))
            //         break;
            //     else
            //     {
            //         printf("wait for imu and leg ... \n");
            //         if (!MULTIPLE_THREAD)
            //             return;
            //         std::chrono::milliseconds dura(5);
            //         std::this_thread::sleep_for(dura);
            //     }
            // }
            mBuf.lock();
            // extract imu data from accbuf, gyrbuf
            // save imu data into accVector and gyrVector

            // TODO: get leg info and IMU together
            if (USE_LEG && USE_IMU)
            {
                getIMUAndLegInterval(prevTime, curTime, accVector, gyrVector, jointAngVector, jointVelVector, contactFlagVector);
            }

            // remove feature buf
            // featureBuf.pop();
            mBuf.unlock();

            Vector3d tmpPs;
            Vector3d tmpPs2;
            Vector3d tmpVs;
            Matrix3d tmpRs;

            if (USE_LEG && USE_IMU)
            {
                // average acc to get initial Rs[0]
                if (!initFirstPoseFlag)
                    initFirstIMUPose(accVector);
                for (size_t i = 0; i < accVector.size(); i++)
                {
                    double dt;
                    if (i == 0)
                        dt = accVector[i].first - prevTime;
                    else if (i == accVector.size() - 1)
                        dt = curTime - accVector[i - 1].first;
                    else
                        dt = accVector[i].first - accVector[i - 1].first;

                    processIMULeg(accVector[i].first, dt, accVector[i].second, gyrVector[i].second,
                                  jointAngVector[i].second, jointVelVector[i].second, contactFlagVector[i].second);
                }

                /* many debug print goes here */
            }

            mProcess.lock();
            // processImage(feature.second, feature.first);

            prevTime = curTime;

            // printStatistics(*this, 0);

            std_msgs::Header header;
            header.frame_id = "world";
            header.stamp = ros::Time::now();

            // pubOdometry(*this, header);
            // pubKeyPoses(*this, header);
            // pubCameraPose(*this, header);
            // pubPointCloud(*this, header);
            // pubKeyframe(*this);
            // pubTF(*this, header);
            mProcess.unlock();
        }

        if (!MULTIPLE_THREAD)
            break;

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void Estimator::setParameter()
{
    mProcess.lock();
    // for (int i = 0; i < NUM_OF_CAM; i++)
    // {
    //     tic[i] = TIC[i];
    //     ric[i] = RIC[i];
    //     cout << " exitrinsic cam " << i << endl
    //          << ric[i] << endl
    //          << tic[i].transpose() << endl;
    // }
    // f_manager.setRic(ric);
    // ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    // ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    // ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    // td = TD;
    // g = G;
    // cout << "set g " << g.transpose() << endl;
    // featureTracker.readIntrinsicParameter(CAM_NAMES);

    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
    if (MULTIPLE_THREAD && !initThreadFlag)
    {
        initThreadFlag = true;
        processThread = std::thread(&Estimator::processMeasurements, this);
    }
    // set leg kinematics related parameters
    // body_to_a1_body, in case the IMU and the robot body frame has nontrival transformation
    p_br = Eigen::Vector3d(0, 0.0, 0);
    R_br = Eigen::Matrix3d::Identity();
    /*** leg order: 0-FL  1-FR  2-RL  3-RR ***/
    // TODO:注释待定
    /* 每条腿在机器人主体坐标系中X轴方向的偏移量 */
    leg_offset_x[0] = 0.1805;
    leg_offset_x[1] = 0.1805;
    leg_offset_x[2] = -0.1805;
    leg_offset_x[3] = -0.1805;
    /* 每条腿在机器人主体坐标系中Y轴方向的偏移量 */
    leg_offset_y[0] = 0.047;
    leg_offset_y[1] = -0.047;
    leg_offset_y[2] = 0.047;
    leg_offset_y[3] = -0.047;
    /* 每条腿在电机位置上的偏移量 */
    motor_offset[0] = 0.0838;
    motor_offset[1] = -0.0838;
    motor_offset[2] = 0.0838;
    motor_offset[3] = -0.0838;
    /* 每条腿的上腿长度 */
    upper_leg_length[0] = upper_leg_length[1] = upper_leg_length[2] = upper_leg_length[3] = 0.21;
    /* 每条腿的下腿长度 */
    lower_leg_length[0] = lower_leg_length[1] = lower_leg_length[2] = lower_leg_length[3] = VILO_LOWER_LEG_LENGTH;

    for (int i = 0; i < NUM_OF_LEG; i++)
    {
        // 创建 RHO_FIX_SIZE 维向量
        Eigen::VectorXd rho_fix(RHO_FIX_SIZE);
        // 使用初始化列表设置向量的值
        rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], upper_leg_length[i];
        // 将 rho_fix 向量添加到容器末尾
        rho_fix_list.push_back(rho_fix);
    }

    // TODO:创建 全1矩阵，作用 ???
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rho1[i] = VILO_LOWER_LEG_LENGTH * Eigen::Matrix<double, RHO_OPT_SIZE, 1>::Ones();
        Rho2[i] = VILO_LOWER_LEG_LENGTH * Eigen::Matrix<double, RHO_OPT_SIZE, 1>::Ones();
        Rho3[i] = VILO_LOWER_LEG_LENGTH * Eigen::Matrix<double, RHO_OPT_SIZE, 1>::Ones();
        Rho4[i] = VILO_LOWER_LEG_LENGTH * Eigen::Matrix<double, RHO_OPT_SIZE, 1>::Ones();
    }

    mProcess.unlock();
}


// save imu data into buffer
// use fastPredict to output an odometry that is the same frequency as IMU
void Estimator::inputIMU(double t, const Eigen::Vector3d &linearAcceleration, const Eigen::Vector3d &angularVelocity)
{
    mBuf.lock();

    accBuf.push(std::make_pair(t, linearAcceleration));
    gyrBuf.push(std::make_pair(t, angularVelocity));
    // printf("input imu with time %f \n", t);
    mBuf.unlock();

    // this logic boosts output to the same rate as IMU
    // need to deal this with leg odometry
    if (solver_flag == NON_LINEAR)
    {
        mPropagate.lock();
        fastPredictIMU(t, linearAcceleration, angularVelocity);
        pubLatestOdometry(latest_P, latest_Q, latest_V, t);
        mPropagate.unlock();
    }
}

void Estimator::inputLeg(double t, const Eigen::Ref<const Vector_dof> &jointAngles,
                         const Eigen::Ref<const Vector_dof> &jointVels, const Eigen::Ref<const Vector_leg> &contact_flags)
{
    mBuf.lock();
    leg_msg_counter++;

    //    if (leg_msg_counter % 2 == 0) {
    legAngBufList.push(make_pair(t, jointAngles));
    legAngVelBufList.push(make_pair(t, jointVels));
    contactFlagBufList.push(make_pair(t, contact_flags));
    //    }
    //    std::cout << "input foot force" << footForces.transpose() << std::endl;
    //    printf("input leg joint state and foot force with time %f \n", t);
    mBuf.unlock();
}

void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
{
    double dt = t - latest_time;
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
}

void Estimator::processIMU(double t, double dt, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        // if(solver_flag != NON_LINEAR)
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Estimator::processIMULeg(double t, double dt,
                              const Vector3d &linear_acceleration, const Vector3d &angular_velocity,
                              const Ref<const Vector_dof> &joint_angle, const Ref<const Vector_dof> &joint_velocity,
                              const Ref<const Vector_leg> &foot_contact)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
        phi_0 = joint_angle;
        dphi_0 = joint_velocity;
        c_0 = foot_contact;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }

    if (!il_pre_integrations[frame_count])
    {
        Vector_rho tmp;
        tmp.segment<RHO_OPT_SIZE>(0 * RHO_OPT_SIZE) = Rho1[frame_count];
        tmp.segment<RHO_OPT_SIZE>(1 * RHO_OPT_SIZE) = Rho2[frame_count];
        tmp.segment<RHO_OPT_SIZE>(2 * RHO_OPT_SIZE) = Rho3[frame_count];
        tmp.segment<RHO_OPT_SIZE>(3 * RHO_OPT_SIZE) = Rho4[frame_count];
        il_pre_integrations[frame_count] = new IMULegIntegrationBase{acc_0, gyr_0, phi_0, dphi_0, c_0,
                                                                     Bas[frame_count], Bgs[frame_count], tmp, rho_fix_list, p_br, R_br};
    }

    if (frame_count != 0)
    {
        //        std::cout << "input leg data: \n" << joint_angle.transpose() << std::endl;
        //        std::cout << joint_velocity.transpose() << std::endl;
        //        std::cout << foot_contact.transpose() << std::endl;
        //        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        il_pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity, joint_angle, joint_velocity, foot_contact);
        // if(solver_flag != NON_LINEAR)
        //        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);
        tmp_il_pre_integration->push_back(dt, linear_acceleration, angular_velocity, joint_angle, joint_velocity, foot_contact);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);
        joint_angle_buf[frame_count].emplace_back(joint_angle);
        joint_velocity_buf[frame_count].emplace_back(joint_velocity);
        foot_contact_buf[frame_count].emplace_back(foot_contact);

        int j = frame_count;
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
    phi_0 = joint_angle;
    dphi_0 = joint_velocity;
    c_0 = foot_contact;
}

void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    printf("init first imu pose\n");
    initFirstPoseFlag = true;
    // return;
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for (size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
    Matrix3d R0 = Utility::g2R(averAcc);
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Rs[0] = R0;
    cout << "init R0 " << endl
         << Rs[0] << endl;
    // Vs[0] = Vector3d(5, 0, 0);
}

// t0: the time of the previous image frame
// t1: the time of the current image frame
bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector,
                               vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    if (accBuf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    //    printf("get imu from %f %f\n", t0, t1);
    //    printf("imu front time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    // must have more IMU than image
    if (t1 <= accBuf.back().first)
    {
        while (accBuf.front().first <= t0)
        {
            accBuf.pop();
            gyrBuf.pop();
        }
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
}

// assume leg measurement aligns with IMU measurement very well
bool Estimator::getIMUAndLegInterval(double t0, double t1,
                                     vector<pair<double, Eigen::Vector3d>> &accVector,
                                     vector<pair<double, Eigen::Vector3d>> &gyrVector,
                                     vector<pair<double, Vector_dof>> &jointAngVector,
                                     vector<pair<double, Vector_dof>> &jointVelVector,
                                     vector<pair<double, Vector_leg>> &contactFlagVector)
{
    if (accBuf.empty())
    {
        printf("not receive imu nor leg\n");
        return false;
    }
    // must have more IMU than image
    if (t1 <= accBuf.back().first)
    {
        while (accBuf.front().first <= t0)
        {
            accBuf.pop();
            gyrBuf.pop();
            legAngBufList.pop();
            legAngVelBufList.pop();
            contactFlagBufList.pop();
        }
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
            jointAngVector.push_back(legAngBufList.front());
            legAngBufList.pop();
            jointVelVector.push_back(legAngVelBufList.front());
            legAngVelBufList.pop();
            contactFlagVector.push_back(contactFlagBufList.front());
            contactFlagBufList.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
        jointAngVector.push_back(legAngBufList.front());
        jointVelVector.push_back(legAngVelBufList.front());
        contactFlagVector.push_back(contactFlagBufList.front());
    }
    else
    {
        printf("wait for imu and leg\n");
        return false;
    }
    return true;
}

// internal
bool Estimator::IMUAvailable(double t)
{
    if (!accBuf.empty() && t <= accBuf.back().first)
        return true;
    else
        return false;
}