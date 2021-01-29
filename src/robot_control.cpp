#include "robot_control.h"
#include <stdio.h>
#include <sys/time.h>
#include <functional>
#include <exception>

void timespec_add_us(struct timespec *t, long us)
{
    t->tv_nsec += us*1000;
    if (t->tv_nsec > 1000000000)
    {
        t->tv_nsec = t->tv_nsec - 1000000000;// + ms*1000000;
        t->tv_sec += 1;
    }
}

//const std::string &robot_ip = "127.0.0.1";
const std::string &robot_ip = "192.168.1.23";

int RobotControl::count = 0;
Wrench RobotControl::s_calibrationMeasurements[CALIBRATION_POS::POSE_Total];

using namespace ARAL;
double distance(const std::vector<double> &a, const std::vector<double> &b)
{
    double res = 0.;
    if (a.size() != b.size()) {
        return -1;
    }
    for (int i = 0; i < (int)a.size(); i++) {
        res += (a[i] - b[i]) * (a[i] - b[i]);
    }
    return sqrt(res);
}

RobotControl::RobotControl(const std::string& model):
    IO_name_("0"),
    IO_switch_(false)
{
    aral_interface_ = CreateRLIntfacePtr(model.c_str(), /*Creat_Share_Memory*/0 | LOG_DEBUG);
//    aral_interface_->initialRLInterface();
    createParaTable();

    aubo_driver = createAuboDriver();
    if(!aubo_driver->connectToServer(robot_ip))
        exit(-1);
//    aubo_driver->waitForTerminate();
    if (aubo_driver->login("user", "111", 50))
    {
        vector<string> name;
        name.push_back("actual_q");
        name.push_back("actual_qd");
        name.push_back("actual_TCP_force");
        name.push_back("actual_TCP_speed");
        name.push_back("actual_TCP_pose");
        name.push_back("actual_current");
        name.push_back("actual_joint_voltage");
        name.push_back("actual_digital_input_bits");
        name.push_back("actual_digital_output_bits");
        aubo_driver->setRtdeOutputRecipe(0, name, 200);
        aubo_driver->startRtdeTransmission(0, std::bind(&RobotControl::getRtdeData, this, aubo_driver));
        aubo_driver->syncRtdeTransmission(0, 50);
    }
}

RobotControl::~RobotControl()
{
    //clean the force_control_ thread. first
    enable_thread_ = false;

    //clean the share memory function.  second

    aral_interface_ = nullptr;

    shmdt(shm);
    shmctl(shmid, IPC_RMID, NULL) ;
}

void RobotControl::createParaTable()
{
    para_table_["cart_mass"] =  CART_MASS;
    para_table_["cart_damp"] = CART_DAMP;
    para_table_["cart_stiffness"] = CART_STIFFNESS;
    para_table_["tool_pose"] = TOOL_POSE;
    para_table_["sensor_pose"]=  SENSOR_POSE;
    para_table_["joint_mass"] = JOINT_MASS;
    para_table_["joint_damp"] = JOINT_DAMP;
    para_table_["joint_stiffness"] = JOINT_STIFFNESS;
    para_table_["end_ft_threshold"] = END_FT_THRESHOLD;
    para_table_["base_ft_threshold"] = BASE_FT_THRESHOLD;
    para_table_["joint_ft_threshold"] = JOINT_FT_THRESHOLD;
    para_table_["end_ft_limit"] = END_FT_LIMIT;
    para_table_["base_ft_limit"] = BASE_FT_LIMIT;
    para_table_["joint_ft_limit"] = JOINT_FT_LIMIT;
    para_table_["select_vector"] = SELECT_VECTOR;
    para_table_["constraint_para"] = CONSTRAINT_PARA;
    para_table_["goal_wrench"] = GOAL_WRENCH;
}

bool RobotControl::initShareMemory()
{
    key_t key;
    char pathname[30] ;
    strcpy(pathname,"/") ;
    key = ftok(pathname,'z');
    if((shmid = shmget(key, sizeof(ForceControlData), 0666 | IPC_CREAT)) < 0)
    {
        perror("failed to create the share memory");
        exit(-1);
    }
    shm = shmat(shmid, (void*)0, 0);
    ft_share_ = (ForceControlData *)shm;
    ft_share_->enableForceControl = 0;

    return true;
}

void RobotControl::updateRobotStatus()
{
    aral_interface_->updateEndFTSensorData(sensor_data_.data());
    aral_interface_->updatJointPVAStatus(current_joints_.data(), NULL, NULL);
}

void RobotControl::updateRobotGoal()
{
    // update reference trajectory
    double q[ROBOT_DOF]/*, qd[ROBOT_DOF], qdd[ROBOT_DOF]*/;
    aral_interface_->setRefTraj(original_pose_.data(), NULL, NULL);
}

void RobotControl::getRobotOutput(std::vector<double> &joint_pos)
{
    joint_pos.resize(ROBOT_DOF);
    aral_interface_->fcGetRobotEndWrench(force_of_end_.data());
    ft_share_->errCode = aral_interface_->calJointCommand(joint_pos.data(), ft_share_->cmdJointVel, ft_share_->cmdJointAcc);
    if(ft_share_->errCode == -116)
    {
        printf("err:-116\n");
        exit(-2);
    }
}

void RobotControl::getRobotEndWrench(double * wrench)
{
    aral_interface_->fcGetRobotEndWrench(wrench);
}

void RobotControl::initalControlPara()
{
    aral_interface_->setControlType(FORCE_ADM_CONTROL);
    aral_interface_->setFeedBackOptions(0x03);     // only position and joint current feedback, no velocity and acceleration feedback
    aral_interface_->fcEnableSingularityConsistent(false);
    ft_share_->startFlag = 0;
    ft_share_->endFlag = 0;
    cart_mass_.fill(0);
    cart_damp_.fill(0);
    cart_stiffness_.fill(0);
    tool_pose_.fill(0);
    ft_sensor_pose_.fill(0);
    selection_vector_.fill(0);
    end_ft_threshold_.fill(0);
    end_ft_limit_.fill(20);  // set as temporary
    goal_wrench_.fill(0);
    enable_thread_ = true;
    setForceControlMode(1); //trajectory tracking control
}

void RobotControl::getRtdeData(AuboDriver *m_instance)
{
    if (0 == m_instance->getJointPositions().size()) {
        m_instance->syncRtdeTransmission(0, 50);
    }
    current_joints_ = m_instance->getJointPositions();
//    cout << "current joint data" << endl;
//    for (auto iter = current_joints_.begin(); iter != current_joints_.end(); iter++)
//          cout << *iter << endl;

//    m_instance->getJointVelocities();
    sensor_data_ = m_instance->getTcpForce();
//    cout << "current sensor data" << endl;
//    for (auto iter = sensor_data_.begin(); iter != sensor_data_.end(); iter++)
//          cout << *iter << endl;
    //    m_instance->getTcpPose();
    //    m_instance->getTcpSpeed();
    //    m_instance->getJointCurrents();
    //    m_instance->getJointVelocities();
    //    m_instance->getIoCurrent();
    //    m_instance->getToolDigitalInput();
    //    m_instance->getToolDigitalOutput();
    //    m_instance->getConfigurableDigitalInput();
    //    m_instance->getConfigurableDigitalOutput();
    //    m_instance->getStandardDigitalInput();
    //    m_instance->getStandardDigitalOutput();
}

int RobotControl::startForceControl()
{
    enable_thread_ = true;
    force_control_Thread_ = std::thread(&RobotControl::forceControlThread, this);
    if(force_control_Thread_.joinable())
        force_control_Thread_.join();
    return 0;
}

int RobotControl::stopForceControl()
{
    enable_thread_ = false;
    return 0;
}

/******** force control thread ********/
int RobotControl::forceControlThread()
{
    struct timeval delay;
    delay.tv_sec = 0;
    delay.tv_usec = 100 * 1000; // 100 ms
    select(0, NULL, NULL, NULL, &delay);
    std::vector<double> joint_pos;
    struct timespec next;
    clock_gettime(CLOCK_REALTIME, &next);
    while(enable_thread_)
    {
//        gettimeofday(&time1, NULL);
        updateRobotStatus();
        updateRobotGoal();
        getRobotOutput(joint_pos);
        auto builder = aubo_driver->getRtdeInputBuilder();
        builder->servoJoint(joint_pos);
        builder->send();
        timespec_add_us(&next, 5*1000); // unit: us
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);

//        gettimeofday(&time2, NULL);
//        int duration = (time2.tv_sec-time1.tv_sec)*1e6+time2.tv_usec-time1.tv_usec;
//        delay.tv_sec = 0;
//        delay.tv_usec = 5000 - duration; // 50 us
//        select(0, NULL, NULL, NULL, &delay);
//        printf("time cost:%d, absolute time: %f\n", duration, time1.tv_usec / 1000.);
    }

    return 0;
}

void RobotControl::obtainCalibrationPos(int index)
{
    //obtain the sensor data in pose 1 2,3;
    sleep(1);       //wait for robot and sensor stable,1s
    for(int i = 0; i < 6; i++)
        s_calibrationMeasurements[index][i] = sensor_data_[i];
    std::cout<<"calibrationMeasurements value: "<<s_calibrationMeasurements[index][0]<<","<<s_calibrationMeasurements[index][1]<<","
            <<s_calibrationMeasurements[index][2]<<","<<s_calibrationMeasurements[index][3]<<","<<s_calibrationMeasurements[index][4]<<","<<s_calibrationMeasurements[index][5]<<std::endl;
}

void RobotControl::enableForceControlThread(bool flag)
{
    IO_switch_ = flag;
}

void RobotControl::enableConstraints(bool flag)
{
    aral_interface_->fcEnableSingularityConsistent(flag);
}

void RobotControl::setConstrainPara(const double* value)
{
    aral_interface_->fcSetConstrainPara(value);
}

void RobotControl::setSelectionVector(const double * vec)
{
    aral_interface_->fcSetSelectVector(vec);
}

void RobotControl::setControlPeriod(const double period)
{
    aral_interface_->setControlPeriod(period);
}

void RobotControl::updateControlPara(const double& value, const int& index, const std::string& typeName)
{
    int type = para_table_[typeName];
    switch(type)
    {
        case END_FT_THRESHOLD: end_ft_threshold_[index] = value;
                               setEndFTSensorThreshold(end_ft_threshold_.data());
                               break;
        case END_FT_LIMIT: end_ft_limit_[index] = value;
                           setEndFTSensorLimit(end_ft_limit_.data());
                           break;
        case TOOL_POSE: tool_pose_[index] = value;
                        setToolPose(tool_pose_.data());
                        break;
        case CART_MASS:cart_mass_[index] = value;
                       setCartMass(cart_mass_.data());
                       break;
        case CART_DAMP: cart_damp_[index] = value;
                        setCartDamp(cart_damp_.data());
                        break;
        case CART_STIFFNESS:cart_stiffness_[index] = value;
                            setCartStiffness(cart_stiffness_.data());
                            break;
        case SENSOR_POSE:ft_sensor_pose_[index] = value;
                         setFTSensorPose(ft_sensor_pose_.data());
                         break;
        case SELECT_VECTOR:selection_vector_[index] = value;
                           setSelectionVector(selection_vector_.data());
                           break;
        case CONSTRAINT_PARA:constraint_para_[index] = value;
                             setConstrainPara(constraint_para_.data());
                             break;
        case GOAL_WRENCH:goal_wrench_[index] = value;
//                         setGoalWrench(goal_wrench_.data());
                         break;

    }
}

bool RobotControl::getHandGuidingSwitch()
{
    if(IO_name_ == "0")
        IO_switch_ = true;
    else {}
//        robotServiceReceive.robotServiceGetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDI, IO_name_, IO_switch_);
    return IO_switch_;
}

void RobotControl::setMaxTranSpeed(double vel)
{
    aral_interface_->setMaxTranSpeed(vel);
}

void RobotControl::setMaxRotSpeed(double rot)
{
    aral_interface_->setMaxRotSpeed(rot);
}

/******** Calibration function ********/
int RobotControl::moveToTargetPose(int index)
{
    double data[3][6] = {{-15*M_PI/180, 15*M_PI/180,75*M_PI/180,60*M_PI/180,80*M_PI/180,0},
                        {-36*M_PI/180,27*M_PI/180,95*M_PI/180,-27*M_PI/180,0*M_PI/180,0},
                        {-36*M_PI/180,21*M_PI/180,100*M_PI/180,-6*M_PI/180,90*M_PI/180,0}};
    std::vector<double> joints[3];
    for(int i = 0; i < 3; i++)
    {
        joints[i].resize(ROBOT_DOF);
        memcpy(joints[i].data(), data[i], sizeof(double) * ROBOT_DOF);

        for(int j = 0; j < SENSOR_DIMENSION; j++)
            calibration_poses_[index][j] = joints[i][j];
    }
    try {
        aubo_driver->moveJoint(joints[index], 0.5, 90 * M_PI / 180.);
    } catch (exception e) {
        std::cout<<e.what()<<std::endl;
    }

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        double dis =
                distance(aubo_driver->getJointPositions(), joints[index]);
        if (dis < 0.1) {
            break;
        }
        std::cout << "distance: " << dis << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    return 0;
}

void RobotControl::getCalibrationPose(int index, double* joint_angle)
{
    //move to target pose
#ifdef USE_SDK
    aubo_robot_namespace::wayPoint_S wayPoint;
    int ret = robot_service_.robotServiceGetCurrentWaypointInfo(wayPoint);
    for(int i = 0; i < ROBOT_DOF; i++)
        calibration_poses_[index](i) = wayPoint.jointpos[i];
    memcpy(joint_angle, wayPoint.jointpos, sizeof(double)*ROBOT_DOF);
#endif
}

int RobotControl::calibrateFTSensor(FtSensorCalibrationResult &result)
{
    JointArray joints[FT_SENSOR_CALIB__NUM];
    Wrench measurement[FT_SENSOR_CALIB__NUM];

//    double data1[] = {-15*M_PI/180,15*M_PI/180,75*M_PI/180,60*M_PI/180,80*M_PI/180,0};
//    double data2[] = {-36*M_PI/180,27*M_PI/180,95*M_PI/180,-27*M_PI/180,0*M_PI/180,0};
//    double data3[] = {-36*M_PI/180,21*M_PI/180,100*M_PI/180,-6*M_PI/180,90*M_PI/180,0};

//    joints[0].setData(6, data1);
//    joints[1].setData(6, data2);
//    joints[2].setData(6, data3);


//    double dd1[] = {-9.8839,-1.82701,-0.12016,0.08255,-0.190,0.1487};
//    double dd2[] = {-9.83771,-1.8097,-0.0406,0.0828,-0.1955,0.1484};
//    double dd3[] = {0.5495,9.7566,-0.2766,0.2374,0.0302,0.1487};


//    measurement[0].setData(6, dd1);
//    measurement[1].setData(6, dd2);
//    measurement[2].setData(6, dd3);

//    FtSensorCalibrationResult result;
//    int ret = aral_interface_->calibToolAndSensor(joints, measurement, result);

    for (int i = 0; i < 3; i++)
    {
        std::cout<<s_calibrationMeasurements[i][0]<<", "<<s_calibrationMeasurements[i][1]<<", "<<s_calibrationMeasurements[i][2]<<", "
                 <<s_calibrationMeasurements[i][3]<<", "<<s_calibrationMeasurements[i][4]<<", "<<s_calibrationMeasurements[i][5]<<", "<<std::endl;
    }
    return aral_interface_->calibToolAndSensor(calibration_poses_, s_calibrationMeasurements, result);
}

/******** Control Parameter function ********/

void RobotControl::setToolDynamicsFromFTSensor(const RigidBodyInertia &I)
{
    aral_interface_->setToolInertialFromFTSensor(I.mass, I.com.data(), I.inertial);
}

/******************** Admittance Control ********************/
void RobotControl::enableAdmittanceControl()
{
    aral_interface_->fcEnable(true);
//    ft_share_->trackEnable = true;
    ft_share_->enableForceControl = 1;
}

void RobotControl::disableAdmittanceControl()
{
    aral_interface_->fcEnable(false);
//    ft_share_->trackEnable = false;
    ft_share_->enableForceControl = 0;
}

void RobotControl::updateAdmittancePIDPara(double value, int index)
{
//    ft_share_->pid_motion[index] = value;
}

void RobotControl::setDragMode(unsigned int type)
{
    aral_interface_->fcSetDragMode((DragMode)type);
}

void RobotControl::setForceControlMode(unsigned int mode)
{
    aral_interface_->fcSetMode(mode);
}

void RobotControl::setThreadMode(unsigned int mode)
{
    aral_interface_->fcSetCalThread(mode);
}

void RobotControl::setCalMethod(unsigned int type)
{
    aral_interface_->fcSetCalMethod(type);
}

void RobotControl::setControlSpace(unsigned int value)
{
    aral_interface_->fcSetSpace(value);
}

void RobotControl::setFilter1(const double value)
{
    aral_interface_->fcSetSensorFilter(value);
}

void RobotControl::setFilter2(const double value)
{
    aral_interface_->fcSetControlFilter(value);
}

void RobotControl::setEndFTSensorThreshold(double data[SENSOR_DIMENSION])
{
    aral_interface_->fcSetEndFTSensorThreshold(data);
}

void RobotControl::setEndFTSensorLimit(double data[SENSOR_DIMENSION])
{
    aral_interface_->fcSetEndFTSensorLimit(data);
}

int RobotControl::setCartStiffness(double data[CARTESIAN_FREEDOM])
{
    int ret = 0;
    return aral_interface_->fcSetCartStiffness(data);
}

int RobotControl::setCartDamp(double data[CARTESIAN_FREEDOM])
{
    return aral_interface_->fcSetCartDamp(data);
}

int RobotControl::setCartMass(double data[CARTESIAN_FREEDOM])
{
    return aral_interface_->fcSetCartMass(data);
}

void RobotControl::setOverEstimatedDis(const double dis)
{
    aral_interface_->fcSetOverEstimatedDis(dis);
}

void RobotControl::setGoalWrench(const double* wrench)
{
    aral_interface_->fcSetGoalWrench(wrench);
}


void RobotControl::setToolPose(double data[SENSOR_DIMENSION])
{
    aral_interface_->setToolPose(data, POS_RPY);
}

void RobotControl::setFTSensorPose(double data[SENSOR_DIMENSION])
{
    aral_interface_->setEndSensorPose(data, POS_RPY);
}




