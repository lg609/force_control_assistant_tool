#include "robot_control.h"
#include <stdio.h>
#include <sys/time.h>

int RobotControl::count = 0;

float R[SENSOR_DIMENSION] = {0.00241, 0.0023, 0.0077, 0.0090, 0.0020, 0.00241};
float Q[SENSOR_DIMENSION] = {1e-5,1e-5,1e-5,1e-7,1e-7,1e-7};

float P[] = {1,1,1,1,1,1};
float pp[6] = {0};
float K[6] = {0};


RobotControl::RobotControl(const std::string& model):
    IO_name_("0"),
    IO_switch_(false)
{
    aral_interface_ = new RLIntface(model);

    if(initShareMemory())
    {
        initalControlPara();
        force_control_ = new std::thread(std::bind(&RobotControl::startForceControl, this));
    }
    else
        std::cout<<"falied to create the shared memory communication!"<<std::endl;
}

RobotControl::~RobotControl()
{
    //clean the force_control_ thread. first
    enable_thread_ = false;
    if(force_control_!= NULL && force_control_->joinable())
        force_control_->join();
    if(force_control_ != NULL)
        delete force_control_;

    //clean the share memory function.  second

    delete aral_interface_;
    aral_interface_ = NULL;

    shmdt(shm);
    shmctl(shmid, IPC_RMID, NULL) ;
}

void RobotControl::createParaTable()
{
    para_table_.insert(std::pair<std::string, int>("cart_mass", CART_MASS));
    para_table_.insert(std::pair<std::string, int>("cart_damp", CART_DAMP));
    para_table_.insert(std::pair<std::string, int>("cart_stiffness", CART_STIFFNESS));
    para_table_.insert(std::pair<std::string, int>("tool_pose", TOOL_POSE));
    para_table_.insert(std::pair<std::string, int>("sensor_pose", SENSOR_POSE));
    para_table_.insert(std::pair<std::string, int>("joint_mass", JOINT_MASS));
    para_table_.insert(std::pair<std::string, int>("joint_damp", JOINT_DAMP));
    para_table_.insert(std::pair<std::string, int>("joint_stiffness", JOINT_STIFFNESS));
    para_table_.insert(std::pair<std::string, int>("end_ft_threshold", END_FT_THRESHOLD));
    para_table_.insert(std::pair<std::string, int>("base_ft_threshold", BASE_FT_THRESHOLD));
    para_table_.insert(std::pair<std::string, int>("joint_ft_threshold", JOINT_FT_THRESHOLD));
    para_table_.insert(std::pair<std::string, int>("end_ft_limit", END_FT_LIMIT));
    para_table_.insert(std::pair<std::string, int>("base_ft_limit", BASE_FT_LIMIT));
    para_table_.insert(std::pair<std::string, int>("joint_ft_limit", JOINT_FT_LIMIT));
    para_table_.insert(std::pair<std::string, int>("select_vector", SELECT_VECTOR));
    para_table_.insert(std::pair<std::string, int>("constraint_para", CONSTRAINT_PARA));
    para_table_.insert(std::pair<std::string, int>("goal_wrench", GOAL_WRENCH));
}

bool RobotControl::initShareMemory()
{
    key_t key;
    char pathname[30] ;
    strcpy(pathname,"/") ;
    key = ftok(pathname,'z');
    if((shmid = shmget(key, sizeof(ForceControlData), 0666 | IPC_CREAT)) < 0)
    {
        perror(pathname);
        return false;
    }
    shm = shmat(shmid, (void*)0, 0);
    ft_share_ = (ForceControlData *)shm;

    return true;
}

void RobotControl::updateRobotStatus()
{
    double q[ROBOT_DOF], qd[ROBOT_DOF], qdd[ROBOT_DOF];
    memcpy(q, ft_share_->curJointPos, sizeof(double) * ROBOT_DOF);
    aral_interface_->updatJointPVAStatus(q, qd, qdd);
    aral_interface_->updateEndFTSensorData(FTSensorDataProcess::getSensorData().data());
}

void RobotControl::updateRobotGoal()
{
    // update reference trajectory
}

void RobotControl::getRobotOutput()
{
    aral_interface_->getJointCommand(ft_share_->curJointPos, ft_share_->curJointVel, ft_share_->curJointAcc);
    aral_interface_->getRobotEndWrench(force_of_end_.data());
}

void RobotControl::getRobotEndWrench(double * wrench)
{
    aral_interface_->getRobotEndWrench(wrench);
}

void RobotControl::initalControlPara()
{
    aral_interface_->setControlType(FORCE_ADM_CONTROL);
    aral_interface_->setFeedBackOptions(0x03);     // only position and joint current feedback, no velocity and acceleration feedback
    aral_interface_->enableSingularityConsistent(false);
    ft_share_->trackEnable = 1;
    cart_mass_.setToZero();
    cart_damp_.setToZero();
    cart_stiffness_.setToZero();
    tool_pose_.setToZero();
    ft_sensor_pose_.setToZero();
    selection_vector_.setConstant(1);
    end_ft_threshold_.setToZero();
    end_ft_limit_.setConstant(20);  // set as temporary
    goal_wrench_.setToZero();
    enable_thread_ = true;
}

/******** force control function ********/
void RobotControl::startForceControl()
{
    struct timeval delay;
    delay.tv_sec = 0;
    delay.tv_usec = 100 * 1000; // 100 ms

    while(enable_thread_)
    {
        if(ft_share_->trackEnable == 1 && IO_switch_)
        {
            ft_share_->trackEnable = 0;
            updateRobotStatus();
            updateRobotGoal();
            getRobotOutput();
        }
        else
        {
            delay.tv_sec = 0;
            delay.tv_usec = 50; // 50 us
            select(0, NULL, NULL, NULL, &delay);
//            gettimeofday(&time1, NULL);
//            printf("count:%d, sec: %d, usec: %d \n", count++, time1.tv_sec, time1.tv_usec);
        }
    }
}

void RobotControl::enableConstraints(bool flag)
{
    aral_interface_->enableSingularityConsistent(flag);
}

void RobotControl::setConstrainPara(const double* value)
{
    aral_interface_->setConstrainPara(value);
}

void RobotControl::setSelectionVector(const double * vec)
{
    aral_interface_->setSelectMatrix(vec);
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
                         setGoalWrench(goal_wrench_.data());
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
int RobotControl::moveToTargetPose(int /*index*/)
{
    //move to target pose
//    robotServiceSend.robotServiceInitGlobalMoveProfile();
//    aubo_robot_namespace::JointVelcAccParam jointMaxAcc, jointMaxVelc;
//    memset(jointMaxAcc.jointPara, joint_max_acc_, sizeof(double)*ARM_DOF);
//    memset(jointMaxVelc.jointPara, joint_max_velc_, sizeof(double)*ARM_DOF);
//    robotServiceSend.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
//    robotServiceSend.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

//    double joint[ROBOT_DOF];
//    s_pose_calibration[index].toDoubleArray(joint);
//    return robotServiceSend.robotServiceJointMove(joint, true);  // Start to move to the selected pose,if success, return 0;
    return 0;
}

void RobotControl::getCalibrationPose(int /*index*/, double* /*joint_angle[]*/)
{
    //move to target pose
//    aubo_robot_namespace::wayPoint_S wayPoint;
//    robotServiceSend.robotServiceGetCurrentWaypointInfo(wayPoint);
//    for(int i = 0; i < ARM_DOF; i++)
//        s_pose_calibration[index](i) = wayPoint.jointpos[i];
//    memcpy(joint_angle, wayPoint.jointpos, sizeof(double)*ARM_DOF);
}

int RobotControl::calibrateFTSensor(FtSensorCalibrationResult &result)
{
    return aral_interface_->calibToolAndSensor(calibration_poses_, FTSensorDataProcess::s_calibrationMeasurements, result);
}

/******** Control Parameter function ********/

void RobotControl::setToolDynamicsFromFTSensor(const RigidBodyInertia &I)
{
    aral_interface_->setToolInertialFromFTSensor(I.mass, I.com.data(), I.inertial);
}

/******************** Admittance Control ********************/
void RobotControl::enableAdmittanceControl()
{
//    ft_share_->trackEnable = true;
    ft_share_->mode = 1;
}

void RobotControl::disableAdmittanceControl()
{
//    ft_share_->trackEnable = false;
    ft_share_->mode = 0;
}

void RobotControl::updateAdmittancePIDPara(double value, int index)
{
//    ft_share_->pid_motion[index] = value;
}

void RobotControl::setDragMode(unsigned int type)
{
    aral_interface_->setDragMode(type);
}

void RobotControl::setForceControlMode(unsigned int mode)
{
    aral_interface_->setForceControlMode(mode);
}

void RobotControl::setThreadMode(unsigned int mode)
{
    aral_interface_->setCalThread(mode);
}

void RobotControl::setCalMethod(unsigned int type)
{
    aral_interface_->setCalMethod(type);
}

void RobotControl::setControlSpace(unsigned int value)
{
    aral_interface_->setForceControlSpace(value);
}

void RobotControl::setFilter1(const double value)
{
    aral_interface_->setSensorFilter1(value);
}

void RobotControl::setFilter2(const double value)
{
    aral_interface_->setSensorFilter2(value);
}

void RobotControl::setEndFTSensorThreshold(double data[SENSOR_DIMENSION])
{
    aral_interface_->setEndFTSensorThreshold(data);
}

void RobotControl::setEndFTSensorLimit(double data[SENSOR_DIMENSION])
{
    aral_interface_->setEndFTSensorLimit(data);
}

int RobotControl::setCartStiffness(double data[CARTESIAN_FREEDOM])
{
    return aral_interface_->setCartStiffness(data);
}

int RobotControl::setCartDamp(double data[CARTESIAN_FREEDOM])
{
    return aral_interface_->setCartDamp(data);
}

int RobotControl::setCartMass(double data[CARTESIAN_FREEDOM])
{
    return aral_interface_->setCartMass(data);
}

void RobotControl::setOverEstimatedDis(const double dis)
{
    aral_interface_->setOverEstimatedDis(dis);
}

void RobotControl::setGoalWrench(const double* wrench)
{
    aral_interface_->setGoalWrench(wrench);
}


void RobotControl::setToolPose(double data[SENSOR_DIMENSION])
{
    aral_interface_->setToolPose(data, POS_RPY);
}

void RobotControl::setFTSensorPose(double data[SENSOR_DIMENSION])
{
    aral_interface_->setEndSensorPose(data, POS_RPY);
}




