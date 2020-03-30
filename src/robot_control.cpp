#include "robot_control.h"
#include <stdio.h>

int RobotControl::s_control_period = 5;         //ms      //unit s
bool RobotControl::s_start_handguiding = false;
bool RobotControl::s_thread_handguiding = true;
double RobotControl::s_tool_pose[CARTESIAN_FREEDOM] = {0};           //TOOL POSE relative to the robot end
//RigidBodyInertia RobotControl::tool_dynamics;
double RobotControl::s_threshold[6];
double RobotControl::s_limit[6];
double RobotControl::force_of_end_[6];
double RobotControl::s_pose_calibration[3][6];

int RobotControl::s_dragMode = 0;
int RobotControl::s_calculateMethod = 0;
int RobotControl::s_controlModel = 0;
int RobotControl::s_controlSpace = 0;

int RobotControl::s_bufferSizeLimit = 42;
int RobotControl::s_filter1 = 0;
int RobotControl::s_filter2 = 0;
double RobotControl::s_mass[SENSOR_DIMENSION] = {0};
double RobotControl::s_damp[SENSOR_DIMENSION] = {0};
double RobotControl::s_stiffness[SENSOR_DIMENSION] = {0};


float R[SENSOR_DIMENSION] = {0.00241, 0.0023, 0.0077, 0.0090, 0.0020, 0.00241};
float Q[SENSOR_DIMENSION] = {1e-5,1e-5,1e-5,1e-7,1e-7,1e-7};

float P[] = {1,1,1,1,1,1};
float pp[6] = {0};
float K[6] = {0};

//#define ARM_DOF 6


RobotControl::RobotControl(const std::string& model):
    enable_constraints_(false),
    IO_name_("0"),
    IO_switch_(0.)/*,
    last_send_joints_(CARTESIAN_FREEDOM),
    current_way_point_(CARTESIAN_FREEDOM)*/
{
    aral_interface_ = new RLIntface(model);
    if(initShareMemory())
    {
        s_thread_handguiding = true;
        force_control_ = new std::thread(std::bind(&RobotControl::startForceControl, this));
    }
    else
        std::cout<<"falied to create the shared memory communication!"<<std::endl;

//    s_pose_calibration[0] = s_pose_calibration[1] = s_pose_calibration[2] = JntArray(6);
//    last_send_joints_.q = last_send_joints_.qdot = last_send_joints_.qdotdot = JntArray(6);
//    current_way_point_.q = current_way_point_.qdot = current_way_point_.qdotdot = JntArray(6);
//    robot_kine_ = new Kinematics(robot_model);


//    paraType_.insert(std::pair<std::string, int>("sensitivity", MASS));
//    paraType_.insert(std::pair<std::string, int>("damp", DAMP));
//    paraType_.insert(std::pair<std::string, int>("stiffness", STIFFNESS));
//    paraType_.insert(std::pair<std::string, int>("threshold", THRESHOLD));
//    paraType_.insert(std::pair<std::string, int>("limit", LIMIT));
//    paraType_.insert(std::pair<std::string, int>("pos", POS));
}

RobotControl::~RobotControl()
{
    //clean the share memory function.
    shmctl(shmid, IPC_RMID, NULL) ;
    shmdt(shm);
    if(ft_share_ != NULL)
    {
        delete ft_share_;
        ft_share_ = NULL;
    }

    //clean the force_control_ thread.
    s_thread_handguiding = false;
    usleep(10*1000);
    if(force_control_!= NULL && force_control_->joinable())
        force_control_->join();
    if(force_control_ != NULL)
        delete force_control_;
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
    JointArray q, qd, qdd;
    memcpy(q.data, ft_share_->curJointPos, sizeof(double) * ROBOT_DOF);
    aral_interface_->updatJointStatus(q, qd, qdd);
    aral_interface_->updateEndFTSensorData(FTSensorDataProcess::getSensorData().data);
}

void RobotControl::updateRobotGoal()
{
    // update reference trajectory


}

void RobotControl::initalControlPara()
{
    aral_interface_->setFeedBackOptions(0x03);     // only position and joint current feedback, no velocity and acceleration feedback
    aral_interface_->setControlType(FORCE_ADM_CONTROL);
    ft_share_->trackEnable = 1;
}

/******** force control function ********/
void RobotControl::startForceControl()
{
    struct timeval delay;
    delay.tv_sec = 0;
    delay.tv_usec = 10 * 1000; // 20 ms

    initalControlPara();
    while(s_thread_handguiding)
    {
        if(ft_share_->trackEnable == 1)
        {
            ft_share_->trackEnable = 0;
            updateRobotStatus();
            updateRobotGoal();
        }
        else
        {
            select(0, NULL, NULL, NULL, &delay);
        }

    }

}

void RobotControl::updateControlPara(const double& value, const int& index, const std::string& typeName)
{

//    int type = paraType_[typeName];
//    switch(type)
//    {
//    case THRESHOLD: s_threshold[index] = value;break;
//    case LIMIT: s_limit[index] = value;break;
//    case POS: s_tool_pose[index] = value;break;
//    case MASS:s_mass[index] = value; ft_share_->aMass[index] = value; break;
//    case DAMP: s_damp[index] = value; ft_share_->aDamp[index] = value; break;
//    case STIFFNESS: s_stiffness[index] = value; ft_share_->aStiffness[index] = value; break;
//    case SENSITIVITY:ft_share_->sensitivity[index] = value;break;
//    }
}



double RobotControl::getHandGuidingSwitch()
{
    if(IO_name_ == "0")
        return 1;
    else
//        robotServiceReceive.robotServiceGetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDI, IO_name_, IO_switch_);
    return IO_switch_;
}


/******** Calibration function ********/
int RobotControl::moveToTargetPose(int index)
{
    //move to target pose
//    robotServiceSend.robotServiceInitGlobalMoveProfile();
//    aubo_robot_namespace::JointVelcAccParam jointMaxAcc, jointMaxVelc;
//    memset(jointMaxAcc.jointPara, joint_max_acc_, sizeof(double)*ARM_DOF);
//    memset(jointMaxVelc.jointPara, joint_max_velc_, sizeof(double)*ARM_DOF);
//    robotServiceSend.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
//    robotServiceSend.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    double joint[ROBOT_DOF];
//    s_pose_calibration[index].toDoubleArray(joint);
//    return robotServiceSend.robotServiceJointMove(joint, true);  // Start to move to the selected pose,if success, return 0;
}
// subtract gravity of tool;


void RobotControl::getCalibrationPose(int index, double joint_angle[])
{
    //move to target pose
//    aubo_robot_namespace::wayPoint_S wayPoint;
//    robotServiceSend.robotServiceGetCurrentWaypointInfo(wayPoint);
//    for(int i = 0; i < ARM_DOF; i++)
//        s_pose_calibration[index](i) = wayPoint.jointpos[i];
//    memcpy(joint_angle, wayPoint.jointpos, sizeof(double)*ARM_DOF);
}


/******** Control Parameter function ********/



bool RobotControl::obtainCenterofMass(double result[])
{
//    robot_kine_->calibrateToolAndSensor(s_pose_calibration, FTSensorDataProcess::s_calibrationMeasurements, result);
}


void RobotControl::setToolProperty()
{
//    robot_kine_->setToolProperty(Frame(Rotation::RPY(s_tool_pose[3],s_tool_pose[4],s_tool_pose[5]), Vector(s_tool_pose[0], s_tool_pose[1], s_tool_pose[2])));
}


/******************** Admittance Control ********************/
void RobotControl::enableAdmittanceControl()
{
    ft_share_->trackEnable = true;
    ft_share_->mode = 1;
}

void RobotControl::disableAdmittanceControl()
{
    ft_share_->trackEnable = false;
    ft_share_->mode = 0;
}

//void RobotControl::setAdmittanceControlFT(double value, CONTROL_AXIS axis)
//{
//    ft_share_->goalWrench[axis] = value;
//}

void RobotControl::updateAdmittancePIDPara(double value, int index)
{
    ft_share_->pid_motion[index] = value;
}

void RobotControl::getMaxSigma(double sigmaValue[])
{
//    for(int i = 0; i < SENSOR_DIMENSION; i++)
//        sigmaValue[i] = s_damp[i]/(s_mass[i]/CONTROL_PERIOD + s_damp[i]);
}

