#include "robotcontrol.h"

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

#define ARM_DOF 6

RobotControl::RobotControl():tcp2CanMode_(false),enable_constraints_(false),IO_name_("0"),IO_switch_(0.),server_host_ip_("127.0.0.1")
  /*,last_send_joints_(CARTESIAN_FREEDOM),current_way_point_(CARTESIAN_FREEDOM)*/
{
//    s_pose_calibration[0] = s_pose_calibration[1] = s_pose_calibration[2] = JntArray(6);
//    last_send_joints_.q = last_send_joints_.qdot = last_send_joints_.qdotdot = JntArray(6);
//    current_way_point_.q = current_way_point_.qdot = current_way_point_.qdotdot = JntArray(6);
//    robot_kine_ = new Kinematics(robot_model);
    key_t key;
    char pathname[30] ;
    strcpy(pathname,"/") ;
    key = ftok(pathname,'z');
    shmid = shmget(key, sizeof(FTSensorData), 0666 | IPC_CREAT);
    shm = shmat(shmid, (void*)0, 0);
    ft_share_ = (FTSensorData *)shm;

//    paraType_.insert(std::pair<std::string, int>("sensitivity", MASS));
//    paraType_.insert(std::pair<std::string, int>("damp", DAMP));
//    paraType_.insert(std::pair<std::string, int>("stiffness", STIFFNESS));
//    paraType_.insert(std::pair<std::string, int>("threshold", THRESHOLD));
//    paraType_.insert(std::pair<std::string, int>("limit", LIMIT));
//    paraType_.insert(std::pair<std::string, int>("pos", POS));

//    initRobotService();
}

RobotControl::~RobotControl()
{
    shmctl(shmid, IPC_RMID, NULL) ;
    shmdt(shm);
    if(ft_share_ != NULL)
    {
        delete ft_share_;
        ft_share_ = NULL;
    }
}


 /******** robot service ********/

bool RobotControl::updateRobotStatus()
{
    /** Get the intial value of theoretical waypoint**/
//    aubo_robot_namespace::wayPoint_S tmp;
//    int ret = robotServiceReceive.robotServiceGetCurrentWaypointInfo(tmp);

//    SetToZero(average_sensor_data_);
//    last_send_joints_.qdot.setToZero();
//    last_send_joints_.qdotdot.setToZero();
//    current_way_point_.qdot.setToZero();
//    current_way_point_.qdotdot.setToZero();
//    for(int i = 0; i < ARM_DOF; i++)
//    {
//        last_send_joints_.q(i) = current_way_point_.q(i) = tmp.jointpos[i];
//        relative_position_[i] = 0.0;
//    }

    //first ensure the right teach mode
    //set the tool's parameters
//    robot_kine_->setToolProperty(Frame(Rotation::RPY(s_tool_pose[3],s_tool_pose[4],s_tool_pose[5]), Vector(s_tool_pose[0], s_tool_pose[1], s_tool_pose[2])));
//    return !(bool)ret;
}

bool RobotControl::initRobotService()
{
    bool flag = false;


    return flag;
}

/******** Hand Guiding function ********/
void RobotControl::startHandGuiding()
{
    QTime current_time;
    int ret, msec1 = 0, msec2 = 0, addSize, rib_buffer_size_ = 0;
    double ts;

}





double RobotControl::getHandGuidingSwitch()
{
    if(IO_name_ == "0")
        return 1;
    else
//        robotServiceReceive.robotServiceGetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDI, IO_name_, IO_switch_);
    return IO_switch_;
}

int RobotControl::enterTcp2CANMode(bool flag)
{
    int ret;
    if(flag)
    {
        // enter Tcp2Canbus Mode
        if(updateRobotStatus())
        {

        }
        else
            emit signal_handduiding_failed("update robot status failed.");
    }
    else
    {
        //        if(tcp2CanMode_)
        s_start_handguiding = false;
//        SetToZero(force_of_end_);
    }
    return ret;
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

    double joint[ARM_DOF];
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

void RobotControl::updateFTSensorData()
{
    //share memory data
    for(int i = 0; i < SENSOR_DIMENSION; i++)
        ft_share_->sensorData[i] = FTSensorDataProcess::getSensorData()[i];
}
