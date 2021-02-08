#include "robot_control.h"
#include <iostream>
#include <sys/time.h>
#include <functional>
#include <exception>
#include <string.h>

//#define _Test
using namespace ARAL;

void timespec_add_us(struct timespec *t, long us)
{
    t->tv_nsec += us*1000;
    if (t->tv_nsec > 1000000000)
    {
        t->tv_nsec = t->tv_nsec - 1000000000;// + ms*1000000;
        t->tv_sec += 1;
    }
}

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

RobotControl::RobotControl(const std::string& model, const std::string& robot_ip, const int& port)
{
    sensor_calibrated_ = false;
    aral_interface_ = CreateRLIntfacePtr(model.c_str(), LOG_DEBUG);
    createParaTable();

    aubo_driver = createAuboDriver();
    if(!aubo_driver->connectToServer(robot_ip, port))
    {
        std::cerr<<" 连接服务器失败!"<<std::endl;
        exit(-1);
    }

    if (aubo_driver->login("user", "111", 50))
    {
        // 定制需要获得的反馈数据
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

    initalControlPara();
}

RobotControl::~RobotControl()
{
    //clean the force_control_ thread. first
    enable_thread_ = false;
    usleep(10 * 1000);   //wait for 10ms

    aral_interface_ = nullptr;

    if(aubo_driver != NULL)
        delete aubo_driver;
}

void RobotControl::initalControlPara()
{
    aral_interface_->mcSetControlType(FORCE_ADM_CONTROL);
    aral_interface_->rsSetFeedBackOptions(0x03);     // only position and joint current feedback, no velocity and acceleration feedback
    aral_interface_->fcEnableSingularityConsistent(false);
    tool_pose_.fill(0);
    ft_sensor_pose_.fill(0);
    goal_wrench_.fill(0);
    enable_thread_ = false;
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

void RobotControl::updateRobotStatus()
{
#ifdef _Test
    sensor_data_ = {0,0,2,0,0,0};
    sensor_data_ = {0,0,0,0,0,0};
#endif

    mutex_.lock();
    aral_interface_->rsUpdateEndFTSensor(sensor_data_.data());
    aral_interface_->rsUpdatJointPVA(cur_joint_pos_.data(), NULL, NULL);
    mutex_.unlock();
}

void RobotControl::updateRobotGoal()
{
    // update reference trajectory
    /*double q[ROBOT_DOF], qd[ROBOT_DOF], qdd[ROBOT_DOF]*/;
    aral_interface_->rsSetRefTraj();
}

int RobotControl::getRobotOutput()
{
    cmd_joint_pos_.resize(ROBOT_DOF);
    aral_interface_->fcGetRobotEndWrench(force_of_end_.data());
    return aral_interface_->rsCalJointCommand(cmd_joint_pos_.data());
}

void RobotControl::getRobotEndWrench(double* wrench)
{
    aral_interface_->fcGetRobotEndWrench(wrench);
}

void RobotControl::getRtdeData(AuboDriver *m_instance)
{
    if (0 == m_instance->getJointPositions().size())
        m_instance->syncRtdeTransmission(0, 50);

    mutex_.lock();
    cur_joint_pos_ = m_instance->getJointPositions();
    cur_joint_vel_ = m_instance->getJointVelocities();
    sensor_data_ = m_instance->getTcpForce();

#ifdef _Test
    if(sensor_data_.size() == 0)
        sensor_data_.resize(6);
    if(current_joints_.size() == 0)
        current_joints_.resize(6);
    current_joints_ = {83.74*M_PI/180, 2*M_PI/180, 108.57*M_PI/180, 17.07*M_PI/180, 92.3*M_PI/180, 10*M_PI/180};
#endif
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
    mutex_.unlock();
}

int RobotControl::startForceControl()
{
    enable_thread_ = true;
    force_control_Thread_ = std::thread(&RobotControl::forceControlThread, this);
    if(force_control_Thread_.joinable())
        force_control_Thread_.detach();
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
    struct timespec next;
    clock_gettime(CLOCK_REALTIME, &next);

    int ret;
    auto builder = aubo_driver->getRtdeInputBuilder();
    aral_interface_->fcEnable(true);
    while(enable_thread_)
    {
        updateRobotStatus();        //更新当前状态(包括机械臂关节和传感器状态)
        updateRobotGoal();          //设置运动目标(包括参考轨迹和机械臂在力控坐标系下输出的wrench)
        if((ret = getRobotOutput()) < 0)    //计算控制输出:机械臂关节轨迹或者驱动力矩
        {
            std::cerr<<"控制命令计算失败, 错误码:"<<ret<<std::endl;
            break;
        }

        builder->servoJoint(cmd_joint_pos_);
        builder->send();            //servoJ模式下发控制指令

        timespec_add_us(&next, control_period_ * 1000); // unit: us
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);    //按照绝对时间进行休眠
    }
    aral_interface_->fcEnable(false);

    return 0;
}

void RobotControl::obtainCalibrationPos(const int& index)
{
    //obtain the sensor data in pose 1 2,3;
    mutex_.lock();
    for(int i = 0; i < 6; i++)
        s_calibrationMeasurements[index][i] = sensor_data_[i];
    mutex_.unlock();
    std::cout<<"calibrationMeasurements value: "<<s_calibrationMeasurements[index][0]<<","<<s_calibrationMeasurements[index][1]<<","
            <<s_calibrationMeasurements[index][2]<<","<<s_calibrationMeasurements[index][3]<<","<<s_calibrationMeasurements[index][4]<<","<<s_calibrationMeasurements[index][5]<<std::endl;
}

void RobotControl::enableConstraints(const bool& flag)
{
    aral_interface_->fcEnableSingularityConsistent(flag);
}

void RobotControl::setConstrainPara(const double* value)
{
    aral_interface_->fcSetConstrainPara(value);
}

void RobotControl::setSelectionVector(const int* vec)
{
    aral_interface_->fcSetSelectVector(vec);
}

void RobotControl::setControlPeriod(const int& period)
{
    control_period_ = period;
    aral_interface_->mcSetControlPeriod(period * 1.0 / 1000);       //将单位转换成秒
}

void RobotControl::updateControlPara(const double& value, const int& index, const std::string& typeName)
{
    return;
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

void RobotControl::setMaxTranSpeed(double vel)
{
    aral_interface_->mcSetMaxTranSpeed(vel);
}

void RobotControl::setMaxRotSpeed(double rot)
{
    aral_interface_->mcSetMaxRotSpeed(rot);
}

/******** Calibration function ********/
int RobotControl::moveToTargetPose(int index)
{
    std::vector<double> joints(ROBOT_DOF);
    memcpy(joints.data(), calibration_poses_[index].data(), sizeof (double) * ROBOT_DOF);
    try
    {
        aubo_driver->moveJoint(joints, 0.5, 90 * M_PI / 180.);
    }
    catch (exception e) {
        std::cout<<e.what()<<std::endl;
    }

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (distance(aubo_driver->getJointPositions(), joints) < 0.05)
            break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));    //wait for robot and sensor stable

    return 0;
}

int RobotControl::calibrateFTSensor(FtSensorCalibrationResult &result)
{
    for (int i = 0; i < 3; i++)
    {
        std::cout<<s_calibrationMeasurements[i][0]<<", "<<s_calibrationMeasurements[i][1]<<", "<<s_calibrationMeasurements[i][2]<<", "
                                                 <<s_calibrationMeasurements[i][3]<<", "<<s_calibrationMeasurements[i][4]<<", "<<s_calibrationMeasurements[i][5]<<", "<<std::endl;
    }

    for (int i = 0; i < 3; i++)
    {
        std::cout<<calibration_poses_[i][0]<<", "<<calibration_poses_[i][1]<<", "<<calibration_poses_[i][2]<<", "
                                          <<calibration_poses_[i][3]<<", "<<calibration_poses_[i][4]<<", "<<calibration_poses_[i][5]<<", "<<std::endl;
    }
    if(aral_interface_->calibToolAndSensor(calibration_poses_, s_calibrationMeasurements, result) == 0)
    {
        aral_interface_->rsUpdateEndFTSensorOffset(result.offset);
        aral_interface_->mdlSetToolInertialFromFTSensor(result.mass, result.com.data(), tool_inertia_.data());
        sensor_calibrated_ = true;
    }

    return 0;
}

/******** Control Parameter function ********/

/******************** Admittance Control ********************/

void RobotControl::enableAdmittanceControl()
{
    aral_interface_->fcEnable(true);
}

void RobotControl::disableAdmittanceControl()
{
    aral_interface_->fcEnable(false);
}

void RobotControl::updateAdmittancePIDPara(const double& /*value*/, const int& /*index*/)
{
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

void RobotControl::setControlSpace(unsigned int value)
{
    aral_interface_->fcSetTaskFrame((CoordTypeEnum)value);
}

void RobotControl::setSensorFilter(const double value)
{
    aral_interface_->fcSetSensorFilter(value);
}

void RobotControl::setControlFilter(const double value)
{
    aral_interface_->fcSetControlFilter(value);
}

void RobotControl::setCalibrationPose(double data[SENSOR_DIMENSION], int index)
{
    for(int i = 0; i < SENSOR_DIMENSION; i++)
        calibration_poses_[index][i] = data[i];
}

void RobotControl::setEndFTSensorThreshold(double data[SENSOR_DIMENSION])
{
    end_ft_threshold_.resize(CARTESIAN_FREEDOM);
    memcpy(end_ft_threshold_.data(), data, sizeof (double)*CARTESIAN_FREEDOM);
    aral_interface_->fcSetEndFTSensorThreshold(data);
}

void RobotControl::setEndFTSensorLimit(double data[SENSOR_DIMENSION])
{
    end_ft_limit_.resize(CARTESIAN_FREEDOM);
    memcpy(end_ft_limit_.data(), data, sizeof (double)*CARTESIAN_FREEDOM);
    aral_interface_->fcSetEndFTSensorLimit(data);
}

void RobotControl::setEndFTSensorSensitivity(double data[SENSOR_DIMENSION])
{
    end_ft_sensitivity_.resize(CARTESIAN_FREEDOM);
    memcpy(end_ft_sensitivity_.data(), data, sizeof (double)*CARTESIAN_FREEDOM);
    aral_interface_->fcSetSensitivity(data);
}

int RobotControl::setCartStiffness(double data[CARTESIAN_FREEDOM])
{
    cart_stiffness_.resize(CARTESIAN_FREEDOM);
    memcpy(cart_stiffness_.data(), data, sizeof (double)*CARTESIAN_FREEDOM);
    return aral_interface_->fcSetCartStiffness(data);
}

int RobotControl::setCartDamp(double data[CARTESIAN_FREEDOM])
{
    cart_damp_.resize(CARTESIAN_FREEDOM);
    memcpy(cart_damp_.data(), data, sizeof (double)*CARTESIAN_FREEDOM);
    return aral_interface_->fcSetCartDamp(data);
}

int RobotControl::setCartMass(double data[CARTESIAN_FREEDOM])
{
    cart_mass_.resize(CARTESIAN_FREEDOM);
    memcpy(cart_mass_.data(), data, sizeof (double)*CARTESIAN_FREEDOM);
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
    aral_interface_->mdlSetToolPose(data, POS_RPY);
}

void RobotControl::setToolInertia(const double data[INERTIA_DIM])
{
    tool_inertia_.resize(INERTIA_DIM);
    memcpy(tool_inertia_.data(), data, sizeof(double) * INERTIA_DIM);
}

void RobotControl::setFTSensorPose(double data[SENSOR_DIMENSION])
{
    aral_interface_->mdlSetEndSensorPose(data, POS_RPY);
}
