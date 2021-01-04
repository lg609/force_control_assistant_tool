#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <fstream>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/shm.h>
#include <mutex>          // std::mutex

#include <string>
#include <iostream>

#include "sensor_data_process.h"
#include "aral/robot_library_interface.hpp"
#include <aubo_driver/aubo_driver.h>
using namespace arcs::aubo_driver;

#define SERVER_HOST "127.0.0.1"
#define SERVER_PORT 8899

enum DRAG_MODE
{
    POSITION = 0,
    ORI,
    POSE
};

enum CONTROL_AXIS
{
    FX = 0,
    FY,
    FZ,
    TX,
    TY,
    TZ
};

enum PARA_TYPE
{
    CART_MASS = 0,
    CART_DAMP,
    CART_STIFFNESS,
    JOINT_MASS,
    JOINT_DAMP,
    JOINT_STIFFNESS,
    END_FT_THRESHOLD,
    BASE_FT_THRESHOLD,
    JOINT_FT_THRESHOLD,
    END_FT_LIMIT,
    BASE_FT_LIMIT,
    JOINT_FT_LIMIT,
    TOOL_POSE,
    SENSOR_POSE,
    SELECT_VECTOR,
    CONSTRAINT_PARA,
    GOAL_WRENCH,
};

using namespace ARAL;
using namespace std;


class RobotControl
{
public:
    explicit RobotControl(const std::string& model);
    //!
    ~RobotControl();

    bool initShareMemory();
    //!
    void updateRobotStatus();
    //!
    void updateRobotGoal();
    //!
    void getRobotOutput();
    //!
    void getRobotEndWrench(double * wrench);
    //!
    void setControlPeriod(const double period);
    //!
    void setMaxTranSpeed(double vel);
    //!
    void setMaxRotSpeed(double rot);

    /******** Force Control function ********/
    void startForceControl();

    void enableForceControlThread(bool);
    //!
    void setSelectionVector(const double * vec);
    //!
    void setEndFTSensorThreshold(double data[SENSOR_DIMENSION]);
    //!
    void setEndFTSensorLimit(double data[SENSOR_DIMENSION]);
    //!
    int setCartStiffness(double data[CARTESIAN_FREEDOM]);
    //!
    int setCartDamp(double data[CARTESIAN_FREEDOM]);
    //!
    int setCartMass(double data[CARTESIAN_FREEDOM]);
    //!
    void setOverEstimatedDis(const double dis);
    //!
    void setGoalWrench(const double* wrench);


    /******** Calibration function ********/
    int moveToTargetPose(int index);
    //!
    void getCalibrationPose(int index, double* joint_angle);
    //!
    int calibrateFTSensor(FtSensorCalibrationResult &result);


    /******** Control Parameter function ********/
    void initalControlPara();
    //!
    void createParaTable();
    //!
    void updateControlPara(const double& value, const int& index, const std::string& type);
    //!
    void setToolDynamicsFromFTSensor(const RigidBodyInertia &I);
    //!
    void setToolPose(double data[SENSOR_DIMENSION]);
    //!
    void setFTSensorPose(double data[SENSOR_DIMENSION]);
    //!
    RigidBodyInertia getToolDynamics(){return tool_dynamics;}

    /******************** Admittance Control ********************/
    //!
    void enableConstraints(bool flag);    //singularity
    //!
    void setConstrainPara(const double* value);    //singularity
    //!
    void enableAdmittanceControl();
    //!
    void disableAdmittanceControl();
    //!
    void updateAdmittancePIDPara(double value, int index);
    //!
    void setDragMode(unsigned int value);
    //!
    void setForceControlMode(unsigned int mode);
    //!
    void setThreadMode(unsigned int mode);
    //!
    void setCalMethod(unsigned int type);
    //!
    void setControlSpace(unsigned int value);
    //!
    void setFilter1(const double value);    // value: 0-1
    //!
    void setFilter2(const double value);
    //!
    bool getHandGuidingSwitch();

    //!
    inline void setHandGuidingSwitchIO(const std::string& IO_name__)
    {
        IO_name_ = IO_name__;
    }

    inline void setCalibrationPose(double data[SENSOR_DIMENSION], int index)
    {
        for(int i = 0; i < SENSOR_DIMENSION; i++)
            calibration_poses_[index][i] = data[i];
    }


private:
    typedef RLJntArray JointArray;
    typedef RLPose CartArray;
    bool enable_thread_;
    double control_period_;
    JointArray calibration_poses_[CALIBRATION_POS::POSE_Total];
    RigidBodyInertia tool_dynamics;
    CartArray cart_mass_;
    CartArray cart_damp_;
    CartArray cart_stiffness_;
    CartArray tool_pose_;
    CartArray ft_sensor_pose_;
    CartArray selection_vector_;
    CartArray constraint_para_;
    Wrench end_ft_threshold_;
    Wrench end_ft_limit_;

    Wrench force_of_end_;
    Wrench goal_wrench_;

private:

    double last_send_joints_;
    double current_way_point_;

    double current_joint_dd_[CARTESIAN_FREEDOM];
    double current_joint_d_[CARTESIAN_FREEDOM];
    double last_joint_d_[CARTESIAN_FREEDOM];
    double last_joint_dd_[CARTESIAN_FREEDOM];

    bool orientaion_enable_;
    std::string IO_name_;
    bool IO_switch_;

//    Kinematics *robot_kine_;
    void *shm;
    int shmid;
    ForceControlData * ft_share_;
    std::thread* force_control_;
    ARAL::RLIntfacePtr aral_interface_;
    std::mutex mtx_;
    static int count;
    struct timeval time1, time2;

    std::map<std::string, int> para_table_;

    FtSensorCalibrationResult ft_sensor_calib_res_;

    const double joint_max_acc_ = 100.0/180.0*M_PI;
    const double joint_max_velc_ = 50.0/180.0*M_PI;
#ifdef USE_SDK
    ServiceInterface robot_service_;
#endif
};

#endif // ROBOTCONTROL_H
