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

#include "sensor_data_process.h"
#include "rl/robot_interface.hpp"

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

typedef struct
{
    //share memory data with robot server
    int trackEnable;    // flag
    int mode;
    double sigma;
    double pid_motion[10];//6:pid+4:motion(Vmax Amax)
    double aMass[6];
    double aDamp[6];
    double aStiffness[6];
    double sensitivity[6];
    double curCurrent[6];
    double curJointPos[6];
    double cmdJointPos[6];
    double cmdCurrent[6];
}ForceControlData;

#define CONTROL_PERIOD 0.005
using namespace ARAL;

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

    /******** Force Control function ********/
    void startForceControl();
    //!
    void enableConstraints(bool flag);    //singularity
    //!
    void setSelectionVector();
    //!
    void setControlPeriod(const double period);
    //!
    bool getHandGuidingSwitch();
    //!
    inline void setHandGuidingSwitchIO(const std::string& IO_name__){IO_name_ = IO_name__;}

    /******** Calibration function ********/
    int moveToTargetPose(int index);
    //!
    void getCalibrationPose(int index, double joint_angle[]);


    /******** Control Parameter function ********/
    void initalControlPara();
    //!
    void updateControlPara(const double& value, const int& index, const std::string& type);
    //!
    int calibrateFTSensor(FtSensorCalibrationResult &result);
    //!
    void setToolProperty();
    //!
    void setToolDynamics(const RigidBodyInertia &I);
    //!
    RigidBodyInertia getToolDynamics(){return tool_dynamics;}

    /******************** Admittance Control ********************/
    void enableAdmittanceControl();
    //!
    void disableAdmittanceControl();
    //!
    void setAdmittanceControlFT(double value, CONTROL_AXIS axis);
    //!
    void updateAdmittancePIDPara(double value, int index);
    //!
    void setDragMode(unsigned int value);
    //!
    void setForceControlMode(unsigned int mode);
    //!
    void setControlSpace(unsigned int value);
    //!
    void setFilter1(const double value);    // value: 0-1
    //!
    void setFilter2(const double value);

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
    void setToolPose(double data[SENSOR_DIMENSION])
    {
        for(int i = 0; i < SENSOR_DIMENSION; i++)
            s_tool_pose[i] = data[i];
    }
    inline void setCalibrationPose(double data[SENSOR_DIMENSION], int index)
    {
        for(int i = 0; i < SENSOR_DIMENSION; i++)
            s_pose_calibration[index](i) = data[i];
    }

    ::Wrench getRobotEndWrench(){return force_of_end_;}

public:
    static bool s_start_handguiding;
    static bool s_thread_handguiding;
    static double s_threshold[6];
    static double s_limit[6];
    static ::Wrench force_of_end_;
    static RigidBodyInertia tool_dynamics;
    static JointArray s_pose_calibration[CALIBRATION_POS::POSE_Total];

    static int s_control_period;

    static double s_mass[SENSOR_DIMENSION];
    static double s_damp[SENSOR_DIMENSION];
    static double s_stiffness[SENSOR_DIMENSION];
    static double s_tool_pose[CARTESIAN_FREEDOM];

    std::map<std::string, int> paraType_;

    double selection_vector_[CARTESIAN_FREEDOM];


private:
    double average_sensor_data_;
//    aubo_robot_namespace::MoveRelative relative_pose_;
    double relative_axis_angles_[CARTESIAN_FREEDOM];
    double relative_position_[CARTESIAN_FREEDOM];

    double last_send_joints_;
    double current_way_point_;

    double current_joint_dd_[CARTESIAN_FREEDOM];
    double current_joint_d_[CARTESIAN_FREEDOM];
    double last_joint_d_[CARTESIAN_FREEDOM];
    double last_joint_dd_[CARTESIAN_FREEDOM];

    bool orientaion_enable_;
    bool IO_switch_;
    std::string IO_name_;

//    Kinematics *robot_kine_;
    void *shm;
    int shmid;
    ForceControlData * ft_share_;
    std::thread* force_control_;
    RLIntface *aral_interface_;

    FtSensorCalibrationResult ft_sensor_calib_res_;

    const double joint_max_acc_ = 100.0/180.0*M_PI;
    const double joint_max_velc_ = 50.0/180.0*M_PI;
};

#endif // ROBOTCONTROL_H
