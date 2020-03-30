#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <fstream>
#include <iostream>
#include <string.h>
#include <QTimer>
#include <QTime>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/shm.h>

#include "FTSensorDataProcess.h"

#define CARTESIAN_FREEDOM 6
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
    int trackEnable;
    int mode;
    double sigma;
    double pid_motion[10];//6:pid+4:motion(Vmax Amax)
    double aMass[6];
    double aDamp[6];
    double aStiffness[6];
    double sensitivity[6];
    double goalWrench[6];
    double sensorData[6];
}FTSensorData;


class RobotControl: public QObject
{
     Q_OBJECT
public:
    explicit RobotControl();
    ~RobotControl();

    /******** robot service ********/
    bool initRobotService();
    bool updateRobotStatus();
    inline void setRobotIP(const std::string& ip){server_host_ip_ = ip;}

    /******** Hand Guiding function ********/
    void startHandGuiding();
//    bool getHandGuidingWaypoints(std::vector<aubo_robot_namespace::wayPoint_S> &wayPointVector);
    inline void closeHandGuidingThread(){s_thread_handguiding = false;}
    inline void enableConstraints(){enable_constraints_ = true;}    //singularity
    double getHandGuidingSwitch();
    inline void setHandGuidingSwitchIO(const std::string& IO_name__){IO_name_ = IO_name__;}
    int enterTcp2CANMode(bool flag);

    /******** Calibration function ********/
    int moveToTargetPose(int index);
    void getCalibrationPose(int index, double joint_angle[]);


    /******** Control Parameter function ********/
    void updateControlPara(const double& value, const int& index, const std::string& type);
    bool obtainCenterofMass(double result[]);
    void setToolProperty();
//    void setToolDynamics(const double I[]){tool_dynamics = I;}
    double* getToolDynamics(){return tool_dynamics;}

    /******************** Admittance Control ********************/
    void enableAdmittanceControl();
    void disableAdmittanceControl();
//    void setAdmittanceControlFT(double value, CONTROL_AXIS axis);
    void updateAdmittancePIDPara(double value, int index);
    void getMaxSigma(double sigmaValue[]);
    void updateFTSensorData();

    inline void setDragMode(int value){s_dragMode = value;}
    inline void setCalculateMethod(int value){s_calculateMethod = value;}
    inline void setControlModel(int value){s_controlModel = value;}
    inline void setControlSpace(int value){s_controlSpace = value;}
    inline void setControlPeriod(int value){s_control_period = value;}
    inline void setBufferSizeLimit(int value){s_bufferSizeLimit = value;}
    inline void setFilter1 (int value){s_filter1 = value;}
    inline void setFilter2 (int value){s_filter2 = value;}

    inline void setThreshold(double data[SENSOR_DIMENSION]){for(int i = 0; i < SENSOR_DIMENSION; i++) s_threshold[i] = data[i];}
    inline void setLimit(double data[SENSOR_DIMENSION]){for(int i = 0; i < SENSOR_DIMENSION; i++) s_limit[i] = data[i];}
    inline void setStiffness(double data[SENSOR_DIMENSION]){for(int i = 0; i < SENSOR_DIMENSION; i++) s_stiffness[i] = data[i];}
    inline void setDamp(double data[SENSOR_DIMENSION]){for(int i = 0; i < SENSOR_DIMENSION; i++) s_damp[i] = data[i];}
    inline void setMass(double data[SENSOR_DIMENSION]){for(int i = 0; i < SENSOR_DIMENSION; i++) s_mass[i] = data[i];}
    inline void setToolPose(double data[SENSOR_DIMENSION]){ for(int i = 0; i < SENSOR_DIMENSION; i++) s_tool_pose[i] = data[i];}
//    inline void setCalibrationPose(double data[SENSOR_DIMENSION], int index){for(int i = 0; i < SENSOR_DIMENSION; i++) s_pose_calibration[index](i) = data[i];}

//    Kinematics* getRobotKine(){return robot_kine_;}
    double* getRobotEndWrench(){return force_of_end_;}

public:
    static bool s_start_handguiding;
    static bool s_thread_handguiding;
    static double s_threshold[6];
    static double s_limit[6];
    static double force_of_end_[6];
    static double tool_dynamics[10];
    static double s_pose_calibration[CALIBRATION_POS::POSE_Total][6];

    static int s_control_period;
    static int s_controlSpace;
    static int s_dragMode;
    static int s_calculateMethod;
    static int s_controlModel;
    static int s_bufferSizeLimit;
    static int s_filter1;
    static int s_filter2;
    static double s_mass[SENSOR_DIMENSION];
    static double s_damp[SENSOR_DIMENSION];
    static double s_stiffness[SENSOR_DIMENSION];
    static double s_tool_pose[CARTESIAN_FREEDOM];

    std::map<std::string, int> paraType_;

signals:
    void signal_handduiding_failed(QString);

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

    bool enable_constraints_;
    bool orientaion_enable_;
    bool tcp2CanMode_;
    double IO_switch_;
    std::string IO_name_;
    std::string server_host_ip_;

//    ServiceInterface robotServiceSend;
//    ServiceInterface robotServiceReceive;

//    Kinematics *robot_kine_;
    void *shm;
    int shmid;
    FTSensorData * ft_share_;

    const double joint_max_acc_ = 100.0/180.0*M_PI;
    const double joint_max_velc_ = 50.0/180.0*M_PI;
};

#endif // ROBOTCONTROL_H
