#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <iostream>
#include "AuboRobotMetaType.h"
#include "serviceinterface.h"
#include "FTSensorDataProcess.h"

#include "../utility/include/util.h"
#include "../utility/include/rmatrix.h"
#include "../utility/include/rvector.h"


#include <string.h>
#include <QTimer>
#include <QTime>

using namespace std;


#define SERVER_HOST "192.168.1.100"
#define SERVER_PORT 8899
#define MAX_ACCELERATION 500.0/180.0*M_PI
#define MAX_VELOCITY 90.0/180.0*M_PI
#define BigModuleRatio 2 * M_PI / 60.0 / 121
#define SmallModuleRatio 2 * M_PI / 60.0 / 101
#define Vmax 1500
#define Amax 3000
#define Jmax 6000
#define CONTROL_CYCLE 200
#define CARTESIAN_FREEDOM 6
#define POSE_NUM 3


class RobotControl: public QObject
{
     Q_OBJECT
public:
    RobotControl();
    ~RobotControl();

    static RobotControl *instance();
    bool initRobotService();
    void startHandGuiding();
    int enterTcp2CANMode(bool flag);
    void setToolProperty();
    int moveToTargetPose(int index);


    bool ObtainCenterofMass();

    static int s_control_period;
    static bool s_start_handguiding;
    static bool s_thread_handguiding;
    static double s_accumulate_time;
    static double s_tool_pose[CARTESIAN_FREEDOM];
    static double m_toolPosition[3];
    static double m_toolOrientation[9];

signals:
    void signal_handduiding_failed(const QString str = NULL);

private:
    float average_sensor_data_[SENSOR_DIMENSION];

    aubo_robot_namespace::MoveRelative relative_pose_;
    double relative_axis_angles_[CARTESIAN_FREEDOM];
    double relative_position_[CARTESIAN_FREEDOM];

    double last_send_joints_[CARTESIAN_FREEDOM];
    double last_acclerations_[CARTESIAN_FREEDOM];
    double penult_send_joints_[CARTESIAN_FREEDOM];
    double last_velocities_[CARTESIAN_FREEDOM];
    double current_acclerations_[CARTESIAN_FREEDOM];
    double current_velocities_[CARTESIAN_FREEDOM];

    aubo_robot_namespace::wayPoint_S theoretical_way_point_;
    aubo_robot_namespace::wayPoint_S  current_way_point_;
    aubo_robot_namespace::wayPoint_S  initial_way_point_;
    aubo_robot_namespace::wayPoint_S intermediate_way_point_;
    aubo_robot_namespace::JointVelcAccParam joint_max_acc_, joint_max_velc_;

    bool orientaion_enable_;
    bool tcp2canMode_;

    ServiceInterface robotServiceSend;
    ServiceInterface robotServiceReceive;
    aubo_robot_namespace::Rpy rpy;
    aubo_robot_namespace::Ori ori;
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;


    bool calculateTheoreticalWaypoint();
 //ADD

};

#endif // ROBOTCONTROL_H
