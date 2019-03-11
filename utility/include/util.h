#ifndef UTIL_H
#define UTIL_H

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

#define SAFETY_COEFFICIENT  0.0157

class Util
{
public:
    Util();

    /** 打印路点信息 **/
    static void printWaypoint(aubo_robot_namespace::wayPoint_S &wayPoint);

    /** 打印关节状态信息 **/
    static void printJointStatus(const aubo_robot_namespace::JointStatus *jointStatus, int len);

    /** 打印事件信息 **/
    static void printEventInfo(const aubo_robot_namespace::RobotEventInfo &eventInfo);

    /** 打印诊断信息 **/
    static void printRobotDiagnosis(const aubo_robot_namespace::RobotDiagnosis &robotDiagnosis);

    static void initJointAngleArray(double *array, double joint0,double joint1,double joint2,double joint3,double joint4,double joint5);

    static double SIGN(double x);

    static void angleAxisToTran(double angle, double axis[], double tran[]);

    static bool tranToQuaternion(double eerot[], aubo_robot_namespace::Ori &ori);

    static void quaternionToOriMatrix(aubo_robot_namespace::Ori q, double eerot[]);

    static void hMatrixMultiply(double eerot[], double *eetrans, double eerot1[], double *eetrans1, aubo_robot_namespace::wayPoint_S *pos_transferred, double *new_rot = NULL);

    static void incrementalTransformation(double eeRot[], double eeTrans[], double incRot[], double incTrans[], aubo_robot_namespace::wayPoint_S *pos_transferred);

    static bool checkForSafe(double j1[], double j2[], int N);

    static bool getAngleVelocity(const double* ds, double* q, double *dq);

    static void EulerAngleToQuaternion(double *EulerAngle, double *orientation);
};

#endif // UTIL_H
