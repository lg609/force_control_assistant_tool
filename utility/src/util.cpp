#include "../include/util.h"
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include "kinematics.cpp"
#include "../include/rvector.h"
#include "../include/rmatrix.h"

Util::Util()
{

}

//打印路点信息
void Util::printWaypoint(aubo_robot_namespace::wayPoint_S &wayPoint)
{
    std::cout<<std::endl<<"start-------------路点信息---------------"<<std::endl;
    //位置信息
    std::cout<<"位置信息: ";
    std::cout<<"x:"<<wayPoint.cartPos.position.x<<"  ";
    std::cout<<"y:"<<wayPoint.cartPos.position.y<<"  ";
    std::cout<<"z:"<<wayPoint.cartPos.position.z<<std::endl;

    //姿态信息
    std::cout<<"姿态信息: ";
    std::cout<<"w:"<<wayPoint.orientation.w<<"  ";
    std::cout<<"x:"<<wayPoint.orientation.x<<"  ";
    std::cout<<"y:"<<wayPoint.orientation.y<<"  ";
    std::cout<<"z:"<<wayPoint.orientation.z<<std::endl;

    //    aubo_robot_namespace::Rpy tempRpy;
    //    robotService.quaternionToRPY(wayPoint.orientation,tempRpy);
    //    std::cout<<"RX:"<<tempRpy.rx<<"  RY:"<<tempRpy.ry<<"   RZ:"<<tempRpy.rz<<std::endl;

    //关节信息
    std::cout<<"关节信息: "<<std::endl;
    for(int i=0;i<aubo_robot_namespace::ARM_DOF;i++)
    {
        std::cout<<"joint"<<i+1<<": "<<wayPoint.jointpos[i]<<" ~ "<<wayPoint.jointpos[i]*180.0/M_PI<<std::endl;
    }
}


//打印关节状态信息
void Util::printJointStatus(const aubo_robot_namespace::JointStatus *jointStatus, int len)
{
    std::cout<<std::endl<<"start----------关节状态信息-------" << std::endl;

    for(int i=0; i<len; i++)
    {
        std::cout<<"关节ID:"   <<i<<"  " ;
        std::cout<<"电流:"     <<jointStatus[i].jointCurrentI<<" ";
        std::cout<<"速度:"     <<jointStatus[i].jointSpeedMoto<<" ";
        std::cout<<"关节角:"   <<jointStatus[i].jointPosJ<<" "<<" ~ "<<jointStatus[i].jointPosJ*180.0/M_PI;
        std::cout<<"电压   :"  <<jointStatus[i].jointCurVol<<" ";
        std::cout<<"温度   :"  <<jointStatus[i].jointCurTemp<<" ";
        std::cout<<"目标电流:"  <<jointStatus[i].jointTagCurrentI<<" ";
        std::cout<<"目标电机速度:" <<jointStatus[i].jointTagSpeedMoto<<" ";
        std::cout<<"目标关节角 :"  <<jointStatus[i].jointTagPosJ<<" ";
        std::cout<<"关节错误   :"  <<jointStatus[i].jointErrorNum <<std::endl;
    }
    std::cout<<std::endl;
}

//打印事件信息
void Util::printEventInfo(const aubo_robot_namespace::RobotEventInfo &eventInfo)
{
    std::cout<<"事件类型:"<<eventInfo.eventType <<"  code:"<<eventInfo.eventCode<<"  内容:"<<eventInfo.eventContent<<std::endl;
}

void Util::printRobotDiagnosis(const aubo_robot_namespace::RobotDiagnosis &robotDiagnosis)
{
    std::cout<<std::endl<<"start----------机械臂统计信息-------" << std::endl;

    std::cout<<std::endl<<"   "<<"CAN通信状态:"<<(int)robotDiagnosis.armCanbusStatus;
    std::cout<<std::endl<<"   "<<"电源当前电流:"<<robotDiagnosis.armPowerCurrent;
    std::cout<<std::endl<<"   "<<"电源当前电压:"<<robotDiagnosis.armPowerVoltage;

    (robotDiagnosis.armPowerStatus)? std::cout<<std::endl<<"   "<<"48V电源状态:开":std::cout<<std::endl<<"   "<<"48V电源状态:关";

    std::cout<<std::endl<<"   "<<"控制箱温度:"<<(int)robotDiagnosis.contorllerTemp;
    std::cout<<std::endl<<"   "<<"控制箱湿度:"<<(int)robotDiagnosis.contorllerHumidity;
    std::cout<<std::endl<<"   "<<"远程关机信号:"<<robotDiagnosis.remoteHalt;
    std::cout<<std::endl<<"   "<<"机械臂软急停:"<<robotDiagnosis.softEmergency;
    std::cout<<std::endl<<"   "<<"远程急停信号:"<<robotDiagnosis.remoteEmergency;
    std::cout<<std::endl<<"   "<<"碰撞检测位:"<<robotDiagnosis.robotCollision;
    std::cout<<std::endl<<"   "<<"进入力控模式标志位:"<<robotDiagnosis.forceControlMode;
    std::cout<<std::endl<<"   "<<"刹车状态:"<<robotDiagnosis.brakeStuats;
    std::cout<<std::endl<<"   "<<"末端速度:"<<robotDiagnosis.robotEndSpeed;
    std::cout<<std::endl<<"   "<<"最大加速度:"<<robotDiagnosis.robotMaxAcc;
    std::cout<<std::endl<<"   "<<"上位机软件状态位:"<<robotDiagnosis.orpeStatus;
    std::cout<<std::endl<<"   "<<"位姿读取使能位:"<<robotDiagnosis.enableReadPose;
    std::cout<<std::endl<<"   "<<"安装位置状态:"<<robotDiagnosis.robotMountingPoseChanged;
    std::cout<<std::endl<<"   "<<"磁编码器错误状态:"<<robotDiagnosis.encoderErrorStatus;
    std::cout<<std::endl<<"   "<<"静止碰撞检测开关:"<<robotDiagnosis.staticCollisionDetect;
    std::cout<<std::endl<<"   "<<"关节碰撞检测:"<<robotDiagnosis.jointCollisionDetect;
    std::cout<<std::endl<<"   "<<"光电编码器不一致错误:"<<robotDiagnosis.encoderLinesError;
    std::cout<<std::endl<<"   "<<"关节错误状态:"<<robotDiagnosis.jointErrorStatus;
    std::cout<<std::endl<<"   "<<"奇异点过速警告:"<<robotDiagnosis.singularityOverSpeedAlarm;
    std::cout<<std::endl<<"   "<<"电流错误警告:"<<robotDiagnosis.robotCurrentAlarm;
    std::cout<<std::endl<<"   "<<"tool error:"<<(int)robotDiagnosis.toolIoError;
    std::cout<<std::endl<<"   "<<"安装位置错位:"<<robotDiagnosis.robotMountingPoseWarning;
    std::cout<<std::endl<<"   "<<"mac缓冲器长度:"<<robotDiagnosis.macTargetPosBufferSize;
    std::cout<<std::endl<<"   "<<"mac缓冲器有效数据长度:"<<robotDiagnosis.macTargetPosDataSize;
    std::cout<<std::endl<<"   "<<"mac数据中断:"<<robotDiagnosis.macDataInterruptWarning;

    std::cout<<std::endl<<"----------------------------------end."<<std::endl;
}

void Util::initJointAngleArray(double *array, double joint0, double joint1, double joint2, double joint3, double joint4, double joint5)
{
    array[0] = joint0;
    array[1] = joint1;
    array[2] = joint2;
    array[3] = joint3;
    array[4] = joint4;
    array[5] = joint5;
}


void Util::angleAxisToTran(double angle, double axis[], double tran[])
{
    double st = sin(angle);
    double ct = cos(angle);
    double verst = 1 - ct;
    tran[0] = axis[0] * axis[0] * verst + ct;
    tran[1] = axis[0] * axis[1] * verst - axis[2] * st;
    tran[2] = axis[0] * axis[2] * verst + axis[1] * st;
    tran[3] = axis[0] * axis[1] * verst + axis[2] * st;
    tran[4] = axis[1] * axis[1] * verst + ct;
    tran[5] = axis[1] * axis[2] * verst - axis[0] * st;
    tran[6] = axis[0] * axis[2] * verst - axis[1] * st;
    tran[7] = axis[1] * axis[2] * verst + axis[0] * st;
    tran[8] = axis[2] * axis[2] * verst + ct;
}

void Util::quaternionToOriMatrix(aubo_robot_namespace::Ori q, double eerot[])
{
    double qw;
    double qx;
    double qy;
    double qz;
    double n;

    qw = q.w;
    qx = q.x;
    qy = q.y;
    qz = q.z;
    n = 1.0f/sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    qw *= n;
    qx *= n;
    qy *= n;
    qz *= n;
    eerot[0] = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;  eerot[1] = 2.0f*qx*qy - 2.0f*qz*qw;         eerot[2] = 2.0f*qx*qz + 2.0f*qy*qw;
    eerot[3] = 2.0f*qx*qy + 2.0f*qz*qw;         eerot[4] = 1.0f - 2.0f*qx*qx - 2.0f*qz*qz;  eerot[5] = 2.0f*qy*qz - 2.0f*qx*qw;
    eerot[6] = 2.0f*qx*qz - 2.0f*qy*qw;         eerot[7] = 2.0f*qy*qz + 2.0f*qx*qw;         eerot[8] = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;
}

void Util::incrementalTransformation(double eeRot[], double eeTrans[], double incRot[], double incTrans[], aubo_robot_namespace::wayPoint_S *pos_transferred)
{
    double R1[9] = {0}, eetrans_result[3] ={0};
    int i,j,k;
    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            //incre pos: R * incTrans
            eetrans_result[i] += eeRot[i*3+j] * incTrans[j];
            //new ori-array: R1 = R*R1
            for (k = 0; k < 3; k++)
                R1[i*3+j] += eeRot[i*3+k] * incRot[k*3+j];
        }
    }

//    double RT[] = {eeRot[0], eeRot[3], eeRot[6], eeRot[1], eeRot[4], eeRot[7], eeRot[2], eeRot[5], eeRot[8]};

//    for (i = 0; i < 3; i++)
//    {
//        for (j = 0; j < 3; j++)
//        {
//            //new ori-array: R2 = R1 * R'
//            for (k = 0; k < 3; k++)

//                R2[i*3+j] += R1[i*3+k]*RT[k*3+j];
//        }
//    }

    tranToQuaternion(R1, pos_transferred->orientation);
    for (i=0; i<3; i++)
        pos_transferred->cartPos.positionVector[i] = eetrans_result[i];
}

void Util::hMatrixMultiply(double eerot[], double *eetrans, double eerot1[], double *eetrans1, aubo_robot_namespace::wayPoint_S *pos_transferred, double *new_rot)
{
    double eerot_result[9] = {0}, eetrans_result[3]={0};
    int i,j,k;
    bool hMatrix = (eetrans != NULL && eetrans1 != NULL && pos_transferred != NULL);

    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            //new ori-array: R*R1
            for (k = 0; k < 3; k++)
                eerot_result[i*3+j] += eerot[i*3+k]*eerot1[k*3+j];//multiply totation matrix

            //new pos: R*pOrg1
            if (hMatrix) eetrans_result[i] += eerot[i*3+j]*eetrans1[j];
        }
        if (hMatrix) eetrans_result[i] += eetrans[i];
    }

    if (hMatrix)
    {
        //transfer to quaternion & update position
        tranToQuaternion(eerot_result, pos_transferred->orientation);
        for (i=0;i<3;i++) pos_transferred->cartPos.positionVector[i] = eetrans_result[i];
    }
    else if (new_rot != NULL) memcpy(new_rot, eerot_result, sizeof(double)*9);
}

double Util::SIGN(double x)
{
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

bool Util::checkForSafe(double j1[], double j2[], int N)
{
    for(int i =0; i < N; i++)
    {
        if(abs(j1[i] -  j2[i]) > SAFETY_COEFFICIENT)
            return false;
    }
    return true;
}

bool Util::tranToQuaternion(double eerot[], aubo_robot_namespace::Ori &ori)
{
    double q[4], r;
    q[0] = (eerot[0] + eerot[4] + eerot[8] + 1)/4;
    for (int i = 1; i < 4; i++) q[i] = (eerot[4*(i-1)]+1)/2.0-q[0];
    for (int i = 0; i < 4; i++)
    {
        if (q[i] < 0)
        {
            if (q[i] < -1e6)
                std::cout<< "negative quaternion square:"<<i<<" "<<q[i]; //due to precision lose of rotation array.
            q[i] = 0;
        }
        else q[i] = sqrt(q[i]);
    }
    q[1] = SIGN(eerot[7] - eerot[5])*q[1];
    q[2] = SIGN(eerot[2] - eerot[6])*q[2];
    q[3] = SIGN(eerot[3] - eerot[1])*q[3];
    r = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (r == 0) return false;

    ori.w = q[0]/r;
    ori.x = q[1]/r;
    ori.y = q[2]/r;
    ori.z = q[3]/r;
    return true;
}

bool Util::getAngleVelocity(const double* ds, double* q, double *dq)
{
    bool flag;
    RVector dDis(6), qq(6), dDelta(6);
    RMatrix J(6,6), A(6,6), T(6,6);

    for(int i = 0; i<1; i++)
    {
        for(int i = 0; i < 6; i++)
        {
            dDis.value[i] = ds[i];              //the increment in sensor coordinate
            qq.value[i] = q[i];
        }
        GetJacobian(J, A, qq);
        RMatrix AJ = A * J;
        flag = RMatrix::RMatrixInv(AJ, T);

//        std::cout<<"1:"<<T.value[0][0]<<","<<T.value[0][1]<<","<<T.value[0][2]<<","<<T.value[0][3]<<","<<T.value[0][4]<<","<<T.value[0][5]<<std::endl;
//        std::cout<<"2:"<<T.value[1][0]<<","<<T.value[1][1]<<","<<T.value[1][2]<<","<<T.value[1][3]<<","<<T.value[1][4]<<","<<T.value[1][5]<<std::endl;
//        std::cout<<"3:"<<T.value[2][0]<<","<<T.value[2][1]<<","<<T.value[2][2]<<","<<T.value[2][3]<<","<<T.value[2][4]<<","<<T.value[2][5]<<std::endl;
//        std::cout<<"4:"<<T.value[3][0]<<","<<T.value[3][1]<<","<<T.value[3][2]<<","<<T.value[3][3]<<","<<T.value[3][4]<<","<<T.value[3][5]<<std::endl;
//        std::cout<<"5:"<<T.value[4][0]<<","<<T.value[4][1]<<","<<T.value[4][2]<<","<<T.value[4][3]<<","<<T.value[4][4]<<","<<T.value[4][5]<<std::endl;
//        std::cout<<"6:"<<T.value[5][0]<<","<<T.value[5][1]<<","<<T.value[5][2]<<","<<T.value[5][3]<<","<<T.value[5][4]<<","<<T.value[5][5]<<std::endl;
//        int a = 10;
    }

    if(flag/*!isnan(dDelta.value[0]) && !isnan(dDelta.value[1]) && !isnan(dDelta.value[2]) && !isnan(dDelta.value[3]) && !isnan(dDelta.value[4]) && !isnan(dDelta.value[5])*/)
    {
      dDelta = T * dDis; //the increment of joint angle
      memcpy(dq, dDelta.value, aubo_robot_namespace::ARM_DOF*sizeof(double));
      return true;
    }
    else
    {
        dq[0] = 0;dq[1] = 0;dq[2] = 0;dq[3] = 0;dq[4] = 0;dq[5] = 0;
        return false;
    }
}

void Util::EulerAngleToQuaternion(double *EulerAngle, double *orientation)
{
    double roll,pitch,yaw;

    roll = EulerAngle[0];
    pitch = EulerAngle[1];
    yaw = EulerAngle[2];

    orientation[0] = cos(roll/2.0)*cos(pitch/2)*cos(yaw/2.0)+sin(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0);
    orientation[1] = sin(roll/2.0)*cos(pitch/2)*cos(yaw/2.0)-cos(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0);
    orientation[2] = cos(roll/2.0)*sin(pitch/2)*cos(yaw/2.0)+sin(roll/2.0)*cos(pitch/2.0)*sin(yaw/2.0);
    orientation[3] = cos(roll/2.0)*cos(pitch/2)*sin(yaw/2.0)-sin(roll/2.0)*sin(pitch/2.0)*cos(yaw/2.0);
}
