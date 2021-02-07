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
#include <mutex>
#include <thread>

#include <string>
#include <iostream>

#include "aral/robot_library_interface.hpp"
#include <aubo_driver/aubo_driver.h>
using namespace arcs::aubo_driver;


#define CALIBRATION_POS_TOTAL 3
#define INERTIA_DIM 9

typedef ARAL::RLWrench Wrench;

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
typedef RLJntArray JointArray;
typedef RLPose CartArray;


class RobotControl
{
public:
    explicit RobotControl(const std::string& model, const std::string& robot_ip, const int& port = 30001);
    //!
    ~RobotControl();
    //!
    void updateRobotStatus();
    //!
    void updateRobotGoal();
    //!
    int getRobotOutput();
    //!
    void getRobotEndWrench(double* wrench);
    //!
    void setControlPeriod(const int& period);
    //!
    void setMaxTranSpeed(double vel);
    //!
    void setMaxRotSpeed(double rot);

    /******** Force Control function ********/
    int startForceControl();

    int stopForceControl();

    int forceControlThread();

    void getRtdeData(AuboDriver *m_instance);

    /**
     * @brief 得到末端传感器在三种不同位姿下的测量值
     * @param index：指定不同的位姿
     */
    void obtainCalibrationPos(const int& index);

    //!
    void setSelectionVector(const int* vec);
    // 设置传感器最小响应阈值
    void setEndFTSensorThreshold(double data[SENSOR_DIMENSION]);
    // 获得传感器最小响应阈值
    std::vector<double> getEndFTSensorThreshold(){return end_ft_threshold_;}
    // 设置传感器最大响应阈值
    void setEndFTSensorLimit(double data[SENSOR_DIMENSION]);
    // 获得传感器最大响应阈值
    std::vector<double> getEndFTSensorLimit(){return end_ft_limit_;}
    // 设置末端刚度系数
    int setCartStiffness(double data[CARTESIAN_FREEDOM]);
    // 获得末端刚度系数
    std::vector<double> getCartStiffness(){return cart_stiffness_;}
    // 设置末端阻尼系数
    int setCartDamp(double data[CARTESIAN_FREEDOM]);
    // 获得末端阻尼系数
    std::vector<double> getCartDamp(){return cart_damp_;}
    // 设置末端质量系数
    int setCartMass(double data[CARTESIAN_FREEDOM]);
    // 获得末端质量系数
    std::vector<double> getCartMass(){return cart_mass_;}
    //!
    void setOverEstimatedDis(const double dis);
    //!
    void setGoalWrench(const double* wrench);

    std::vector<double> getSensorData()
    {
        mutex_.lock();
        std::vector<double> data = sensor_data_;        //此处需要加互斥锁进行数据拷贝
        mutex_.unlock();
        return data;
    }

    bool getSensorCalibrateStatus(){return sensor_calibrated_;}


    /******** Calibration function ********/
    int moveToTargetPose(int index);

    //!
    int calibrateFTSensor(FtSensorCalibrationResult &result);


    /******** Control Parameter function ********/
    void initalControlPara();
    //!
    void createParaTable();
    //!
    void updateControlPara(const double& value, const int& index, const std::string& type);
    //!
    void setToolInertia(const double data[INERTIA_DIM]);
    //!
    void setToolPose(double data[SENSOR_DIMENSION]);
    //!
    void setFTSensorPose(double data[SENSOR_DIMENSION]);
    //!

    /******************** Admittance Control ********************/
    // 使能奇异防护功能
    void enableConstraints(const bool& flag);
    //!
    void setConstrainPara(const double* value);    //singularity

    void enableAdmittanceControl();
    //!
    void disableAdmittanceControl();
    //!
    void updateAdmittancePIDPara(const double& value, const int& index);
    //设置拖动的模式
    void setDragMode(unsigned int value);
    //!
    void setForceControlMode(unsigned int mode);
    //!
    void setThreadMode(unsigned int mode);
    //!
    void setControlSpace(unsigned int value);
    //!
    void setSensorFilter(const double value);    // value: 0-1
    //!
    void setControlFilter(const double value);
    //设置传感器标定对应的机械臂位置
    void setCalibrationPose(double data[SENSOR_DIMENSION], int index);

private:
    ARAL::RLIntfacePtr aral_interface_;  //机器人算法库实例

    std::thread force_control_Thread_;  //力控线程
    bool enable_thread_;
    int control_period_;  // 控制周期 unit: ms

    AuboDriver* aubo_driver;    //实时SDK接口

    std::map<std::string, int> para_table_;

/****************控制使用变量************************/
    std::vector<double> cart_mass_;
    std::vector<double> cart_damp_;
    std::vector<double> cart_stiffness_;
    CartArray tool_pose_;
    CartArray ft_sensor_pose_;
    std::vector<int> selection_vector_;
    CartArray constraint_para_;
    std::vector<double> end_ft_threshold_;
    std::vector<double> end_ft_limit_;
    Wrench force_of_end_;
    Wrench goal_wrench_;
    std::vector<double> cmd_joint_pos_;

    std::vector<double> tool_inertia_;                          //工具惯性参数
    FtSensorCalibrationResult ft_sensor_calib_res_;             //传感器标定结果
    Wrench s_calibrationMeasurements[CALIBRATION_POS_TOTAL];    //传感器标定对应的3个传感器数值
    JointArray calibration_poses_[CALIBRATION_POS_TOTAL];       //传感器标定对应的3个机器人位置
    bool sensor_calibrated_;                                    //传感器是否标定
    std::vector<double> sensor_offset_;                         //传感器偏置

    //对于在多个线程中使用的变量,需要加互斥锁
    std::mutex mutex_;
    std::vector<double> sensor_data_;                           //传感器数据
    std::vector<double> cur_joint_pos_;                         //机械臂当前关节位置
    std::vector<double> cur_joint_vel_;                         //机械臂当前关节速度
};

#endif // ROBOTCONTROL_H
