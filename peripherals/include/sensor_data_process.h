#ifndef FT_SENSOR_DATA_PROCESS_H
#define FT_SENSOR_DATA_PROCESS_H

#include <thread>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/shm.h>
#include <iostream>
#include <vector>
#include <map>

#include "ft_sensor.h"
#include "kunwei_sensor.h"


enum CALIBRATION_POS
{
    POSE_X = 0,
    POSE_Y,
    POSE_Z,
    POSE_Total
};


/*class FTSensorDataProcess
{
public:
    FTSensorDataProcess();
    ~FTSensorDataProcess();
    
    /******** get sensor data  information*******
    //! poses for calibration
    void obtainCalibrationPos(int index);
    void obtainFTSensorData();
    inline static Wrench getSensorData(){return s_sensor_data;}
    inline bool getSensorCalibrateStatus(){return s_sensor_data_calibrated;}
    inline void setSensorCalibrateStatus(bool calibrated){s_sensor_data_calibrated = calibrated;}

public:
    static Wrench s_sensor_data;
    static Wrench s_sensor_offset;
    static bool s_sensor_data_calibrated;


private:
    std::thread* read_sensor_data_;
    std::map<std::string, int> sensor_type_;
    bool sensor_data_stable_;
    bool thread_live_;

    FTSensor* ft_sensor_;
//    OptoForceSensor optoforce;       //optoforce
//    RobotiqSensor robotiq;           //robotiq
//    ATISensor *ati;           //ATI
    KunWeiSensor *kws;           //KWS
    std::string connect_name_;

    const int control_period_ = 0.005;  //the width of the plot area
};*/

#endif // FT_SENSOR_DATA_PROCESS_H
