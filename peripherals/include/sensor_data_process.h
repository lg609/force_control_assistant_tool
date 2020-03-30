#ifndef FTSENSORDATAPROCESS_H
#define FTSENSORDATAPROCESS_H

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
#include "kdl/frames.hpp"

using namespace  AUBO;

enum CALIBRATION_POS
{
    POSE_X = 0,
    POSE_Y,
    POSE_Z,
    POSE_Total
};

class FTSensorDataProcess
{
public:
    FTSensorDataProcess();
    ~FTSensorDataProcess();
    
    //! select sensor
    bool sensorTypeSelect(std::string sensorType, std::string devName = "");
    //!
    inline void setConnectName(std::string connectName){connect_name_ = connectName;}

    /******** data base  operation ********/
//    bool openDatabase(std::string name);
//    bool getFTSensorOffsetFromDB(double sensorOffset[]);
//    bool getFTDBData(std::string tableName, std::string key, std::string &value);
//    bool getFTDBData(std::string tableName, std::string key, double *value);
//    bool insertFTDBData(std::string tableName, std::string key, std::string value);
//    bool insertFTDBData(std::string tableName, std::string key, double value[]);
//    void setFTSensorOffsetToDB();
//    bool setFTDBData(std::string tableName, std::string key, std::string value);
//    bool setFTDBData(std::string tableName, std::string key, std::string value, int id);
//    bool createFTDBTable(std::string name);
//    bool querydatabes();

    /******** display sensor data ********/
    inline void enableDisplaySenssorData(){display_sensor_data_ = true;}
    void disableDisplaySenssorData();
    inline int getPlotDataLength(){return plot_data_length;}
    int getDisplayDataExtremum(double extremum[], byte flag);

    /******** get sensor data  information********/
    //! poses for calibration
    void obtainCalibrationPos(int index);
    void obtainFTSensorData();
    inline static Wrench getSensorData(){return s_sensor_data;}
    inline bool getSensorCalibrateStatus(){return s_sensor_data_calibrated;}
    inline void setSensorCalibrateStatus(bool calibrated){s_sensor_data_calibrated = calibrated;}

public:
//    DataBase *db_;
    static Wrench s_calibrationMeasurements[CALIBRATION_POS::POSE_Total];
    static Wrench s_sensor_data;
    static Wrench s_sensor_offset;
    static bool s_sensor_data_calibrated;
    std::vector<Wrench > m_sensor_data_display;

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

    bool display_sensor_data_;
    const int plot_data_length = 600;  //the width of the plot area
    const int control_period_ = 5;  //the width of the plot area

};

#endif // FTSENSORDATAPROCESS_H
