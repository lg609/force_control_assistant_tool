#ifndef FTSENSORDATAPROCESS_H
#define FTSENSORDATAPROCESS_H

#include <QObject>
#include <QtSql>
#include <QSqlDriver>
#include <QSqlDatabase>
#include <thread>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/shm.h>
#include <iostream>
#include <vector>

#include "ft_sensor.h"
#include "kunwei_sensor.h"
#include "../utility/include/database.h"

//#include "model/frames.hpp"
//using namespace  AUBO;

enum CALIBRATION_POS
{
    POSE_X = 0,
    POSE_Y,
    POSE_Z,
    POSE_Total
};


class FTSensorDataProcess:public QObject
{
    Q_OBJECT
public:
    FTSensorDataProcess();
    ~FTSensorDataProcess();
    
    //! select sensor
    bool sensorTypeSelect(std::string sensorType, std::string devName = "");
    //!
    inline void setConnectName(std::string connectName){connect_name_ = connectName;}

    /******** data base  operation ********/
    bool openDatabase(QString name);
    bool getFTSensorOffsetFromDB(double sensorOffset[]);
    bool getFTDBData(QString tableName, QString key, QString &value);
    bool getFTDBData(QString tableName, QString key, double *value);
    bool insertFTDBData(QString tableName, QString key, QString value);
    bool insertFTDBData(QString tableName, QString key, double value[]);
    void setFTSensorOffsetToDB();
    bool setFTDBData(QString tableName, QString key, QString value);
    bool setFTDBData(QString tableName, QString key, QString value, int id);
    bool createFTDBTable(QString name);
    bool querydatabes();

    /******** display sensor data ********/
    inline void enableDisplaySenssorData(){display_sensor_data_ = true;}
    void disableDisplaySenssorData();
    inline int getPlotDataLength(){return plot_data_length;}
    int getDisplayDataExtremum(double extremum[], byte flag);

    /******** get sensor data  information********/
    //! poses for calibration
    void obtainCalibrationPos(int index);
    void obtainFTSensorData();
    inline static double* getSensorData(){return s_sensor_data;}
    inline bool getSensorCalibrateStatus(){return s_sensor_data_calibrated;}
    inline void setSensorCalibrateStatus(bool calibrated){s_sensor_data_calibrated = calibrated;}

signals:
    void signal_sensor_over_range(QString);

public:
    DataBase *db_;
    static double s_calibrationMeasurements[CALIBRATION_POS::POSE_Total][6];
    static double s_sensor_data[6];
    static double s_sensor_offset[6];
    static bool s_sensor_data_calibrated;
    std::vector<Wrench> m_sensor_data_display;

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
