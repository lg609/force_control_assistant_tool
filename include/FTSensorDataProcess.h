#ifndef FTSENSORDATAPROCESS_H
#define FTSENSORDATAPROCESS_H
#include <QObject>
#include <QtSql>
#include <QSqlDriver>
#include <QSqlDatabase>
#include <thread>
#include <boost/bind.hpp>
#include <unistd.h>
#include <math.h>
#include <iostream>

#include "../ftsensor/include/ftsensor.h"
#include "../ftsensor/include/OptoForceSensor.h"
#include "../ftsensor/include/RobotiqSensor.h"
#include "../ftsensor/include/ATISensor.h"
#include "../utility/include/database.h"

enum PARAMATER_TYPE
{
    SENSITIVITY = 0,
    DAMP,
    STIFFNESS,
    THRESHOLD,
    LIMIT,
    POS
};
enum CONTROL_MODE
{
    VELOCITY = 0,
    ACCLERATION
};

enum CALCULATE_METHOD
{
    JACOBIAN = 0,
    IK
};

enum DRAG_MODE
{
    POSITION = 0,
    ORI,
    POSE
};

class FTSensorDataProcess:public QObject
{
    Q_OBJECT
public:
    FTSensorDataProcess();
    ~FTSensorDataProcess();
    
    void obtainCalibrationPos(int index);
    void getFTSensorData();
    bool sensorTypeSelect(QString sensorType);
    void setFTSensorOffsetToDB();
    bool getFTSensorOffsetFromDB();


    bool getFTDBData(QString tableName, QString key, QString &value);
    bool getFTDBData(QString tableName, QString key, double *value);
    bool insertFTDBData(QString tableName, QString key, QString value);
    bool insertFTDBData(QString tableName, QString key, double value[]);
    bool setFTDBData(QString tableName, QString key, QString value);
    bool setFTDBData(QString tableName, QString key, QString value, int id);
    bool createFTDBTable(QString name);
    bool querydatabes();
    bool openDatabase(QString name);
    
signals:
    void signal_sensor_over_range(QString);

public:
    static double calibrationMessurement_[3][6];
    DataBase *db_;
    static int s_dragMode;
    static int s_calculateMethod;
    static int s_controlModel;
    static int s_bufferSizeLimit;
    static int s_filter1;
    static int s_filter2;
    static double s_sensitivity[SENSOR_DIMENSION];
    static double s_damp[SENSOR_DIMENSION];
    static double s_stiffness[SENSOR_DIMENSION];
    //static double s_threshold[SENSOR_DIMENSION];
    static double s_pos[6];

    static double s_sensor_data[SENSOR_DIMENSION];
    static double s_sensor_offset[SENSOR_DIMENSION];
    static bool sensor_data_calibrated_;

private:
    std::thread* read_sensor_data_;
    std::map<std::string, int> sensor_type_;
    bool sensor_data_stable_;
    bool thread_live_;

    FTSensor* ft_sensor_;
    OptoForceSensor optoforce;       //optoforce
    RobotiqSensor robotiq;           //robotiq
    ATISensor ati;           //robotiq
};

#endif // FTSENSORDATAPROCESS_H
