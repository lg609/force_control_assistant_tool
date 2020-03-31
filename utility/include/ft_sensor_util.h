#ifndef FT_SENSOR_UTIL_H
#define FT_SENSOR_UTIL_H

#include "database.h"
#include "ft_sensor.h"
#include "rl/robot_interface.hpp"

#include <string>

using namespace ARAL;


class FTSensorUtil
{
public:
    FTSensorUtil();
    ~FTSensorUtil();
    /******** data base  operation ********/
    bool openDatabase(QString name);
    bool getFTSensorOffsetFromDB(Wrench &sensorOffset);
    bool getFTDBData(QString tableName, QString key, QString &value);
    bool getFTDBData(QString tableName, QString key, double *value);
    bool insertFTDBData(QString tableName, QString key, QString value);
    bool insertFTDBData(QString tableName, QString key, double value[]);
    void setFTSensorOffsetToDB(const Wrench & sensorOffset);
    bool setFTDBData(QString tableName, QString key, QString value);
    bool setFTDBData(QString tableName, QString key, QString value, int id);
    bool createFTDBTable(QString name);
    bool querydatabes();

    /******** display sensor data ********/
    inline void enableDisplaySenssorData(){display_sensor_data_ = true;}
    void disableDisplaySenssorData();
    void cleanDisplaySenssorData();
    inline int getPlotDataLength(){return plot_data_length;}
    int getDisplayDataExtremum(double extremum[], unsigned char flag);

    void getgetDisplayData(const Wrench& sensorData);

    DataBase *db_;
    std::vector<Wrench> m_sensor_data_display;

private:
    bool display_sensor_data_;
    const int plot_data_length = 600;  //the width of the plot area
};

#endif // FT_SENSOR_UTIL_H
