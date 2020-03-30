#include "ft_sensor_util.h"
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>

FTSensorUtil::FTSensorUtil(): display_sensor_data_(false)
{
    m_sensor_data_display.resize(0);
    db_ = new DataBase();
}

FTSensorUtil::~FTSensorUtil()
{
    if(db_ != NULL)
        delete db_;
}

void FTSensorUtil::getgetDisplayData(const Wrench& sensorData)
{
    //handle dispaly data
    if(display_sensor_data_)
    {
        Wrench plotWrench = sensorData;
        if(m_sensor_data_display.size() > plot_data_length * 1.2)
        {
            m_sensor_data_display.erase(m_sensor_data_display.begin());
            std::cout<<"The length of sensor data display vector is too long!"<<std::endl;
        }
        m_sensor_data_display.push_back(plotWrench);
    }
}

/******** display sensor data ********/
void FTSensorUtil::disableDisplaySenssorData()
{
    m_sensor_data_display.resize(0);
    display_sensor_data_ = false;
}

void FTSensorUtil::cleanDisplaySenssorData()
{
    m_sensor_data_display.resize(0);
}

int FTSensorUtil::getDisplayDataExtremum(double extremum[], unsigned char flag)
{
    int N = m_sensor_data_display.size();
    for(int i = 0; i < N; i++)
    {
        //min and max for force
        if(flag & 0x01)
        {
            if(m_sensor_data_display[i][0] < extremum[0])
                extremum[0] = m_sensor_data_display[i][0];
            else if(m_sensor_data_display[i][0] > extremum[1])
                extremum[1] = m_sensor_data_display[i][0];
        }
        if(flag & 0x02)
        {
            if(m_sensor_data_display[i][1] < extremum[0])
                extremum[0] = m_sensor_data_display[i][1];
            else if(m_sensor_data_display[i][1] > extremum[1])
                extremum[1] = m_sensor_data_display[i][1];
        }
        if(flag & 0x04)
        {
            if(m_sensor_data_display[i][2] < extremum[0])
                extremum[0] = m_sensor_data_display[i][2];
            else if(m_sensor_data_display[i][2] > extremum[1])
                extremum[1] = m_sensor_data_display[i][2];
        }

        //min and max for torque
        if(flag & 0x08)
        {
            if(m_sensor_data_display[i][3] < extremum[2])
                extremum[2] = m_sensor_data_display[i][3];
            else if(m_sensor_data_display[i][3] > extremum[3])
                extremum[3] = m_sensor_data_display[i][3];
        }
        if(flag & 0x10)
        {
            if(m_sensor_data_display[i][4] < extremum[2])
                extremum[2] = m_sensor_data_display[i][4];
            else if(m_sensor_data_display[i][4] > extremum[3])
                extremum[3] = m_sensor_data_display[i][4];
        }
        if(flag & 0x20)
        {
            if(m_sensor_data_display[i][5] < extremum[2])
                extremum[2] = m_sensor_data_display[i][5];
            else if(m_sensor_data_display[i][5] > extremum[3])
                extremum[3] = m_sensor_data_display[i][5];
        }
    }
    return N;
}


/******** data base  operation ********/
bool FTSensorUtil::openDatabase(QString name)
{
   return db_->openDataBase(name);
}

bool FTSensorUtil::getFTDBData(QString tableName, QString key, QString &value)
{
   return db_->queryValue(tableName, key, value);
}

bool FTSensorUtil::getFTDBData(QString tableName, QString key, double *value)
{
   return db_->queryValue(tableName, key, value);
}

bool FTSensorUtil::setFTDBData(QString tableName, QString key, QString value)
{
   return db_->updateByKey(tableName, key, value);
}

bool FTSensorUtil::setFTDBData(QString tableName, QString key, QString value, int id)
{
   return db_->updateByID(tableName, key, value, id);
}

bool FTSensorUtil::insertFTDBData(QString tableName, QString key, QString value)
{
   return db_->insert(tableName, key, value);
}

bool FTSensorUtil::insertFTDBData(QString tableName, QString key, double value[])
{
   return db_->insert(tableName, key, value);
}

bool FTSensorUtil::createFTDBTable(QString name)
{
   return db_->createTable(name);
}

bool FTSensorUtil::getFTSensorOffsetFromDB(Wrench &sensorOffset)
{
   double offset[SENSOR_DIMENSION] = {0};
   bool flag = getFTDBData("parameter", "offset", offset);
   sensorOffset[0] = offset[0];
   sensorOffset[1] = offset[1];
   sensorOffset[2] = offset[2];
   sensorOffset[3] = offset[3];
   sensorOffset[4] = offset[4];
   sensorOffset[5] = offset[5];
   std::cout<<"Fxoffset:"<<sensorOffset[0]<<" Fyoffset:"<<sensorOffset[1]<<" Fzoffset:"<<sensorOffset[2]
            <<" Txoffset:"<<sensorOffset[3]<<" Tyoffset:"<<sensorOffset[4]<<" Tzoffset:"<<sensorOffset[5]<<std::endl;
   return flag;
}

void FTSensorUtil::setFTSensorOffsetToDB(const Wrench & sensorOffset)
{
   double offset[SENSOR_DIMENSION] = {0};
   if(!getFTDBData("parameter", "offset", offset))
       insertFTDBData("parameter","offset", offset);
   else
   {
//        insertFTDBData("parameter","offset", s_sensor_offset);
       for(int i = 0; i < SENSOR_DIMENSION; i++)
           setFTDBData("parameter","offset", QString::number(sensorOffset[i]), i+1);
   }
}
