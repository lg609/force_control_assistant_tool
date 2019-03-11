#include "FTSensorDataProcess.h"

int FTSensorDataProcess::s_dragMode = 0;
int FTSensorDataProcess::s_calculateMethod = 0;
int FTSensorDataProcess::s_controlModel = 0;
int FTSensorDataProcess::s_bufferSizeLimit = 42;
int FTSensorDataProcess::s_filter1 = 0;
int FTSensorDataProcess::s_filter2 = 0;
double FTSensorDataProcess::s_sensitivity[SENSOR_DIMENSION] = {0};
double FTSensorDataProcess::s_damp[SENSOR_DIMENSION] = {0};
double FTSensorDataProcess::s_stiffness[SENSOR_DIMENSION] = {0};
double FTSensorDataProcess::s_threshold[SENSOR_DIMENSION] = {0};
double FTSensorDataProcess::s_limit[SENSOR_DIMENSION] = {0};
double FTSensorDataProcess::s_sensor_data[SENSOR_DIMENSION] = {0};
double FTSensorDataProcess::s_sensor_offset[SENSOR_DIMENSION] = {0};

double FTSensorDataProcess::calibrationMessurement_[3][6] = {0};
bool FTSensorDataProcess::sensor_data_calibrated_ = false;

FTSensorDataProcess::FTSensorDataProcess():sensor_data_stable_(false),thread_live_(true)
{
    ft_sensor_ = NULL;
    sensor_type_.insert(std::pair<std::string, int>("optoforce", SENSOR_TYPE::OPTOFORCE));
    sensor_type_.insert(std::pair<std::string, int>("Robotiq", SENSOR_TYPE::ROBOTIQ));
    sensor_type_.insert(std::pair<std::string, int>("ATI", SENSOR_TYPE::ATI));

    db_ = new DataBase();
    read_sensor_data_ = new std::thread(boost::bind(&FTSensorDataProcess::getFTSensorData, this));
    qDebug()<<"Start the read sensor thread!";
}

FTSensorDataProcess::~FTSensorDataProcess()
{
    thread_live_ = false;
    usleep(10*1000);
    read_sensor_data_->join();
    if(read_sensor_data_ != NULL)
        delete read_sensor_data_;
//    if(ft_sensor_ != NULL)
//        delete ft_sensor_;
    if(db_ != NULL)
        delete db_;
}

bool FTSensorDataProcess::sensorTypeSelect(QString sensorType)
{
    if(ft_sensor_ != NULL && sensor_data_stable_)
    {
        ft_sensor_->uninitialFTSensor();
        sensor_data_stable_ = false;
    }

    if(sensor_type_[sensorType.toStdString()] == SENSOR_TYPE::OPTOFORCE)
    {
         ft_sensor_ = (OptoForceSensor*)&optoforce;
    }
    else if(sensor_type_[sensorType.toStdString()] == SENSOR_TYPE::ROBOTIQ)
    {
         ft_sensor_ = (RobotiqSensor*)&robotiq;
    }
    else if(sensor_type_[sensorType.toStdString()] == SENSOR_TYPE::ATI)
    {
         ft_sensor_ = (ATISensor*)&ati;
    }

    sensor_data_stable_ = ft_sensor_->initialFTSensor();

    return sensor_data_stable_;
}

bool FTSensorDataProcess::openDatabase(QString name)
{
    return db_->openDataBase(name);
}

void FTSensorDataProcess::obtainCalibrationPos(int index)
{
    //obtain the sensor data in pose 1 2,3;
    sleep(1);//wait for robot and sensor stable,1s
    if(index == 1)
    {
        for(int i = 0; i < 6; i++)
        {
            calibrationMessurement_[0][i] = s_sensor_data[i];
        }
        //calibrationMessurement_
    }
    else if(index == 2)
    {
        for(int i = 0; i < 6; i++)
        {
            calibrationMessurement_[1][i] = s_sensor_data[i];
        }
        //calibrationMessurement_
    }
    else if(index == 3)
    {
        for(int i = 0; i < 6; i++)
        {
            calibrationMessurement_[2][i] = s_sensor_data[i];
        }
    }
    std::cout<<"s_sensor_data"<<s_sensor_data[0]<<","<<s_sensor_data[1]<<","<<s_sensor_data[2]<<","<<s_sensor_data[3]<<","<<s_sensor_data[4]<<","<<s_sensor_data[5]<<std::endl;
}

bool FTSensorDataProcess::getFTSensorOffsetFromDB()
{
    bool flag = getFTDBData("parameter", "offset", s_sensor_offset);
    std::cout<<"Fxoffset:"<<s_sensor_offset[0]<<" Fyoffset:"<<s_sensor_offset[1]<<" Fzoffset:"<<s_sensor_offset[2]<<" Txoffset:"<<s_sensor_offset[3]<<" Tyoffset:"<<s_sensor_offset[4]<<" Tzoffset:"<<s_sensor_offset[5];
    return flag;
}

void FTSensorDataProcess::setFTSensorOffsetToDB()
{
    double offset[SENSOR_DIMENSION] = {0};
    if(!getFTDBData("parameter", "offset", offset))
        insertFTDBData("parameter","offset", offset);
    else
    {
        for(int i = 0; i < SENSOR_DIMENSION; i++)
        {
            setFTDBData("parameter","offset", QString::number(s_sensor_offset[i]), i);
            getFTDBData("parameter", "offset", offset);
        }
    }
}

void FTSensorDataProcess::getFTSensorData()
{
    float m_ftData[SENSOR_DIMENSION] = {0};
    float m_ftDataOld[SENSOR_DIMENSION] = {0};
    float R[SENSOR_DIMENSION], Q[SENSOR_DIMENSION];
    if(s_controlModel == CONTROL_MODE::ACCLERATION)
    {
        memcpy(R, new float[SENSOR_DIMENSION]{0.00087339583, 0.0010027597, 0.079814652, 0.000052964314, 0.000032147492, 0.000014059953}, sizeof(float)*SENSOR_DIMENSION);
        memcpy(Q, new float[SENSOR_DIMENSION]{1e-5,1e-5,1e-5,1e-5,1e-5,1e-5}, sizeof(float)*SENSOR_DIMENSION);
    }
    else
    {
        memcpy(R, new float[SENSOR_DIMENSION]{0.00087339583, 0.0010027597, 0.079814652, 0.000052964314, 0.000032147492, 0.000014059953}, sizeof(float)*SENSOR_DIMENSION);
        memcpy(Q, new float[SENSOR_DIMENSION]{1e-6,1e-6,1e-6,1e-6,1e-6,1e-6}, sizeof(float)*SENSOR_DIMENSION);
    }
    float P[] = {1,1,1,1,1,1};
    float pp[SENSOR_DIMENSION] = {0};
    float K[SENSOR_DIMENSION] = {0};
    // Loop Period is about 11ms
    // One problem: sometimes when unload the payload, the output will still be hold for about 5s. -> check for this.

    //    std::string name[] = {"ForceX","ForceY","ForceZ","TorqueX","TorqueY","TorqueZ"};

    while(thread_live_)
    {
        QTime current_time1 = QTime::currentTime();
        if(sensor_data_stable_)
        {
            ft_sensor_->obtainFTSensorData(m_ftData);
            //qDebug()<<m_ftData[0]<<m_ftData[1]<<m_ftData[2]<<m_ftData[3]<<m_ftData[4]<<m_ftData[5];
            if(sensor_data_calibrated_)
            {
                for(int i = 0; i < SENSOR_DIMENSION; i++)
                {
                    pp[i] = P[i] + Q[i];
                    K[i] = pp[i] / (pp[i] + R[i]);
                    m_ftData[i] = m_ftDataOld[i] + K[i] * (m_ftData[i] - m_ftDataOld[i]);
                    P[i] = (1 - K[i]) * pp[i];
                    m_ftDataOld[i] = m_ftData[i];
                    m_ftData[i] -= s_sensor_offset[i];    //Subtract the offset force and torque
                }

                //set the threshold value
                for(int i = 0; i < SENSOR_DIMENSION; i++)
                {
                    //threshold + max limit
                    if(abs(m_ftData[i]) < s_threshold[i])
                        m_ftData[i] = 0;
                    else if (m_ftData[i] > 0)
                    {
                        m_ftData[i] -= s_threshold[i];
                        if(m_ftData[i] > s_limit[i])
                            m_ftData[i] = s_limit[i];
                    }
                    else
                    {
                        m_ftData[i] += s_threshold[i];
                        if(m_ftData[i] < -s_limit[i])
                            m_ftData[i] = -s_limit[i];
                    }
                }
            }

            //        mlock_.lock();
            for(int i = 0; i < 6; i++)
            {
                s_sensor_data[i] = m_ftData[i];
            }

        }
        else
        {
            //            qDebug()<<"Fx:"<<m_ftData[0]<<" Fy:"<<m_ftData[1]<<" Fz:"<<m_ftData[2]<<" Tx:"<<m_ftData[3]<<" Ty:"<<m_ftData[4]<<" Tz:"<<m_ftData[5];
        }

#ifdef  MEMCACHED_SHARE
        char str[256];
        sprintf(str, "%lf", m_ftData[0]);
        mc.Insert("ForceX", str);
        sprintf(str, "%lf", m_ftData[1]);
        mc.Insert("ForceY", str);
        sprintf(str, "%lf", m_ftData[2]);
        mc.Insert("ForceZ", str);
        sprintf(str, "%lf", m_ftData[3]);
        mc.Insert("TorqueX", str);
        sprintf(str, "%lf", m_ftData[4]);
        mc.Insert("TorqueY", str);
        sprintf(str, "%lf", m_ftData[5]);
        mc.Insert("TorqueZ", str);
        qDebug()<<m_ftData[0]<<m_ftData[1]<<m_ftData[2]<<m_ftData[3]<<m_ftData[4]<<m_ftData[5];

        //        for(int i = 0; i < 6; i++)
        //        {
        //            sprintf(str, "%lf", m_ftData[i]);
        //            mc.Insert("ForceX", str);
        //        }
#endif

        QTime current_time2 = QTime::currentTime();
        int sleepT = current_time2.msec() - current_time1.msec();
        if(sleepT < 0) sleepT += 1000;
        double ts = (5-sleepT);
        if(ts < 0)
            ts = 0;
        usleep(ts*1000);
    }

}

bool FTSensorDataProcess::getFTDBData(QString tableName, QString key, QString &value)
{
    return db_->queryValue(tableName, key, value);
}

bool FTSensorDataProcess::getFTDBData(QString tableName, QString key, double *value)
{
    return db_->queryValue(tableName, key, value);
}

bool FTSensorDataProcess::setFTDBData(QString tableName, QString key, QString value)
{
    return db_->updateByKey(tableName, key, value);
}

bool FTSensorDataProcess::setFTDBData(QString tableName, QString key, QString value, int id)
{
    return db_->updateByID(tableName, key, value, id);
}

bool FTSensorDataProcess::insertFTDBData(QString tableName, QString key, QString value)
{
    return db_->insert(tableName, key, value);
}

bool FTSensorDataProcess::insertFTDBData(QString tableName, QString key, double value[])
{
    return db_->insert(tableName, key, value);
}

bool FTSensorDataProcess::createFTDBTable(QString name)
{
    return db_->createTable(name);
}
