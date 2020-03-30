#include "FTSensorDataProcess.h"

double FTSensorDataProcess::s_sensor_data[6];
double FTSensorDataProcess::s_sensor_offset[6];
double FTSensorDataProcess::s_calibrationMeasurements[CALIBRATION_POS::POSE_Total][6];

bool FTSensorDataProcess::s_sensor_data_calibrated = false;

FTSensorDataProcess::FTSensorDataProcess():sensor_data_stable_(false),thread_live_(true),display_sensor_data_(false)
{
    ft_sensor_ = NULL;
    kws = NULL;
//    ati = NULL;
    sensor_type_.insert(std::pair<std::string, int>("optoforce", SENSOR_TYPE::OPTOFORCE));
    sensor_type_.insert(std::pair<std::string, int>("Robotiq", SENSOR_TYPE::ROBOTIQ));
    sensor_type_.insert(std::pair<std::string, int>("ATI", SENSOR_TYPE::ATI));
    sensor_type_.insert(std::pair<std::string, int>("KunWei", SENSOR_TYPE::KunWei));

    db_ = new DataBase();
    read_sensor_data_ = new std::thread(std::bind(&FTSensorDataProcess::obtainFTSensorData, this));
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

bool FTSensorDataProcess::sensorTypeSelect(std::string sensorType, std::string devName)
{
    if(ft_sensor_ != NULL && sensor_data_stable_)
    {
        ft_sensor_->uninitialFTSensor();
        sensor_data_stable_ = false;
    }

    connect_name_ = devName;
    //    if(sensor_type_[sensorType.toStdString()] == SENSOR_TYPE::OPTOFORCE)
    //    {
    //         ft_sensor_ = (OptoForceSensor*)&optoforce;
    //    }
    //    else if(sensor_type_[sensorType.toStdString()] == SENSOR_TYPE::ROBOTIQ)
    //    {
    //         ft_sensor_ = (RobotiqSensor*)&robotiq;
    //    }
    //    else if(sensor_type_[sensorType.toStdString()] == SENSOR_TYPE::ATI)
    //    {
    //        if(ati != NULL)
    //        {
    ////            delete ati;
    ////            ati = NULL;
    //        }
    //        ati = new ATISensor(connect_name_);
    //        ft_sensor_ = (ATISensor*)ati;
    //    }
    //    else
        if(sensor_type_[sensorType] == SENSOR_TYPE::KunWei)
        {
            if(kws != NULL)
            {
                kws->uninitialFTSensor();
                delete kws;
                kws = NULL;
            }
            if(devName == "")
                devName = "ttyUSB0";
            kws = new KunWeiSensor(devName);
            ft_sensor_ = (KunWeiSensor*)kws;
        }

    sensor_data_stable_ = ft_sensor_->initialFTSensor();
    m_sensor_data_display.resize(0);

    return sensor_data_stable_;
}


void FTSensorDataProcess::obtainCalibrationPos(int index)
{
    //obtain the sensor data in pose 1 2,3;
    sleep(1);       //wait for robot and sensor stable,1s
    for(int i = 0; i < 6; i++)
        s_calibrationMeasurements[index][i] = s_sensor_data[i] + s_sensor_offset[i];
    std::cout<<"s_calibrationMeasurements"<<s_calibrationMeasurements[index][0]<<","<<s_calibrationMeasurements[index][1]<<","
            <<s_calibrationMeasurements[index][2]<<","<<s_calibrationMeasurements[index][3]<<","<<s_calibrationMeasurements[index][4]<<","<<s_calibrationMeasurements[index][5]<<std::endl;
}


/******** get sensor data  information********/
void FTSensorDataProcess::obtainFTSensorData()
{
    float m_ftData[SENSOR_DIMENSION] = {0};
    float m_ftDataOld[SENSOR_DIMENSION] = {0};
    float R[SENSOR_DIMENSION], Q[SENSOR_DIMENSION];
    memcpy(R, new float[SENSOR_DIMENSION]{0.00087339583, 0.0010027597, 0.079814652, 0.000052964314, 0.000032147492, 0.000014059953}, sizeof(float)*SENSOR_DIMENSION);
    memcpy(Q, new float[SENSOR_DIMENSION]{1e-5,1e-5,1e-5,1e-5,1e-5,1e-5}, sizeof(float)*SENSOR_DIMENSION);

    float P[] = {1,1,1,1,1,1};
    float pp[SENSOR_DIMENSION] = {0};
    float K[SENSOR_DIMENSION] = {0};
    float ftData[SENSOR_DIMENSION];
    // One problem: sometimes when unload the payload, the output will still be hold for about 5s. -> check for this.
    //    std::string name[] = {"ForceX","ForceY","ForceZ","TorqueX","TorqueY","TorqueZ"};

    while(thread_live_)
    {
        QTime current_time1 = QTime::currentTime();
        if(sensor_data_stable_)
        {
            ft_sensor_->obtainFTSensorData(m_ftData);
            //std::cout<<m_ftData[0]<<m_ftData[1]<<m_ftData[2]<<m_ftData[3]<<m_ftData[4]<<m_ftData[5]<<std::endl;
            double m_range[6] = {0};
            ft_sensor_->getFTSensorRange(m_range);
            for(int i = 0; i < SENSOR_DIMENSION; i++)
            {
                if(m_ftData[i] > m_range[i])
                {
                    std::cout<<"Force Exceed the Range!";
                    //ANOTHER WINDOW
//                    emit signal_sensor_over_range("Force Exceed the Sensor Range!");
                }

            }
            if(s_sensor_data_calibrated)
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
            }

            //        mlock_.lock();
            for(int i = 0; i < 6; i++)
            {
                s_sensor_data[i] = m_ftData[i];
            }

            //handle dispaly data
            if(display_sensor_data_)
            {
                Wrench plotWrench;
                memcpy(plotWrench.data, s_sensor_data, sizeof(double) * 6);
                if(m_sensor_data_display.size() > plot_data_length * 1.2)
                {
                    m_sensor_data_display.erase(m_sensor_data_display.begin());
                    std::cout<<"The length of sensor data display vector is too long!"<<std::endl;
                }
                m_sensor_data_display.push_back(plotWrench);
            }
        }

        QTime current_time2 = QTime::currentTime();
        int sleepT = current_time2.msec() - current_time1.msec();
        if(sleepT < 0) sleepT += 1000;
        double ts = (control_period_-sleepT);
        if(ts < 0)
            ts = 0;
        usleep(ts*1000);
    }
}



/******** display sensor data ********/
void FTSensorDataProcess::disableDisplaySenssorData()
{
    m_sensor_data_display.resize(0);
    display_sensor_data_ = false;
}

int FTSensorDataProcess::getDisplayDataExtremum(double extremum[], byte flag)
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
bool FTSensorDataProcess::openDatabase(QString name)
{
    return db_->openDataBase(name);
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

bool FTSensorDataProcess::getFTSensorOffsetFromDB(double sensorOffset[])
{
    double offset[SENSOR_DIMENSION] = {0};
    bool flag = getFTDBData("parameter", "offset",offset);
    s_sensor_offset[0] = offset[0];s_sensor_offset[1] = offset[1];s_sensor_offset[2] = offset[2];
    s_sensor_offset[3] = offset[3];s_sensor_offset[4] = offset[4];s_sensor_offset[5] = offset[5];
    std::cout<<"Fxoffset:"<<s_sensor_offset[0]<<" Fyoffset:"<<s_sensor_offset[1]<<" Fzoffset:"<<s_sensor_offset[2]<<" Txoffset:"<<s_sensor_offset[3]<<" Tyoffset:"<<s_sensor_offset[4]<<" Tzoffset:"<<s_sensor_offset[5];
    memcpy(sensorOffset, offset, sizeof(double)*SENSOR_DIMENSION);
    return flag;
}

void FTSensorDataProcess::setFTSensorOffsetToDB()
{
    double offset[SENSOR_DIMENSION] = {0};
    if(!getFTDBData("parameter", "offset", offset))
        insertFTDBData("parameter","offset", offset);
    else
    {
//        insertFTDBData("parameter","offset", s_sensor_offset);
        for(int i = 0; i < SENSOR_DIMENSION; i++)
            setFTDBData("parameter","offset", QString::number(s_sensor_offset[i]), i+1);
    }
}


