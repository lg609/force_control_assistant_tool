#include "sensor_data_process.h"
//using namespace AUBO;

Wrench FTSensorDataProcess::s_sensor_data;
Wrench FTSensorDataProcess::s_sensor_offset;
Wrench FTSensorDataProcess::s_calibrationMeasurements[CALIBRATION_POS::POSE_Total];

bool FTSensorDataProcess::s_sensor_data_calibrated = false;

FTSensorDataProcess::FTSensorDataProcess():sensor_data_stable_(false),thread_live_(true)
{
    ft_sensor_ = NULL;
    kws = NULL;
//    ati = NULL;
    sensor_type_.insert(std::pair<std::string, int>("optoforce", SENSOR_TYPE::OPTOFORCE));
    sensor_type_.insert(std::pair<std::string, int>("Robotiq", SENSOR_TYPE::ROBOTIQ));
    sensor_type_.insert(std::pair<std::string, int>("ATI", SENSOR_TYPE::ATI));
    sensor_type_.insert(std::pair<std::string, int>("KunWei", SENSOR_TYPE::KunWei));

    read_sensor_data_ = new std::thread(std::bind(&FTSensorDataProcess::obtainFTSensorData, this));
    std::cout<<"Start the read sensor thread!"<<std::endl;
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
//        if(devName == "")
            devName = "ttyUSB0";
        kws = new KunWeiSensor(devName);
        ft_sensor_ = (KunWeiSensor*)kws;
    }

    sensor_data_stable_ = ft_sensor_->initialFTSensor();


    return sensor_data_stable_;
}


void FTSensorDataProcess::obtainCalibrationPos(int index)
{
    //obtain the sensor data in pose 1 2,3;
    sleep(1);       //wait for robot and sensor stable,1s
    s_calibrationMeasurements[index] = s_sensor_data + s_sensor_offset;
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
            s_sensor_data[2] -= 10;
        }
        usleep(1*1000);
    }
}




