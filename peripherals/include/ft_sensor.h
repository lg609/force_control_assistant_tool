#ifndef AUBO_FT_SENSOR_H
#define AUBO_FT_SENSOR_H

#include <string>

#define SENSOR_DIMENSION 6
#define GRAVITY_ACC_  9.81

enum SENSOR_TYPE
{
    OPTOFORCE = 0,
    ROBOTIQ,
    ATI,
    KunWei
};

typedef struct _wrench
{
    double data[6];
    double& operator()(int i){return data[i];}
    double operator()(int i)const{return data[i];}
    double& operator[](int i){return data[i];}
    double operator[](int i)const{return data[i];}
    struct _wrench operator+(const struct _wrench & lhs)
    {
        struct _wrench tmp;
        for(int i = 0; i < 6; i++)
        tmp.data[i] = lhs.data[i] + (*this)[i];

        return tmp;
    }
}Wrench;

class FTSensor
{
public:
    FTSensor();
    //!
    ~FTSensor();

    //!
    virtual bool obtainFTSensorData(float m_ftData[SENSOR_DIMENSION]) = 0;
    //!
    virtual bool initialFTSensor() = 0;
    //!
    virtual void getFTSensorRange(double range[]) = 0;
    //!
    virtual bool uninitialFTSensor() = 0;
    //!
    virtual bool readConfig() = 0;
    //!
    virtual bool getCalibStauts() = 0;

private:
    std::string sensor_name_;

    bool calib_status_;

    double offset_[SENSOR_DIMENSION];


};

#endif // AUBO_FT_SENSOR_H
