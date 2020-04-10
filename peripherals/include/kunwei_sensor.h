#ifndef KUNWEI_SENSOR_H
#define KUNWEI_SENSOR_H

#include "ft_sensor.h"
#include "serial_port.h"

#include <unistd.h>
#include <thread>
#include <iostream>
//#include <queue>
#include <sys/timeb.h>

#define REC_DATA_LEN  60

typedef enum{
    INIT = 0,
    CONTINUOUS_MODE,
    REQUEST_MODE,
    CLEAR,
}SENSOR_COMMAND;


typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;
typedef int SOCKET_HANDLE;

typedef struct KunWeiResponseStruct
{
    int fd;
    uint32 ft_sequence;
    uint32 status;
    float FTData[6];
    int force_available;
    int mode;
    int running;
    pthread_t tid;
} KunWeiResponse;


#define NEW_KUNWEI_SENSOR

class KunWeiSensor: public FTSensor
{
public:
    KunWeiSensor(std::string port_name = "ttyUSB0");
    //!
    ~KunWeiSensor();

    //!
    bool obtainFTSensorData(float m_ftData[SENSOR_DIMENSION]);
    //!
    bool initialFTSensor();
    //!
    bool uninitialFTSensor();
    //!
    void getFTSensorRange(double range[]);
    //!
    bool readConfig();
    //!
    virtual bool getCalibStauts(){}

private:
#ifdef NEW_KUNWEI_SENSOR
    typedef void* KWRHandle;
    int kwr_read_continous_request();
    int kwr_ft_zero();
    KunWeiResponse *dev;
#else
    int writeCommand(SENSOR_COMMAND command);
    int readSensor(unsigned char m_rec_data[]);
    void handleSensorData();
    void handleSensorData2();
#endif


private:
    std::string port_name_;
    SerialPort *serial_port_;
    KunWeiResponse ftResponse;
    bool read_success_;
    bool sensor_running_;
    int m_serial_fd_;
    int baud_level_;
    char command_head_;
    std::thread read_sensor_data_;
};
void data2force(char* data, float* Force);

#endif // KUNWEISENSOR_H
