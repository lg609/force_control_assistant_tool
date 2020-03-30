#ifndef ROBOTIQ_SENSOR
#define ROBOTIQ_SENSOR

#include "ftsensor.h"
#include "../../utility/include/rq_int.h"
#include "../../utility/include/rq_sensor_com.h"
#include "../../utility/include/rq_sensor_state.h"
#include "../../utility/include/rq_sensor_socket.h"
#include "../../utility/include/rq_int.h"
#include "../../utility/include/rq_thread.h"

#include <QObject>
#include <QTimer>
#include <atomic>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include "QDebug"

struct FtData
{
   float Fx;
   float Fy;
   float Fz;
   float Mx;
   float My;
   float Mz;
};

class RobotiqSensor : public QObject, public FTSensor
{
    Q_OBJECT
public:
    RobotiqSensor();
    int wait_for_other_connection();
    void get_data(INT_8 *chr_return);
    bool obtainFTSensorData(float m_ftData[SENSOR_DIMENSION]);
    bool initialFTSensor();
    bool uninitialFTSensor();
    void getFTSensorRange(double range[]);

    void FT_ManualSetZero();
    bool FT_Calibration_X();
    bool FT_Calibration_Y();
    bool FT_Calibration_Z();

public slots:
    bool Start_DataStream();

private:
    std::atomic<float> FX;
    std::atomic<float> FY;
    std::atomic<float> FZ;
    std::atomic<float> MX;
    std::atomic<float> MY;
    std::atomic<float> MZ;

    static double s_range[SENSOR_DIMENSION];
};
#endif // ROBOTIQ_SENSOR
