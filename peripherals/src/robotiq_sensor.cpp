#include "../include/RobotiqSensor.h"

double RobotiqSensor::s_range[SENSOR_DIMENSION] = {300,300,800,10,10,15};

/**
 * 函数功能: RobotiqSensor构造函数(初始化信息)
 *
 * 输入参数: 无
 *
 * 输出参数: 无
 *
 *
 * 备注： 无
 *
 */
RobotiqSensor::RobotiqSensor()
{
    FX = 0.0;
    FY = 0.0;
    FZ = 0.0;
    MX = 0.0;
    MY = 0.0;
    MZ = 0.0;
}

/**
 * 函数功能:  FT300库1
 *
 *
 * 备注： 无
 *
 */
int RobotiqSensor::wait_for_other_connection()
{
    INT_8 ret;
    struct timespec tim;
    tim.tv_sec = 0;
    tim.tv_nsec = 1e6L;
    int N = 0;

    while(N < 1)
    {
        nanosleep(&tim, (struct timespec *)NULL);
        ret = rq_sensor_state();
        if(ret == 0)
        {
            break;
        }
        N++;
    }
    return ret;
}

/**
 * 函数功能: FT300库2
 *
 *
 * 备注： 无
 *
 */
void RobotiqSensor::get_data(INT_8 *chr_return)
{
    INT_8 i;
    INT_8 floatData[50];
    for(i = 0; i < 6; i++)
    {
        sprintf(floatData, "%f", rq_state_get_received_data(i));
        if(i == 0)
        {
            strcpy(chr_return, "( ");
            strcat(chr_return, floatData);
        }
        else
        {
            strcat(chr_return," , ");
            strcat(chr_return,floatData);
        }
        if(i == 5)
        {
            strcat(chr_return, " )");
        }
    }
}

/**
 * 函数功能: 开启数据流
 *
 * 输入参数: 无
 *
 * 输出参数:
 *          true(开启数据流成功)
 *          false(开启数据流失败)
 * 备注： 无
 *
 */
bool RobotiqSensor::Start_DataStream()
{
    INT_8 bufStream[512];
    INT_8 ret = rq_sensor_state();
    QString Info;


   if(rq_sensor_get_current_state() == RQ_STATE_RUN)
    {
        strcpy(bufStream,"");
        get_data(bufStream);
        //qDebug()<<bufStream;

        /** 数组转字符串 **/
        for(int i=2;i<75;i++)
        {
            Info.append(bufStream[i]);
        }

        Info = Info.simplified().remove(" ");
        QString Fx = Info.section(",",0,0).remove(5,5);
        QString Fy = Info.section(",",1,1).remove(5,5);
        QString Fz = Info.section(",",2,2).remove(5,5);
        QString Mx = Info.section(",",3,3).remove(5,5);
        QString My = Info.section(",",4,4).remove(5,5);
        QString Mz = Info.section(",",5,5).remove(5,10);

        //源数据开始向多线程进行安全读取
        FX = Fx.toFloat();
        FY = Fy.toFloat();
        FZ = Fz.toFloat();
        MX = Mx.toFloat();
        MY = My.toFloat();
        MZ = Mz.toFloat();

        return true;
   }
   else
   {
       return false;
   }
}

/**
 * 函数功能: 对外获取FT300的力和力矩数据
 *
 * 输入参数: m_fx, m_fy, m_fz( 力<引用> )
 *          m_mx, m_my, m_mz( 力矩<引用> )
 */
bool RobotiqSensor::obtainFTSensorData(float m_ftData[SENSOR_DIMENSION])
{
    m_ftData[0] = FX;
    m_ftData[1] = FY;
    m_ftData[2] = FZ;
    m_ftData[3] = MX;
    m_ftData[4] = MY;
    m_ftData[5] = MZ;
    return true;
}

/**
 * 函数功能: FT300传感器的状态提前连接初始化
 */

bool RobotiqSensor::initialFTSensor()
{
    INT_8 ret = rq_sensor_state();
    //Read high-level informations
//    if(ret == -1)
//    {
//        wait_for_other_connection();
//    }
    return ret==0;
}

/**
 * 函数功能: 停止数据流定时器，断开与FT300的连接
 * 备注：
 *      发送断开信号并将连接状态ConnectStatus置为false
 *
 */
bool RobotiqSensor::uninitialFTSensor()
{
    FX = 0.0;
    FY = 0.0;
    FZ = 0.0;
    MX = 0.0;
    MY = 0.0;
    MZ = 0.0;

    FT300_Disconnect();
    init_sensor_state();
    initialFTSensor();
    return true;
}

/**
 * 函数功能: 手动将FT300传感器数据置零（断点重新上电后无效）
 * 备注： 无
 *
 */
void RobotiqSensor::FT_ManualSetZero()
{
    rq_set_zero();
}

/**
 * 函数功能: FT300校准传感器X轴
 * 输出参数: 成功返回true，失败返回false
 */
bool RobotiqSensor::FT_Calibration_X()
{
    bool calibration_state = false;
    init_sensor_state();                   //将连接状态设置为0，从头开始连接
    calibration_state = Calibration_FT300_X();
    return calibration_state;
}

/**
 * 函数功能: FT300校准传感器Y轴
 * 输出参数: 成功返回true，失败返回false
 */
bool RobotiqSensor::FT_Calibration_Y()
{
    bool calibration_state = false;
    init_sensor_state();                   //将连接状态设置为0，从头开始连接
    calibration_state = Calibration_FT300_Y();
    return calibration_state;
}

/**
 * 函数功能: FT300校准传感器Z轴
 * 输入参数: 无
 * 输出参数: 成功返回true，失败返回false
 * 备注： 无
 *
 */
bool RobotiqSensor::FT_Calibration_Z()
{
    bool calibration_state = false;
    init_sensor_state();                   //将连接状态设置为0，从头开始连接
    calibration_state = Calibration_FT300_Z();
    return calibration_state;
}

void RobotiqSensor::getFTSensorRange(double range[])
{
    memcpy(range, s_range, sizeof(double)*SENSOR_DIMENSION);
}
