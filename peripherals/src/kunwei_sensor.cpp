#include "kunwei_sensor.h"
#include  <stdio.h>
#include  <stdlib.h> // malloc
#include  <unistd.h>
#include  <sys/types.h>
#include  <sys/signal.h>
#include  <sys/stat.h>
#include  <fcntl.h>
#include  <termios.h>
#include  <errno.h>
#include  <limits.h>
#include  <string.h>
#include  <time.h>
#include  <pthread.h>
#include  <assert.h>

//KunWeiResponse KunWeiSensor::ftResponse;
//bool KunWeiSensor::sensor_running_ = false;
//char KunWeiSensor::command_head_ = 0;

KunWeiSensor::KunWeiSensor(std::string port_name)
{
    port_name_  = "/dev/" + port_name;
#ifndef NEW_KUNWEI_SENSOR
    std::cout<<"port_name:"<<port_name<<std::endl;
    serial_port_ = new SerialPort();
    m_serial_fd_ = serial_port_->openPort(port_name_.c_str());
    memset(&ftResponse, 0, sizeof(KunWeiResponse));
#else
    dev = (KunWeiResponse *)malloc(sizeof(KunWeiResponse));
    dev->fd = open(port_name_.c_str(), O_RDWR | O_NOCTTY |O_NONBLOCK);
    if(dev->fd < 0)
    {
        fprintf(stderr, "Open serial port %s failed!\n", port_name_.c_str());
    }
#endif
}

bool KunWeiSensor::readConfig()
{
    baud_level_ = 7;
    return true;
}

KunWeiSensor::~KunWeiSensor()
{
#ifndef NEW_KUNWEI_SENSOR
    read_sensor_data_.join();

    serial_port_->closePort(m_serial_fd_);
    if(serial_port_ != NULL)
        delete serial_port_;
#else
//    assert(dev);
//    dev->running = 0;
//    pthread_join(dev->tid, NULL);

//    close(dev->fd);
#endif
}

#ifdef NEW_KUNWEI_SENSOR
static void* kwr_recv_handler(void* arg)
{
    KunWeiResponse *dev = (KunWeiResponse *)arg;
    //read and deal
    unsigned char readdata[1024];
    char frame[16];
    int frame_index = 0;
    memset(readdata, 0, 1024); //the length//////////////////////////////
    int max_fd = 0;
    fd_set readset = {0};
    struct timeval tv = {0};
    int len;
    int state = 0;

    while (dev->running)
    {
        len = 0;
        FD_ZERO(&readset);
        FD_SET((unsigned int)dev->fd, &readset);
        max_fd = dev->fd +1;
        tv.tv_sec=0;
        tv.tv_usec=0;
        if (select(max_fd, &readset, NULL, NULL, &tv ) < 0) {
            printf("ReadData: select error\n");
            return NULL;
        }
        int nRet = FD_ISSET(dev->fd, &readset);
        if (nRet) {
            len = read(dev->fd, readdata, 150);
        }

        //! 连续反馈模式
        if (dev->mode == CONTINUOUS_MODE) {
            int index = 0;
            while (index < len) {
                char ch = readdata[index++];
                switch (state) {
                case 0:
                    if (ch == 0x48) {
                        state = 1;
                        frame[frame_index++] = ch;
                    }
                    break;
                case 1:
                    // Fx Fy Fz Mx My Mz: 1-13
                    frame[frame_index++] = ch;
                    if (frame_index == 13) {
                        state = 2;
                    }
                    break;
                case 2:
                    // 0x0D
                    if (ch == 0x0D) {
                        state = 3;
                        frame[frame_index++] = ch;
                    } else {
                        state = 0;
                        frame_index = 0;
                    }
                    break;
                case 3:
                    // 0x0A
                    if (ch == 0x0A) {
                        state = 4;
                        frame[frame_index++] = ch;
                    } else {
                        state = 0;
                        frame_index = 0;
                    }
                    break;
                case 4:
                    // 完整正确的frame
                    data2force(&frame[1], dev->FTData);
                    dev->force_available = 1;
                    frame_index = 0;
                    state = 0;
                    break;
                default:
                    break;
                }
            }
        }
        else if (dev->mode == REQUEST_MODE)
        {

        }

    }

    return (void*)0;
}
#endif

bool KunWeiSensor::initialFTSensor()
{
#ifndef NEW_KUNWEI_SENSOR
    if(!readConfig())
        return false;

    bool ret = false;
    sensor_running_ = false;
    if(m_serial_fd_ > 0)
    {
        serial_port_->setPara(m_serial_fd_, baud_level_);
        writeCommand(INIT);
        writeCommand(CLEAR);
        writeCommand(INIT);
        sensor_running_ = true;
        read_sensor_data_ = std::thread(&KunWeiSensor::handleSensorData2, this);
        if (!read_sensor_data_.joinable())
        {
            std::cout<<"ERROR; return code from pthread_create() is"<<std::endl;
            exit(-1);
        }
        else
        {
            std::cout<<"Start the read sensor thread!"<<std::endl;
            ret = true;
        }
    }
    else
    {
        std::cout<<"Open serial port failed!"<<std::endl;
        ret = false;
    }

    return ret;
#else
    struct termios OnesensorTermios_old;
    struct termios OnesensorTermios;

    tcgetattr(dev->fd , &OnesensorTermios_old);
    bzero( &OnesensorTermios, sizeof(OnesensorTermios));
    cfmakeraw(&OnesensorTermios);
    OnesensorTermios.c_cflag=B230400;
    OnesensorTermios.c_cflag |= CLOCAL | CREAD;
    OnesensorTermios.c_cflag &= ~CSIZE;
    OnesensorTermios.c_cflag |= CS8;
    OnesensorTermios.c_cflag &= ~PARENB;
    OnesensorTermios.c_cflag &= ~CSTOPB;
    tcflush(dev->fd,TCIFLUSH);
    tcflush(dev->fd,TCOFLUSH);
    OnesensorTermios.c_cc[VTIME] = 1;
    OnesensorTermios.c_cc[VMIN] = 1;
    tcflush (dev->fd, TCIFLUSH);
    tcsetattr(dev->fd,TCSANOW,&OnesensorTermios);

    dev->force_available = 0;
    dev->running = 1;
    pthread_create(&dev->tid, 0, kwr_recv_handler, dev);
#endif

}

bool KunWeiSensor::uninitialFTSensor()
{

#ifndef NEW_KUNWEI_SENSOR
    sensor_running_ = false;
    usleep(3*1000);
    serial_port_->closePort(m_serial_fd_);
    if(serial_port_ != NULL)
        delete serial_port_;
#else
    assert(dev);
    dev->running = 0;
    pthread_join(dev->tid, NULL);

    close(dev->fd);
#endif

    return true;
}


void KunWeiSensor::getFTSensorRange(double range[])
{
    double s_range[SENSOR_DIMENSION];
    if(1 /*sensor_name_ == ""*/)
    {
        s_range[0] = 300;
        s_range[1] = 300;
        s_range[2] = 800;
        s_range[3] = 10;
        s_range[4] = 10;
        s_range[5] = 15;
    }
    memcpy(range, s_range, sizeof(double)*SENSOR_DIMENSION);
}

bool KunWeiSensor::obtainFTSensorData(float m_ftData[SENSOR_DIMENSION])
{
#ifndef NEW_KUNWEI_SENSOR
    memcpy(m_ftData, ftResponse.FTData, sizeof(float)*SENSOR_DIMENSION);
    return true;
#else
    assert(dev);
    if (dev->force_available) {
        memcpy(m_ftData, dev->FTData, sizeof(float)*6);
        return true;
    }
    return false;
#endif
}


#ifndef NEW_KUNWEI_SENSOR
int KunWeiSensor::writeCommand(SENSOR_COMMAND command)
{
    char data[4]= {0};
    switch(command)
    {
        case INIT:data[0] = 0x43; data[1] = 0xAA; data[2] = 0x0D; data[3] = 0x0A;break;
        case CONTINUOUS_MODE:data[0] = 0x48; data[1] = 0xAA; data[2] = 0x0D; data[3] = 0x0A;break;
        case REQUEST_MODE:data[0] = 0x49; data[1] = 0xAA; data[2] = 0x0D; data[3] = 0x0A;break;
        case CLEAR:data[0] = 0x47; data[1] = 0xAA; data[2] = 0x0D; data[3] = 0x0A;break;
    }

    command_head_ = data[0];
    if(serial_port_->writeData(m_serial_fd_, data, strlen(data)) < 0)
    {
           printf("Write Data Fail!\n");
           return -1;
    }
    return 0;
}

int KunWeiSensor::readSensor(unsigned char m_rec_data[REC_DATA_LEN])
{
    int m_rec_data_length = serial_port_->readData(m_serial_fd_,m_rec_data,REC_DATA_LEN);
    if( m_rec_data_length > 0 )
      return m_rec_data_length;
    else
      return -1;
}

static long ncount = 0;
static long mcount = 0;
//static std::vector<unsigned char> dataVector;
void KunWeiSensor::handleSensorData()
{
    unsigned char sensor_data_[1024];
    while(sensor_running_)
    {
        while(sensor_running_ /*&& serial_port_->waitForReadyRead(2)*/)
        {
            ncount++;
            int len;
            len = serial_port_->readData(m_serial_fd_, sensor_data_, 1024);
//            serial_port_->clear();

//                    std::cout<<"rate:::::::"<<mcount*100.0/ncount<<std::endl;
            if(len >= 27)
                if(sensor_data_[0] != 0x48 || sensor_data_[25] != 0x0D || sensor_data_[26] != 0x0A)
                    read_success_ = false;
                else
                {
                    mcount++;
                    unsigned char c[4] ;
                    for(int i = 0; i < SENSOR_DIMENSION; i++)
                    {
                        c[0] = sensor_data_[1+4*i];
                        c[1] = sensor_data_[2+4*i];
                        c[2] = sensor_data_[3+4*i];
                        c[3] = sensor_data_[4+4*i];
                        memcpy(&ftResponse.FTData[i], c, sizeof(float));
                        //                     memcpy(&ftResponse.FTData[i], sensor_data_+1+4*i, sizeof(float));
                        ftResponse.FTData[i] = ftResponse.FTData[i] * GRAVITY_ACC_;
                    }
                    read_success_ = true;
                }
        }
    }
}

void KunWeiSensor::handleSensorData2()
{
    unsigned char m_rec_data[REC_DATA_LEN];
    int control_period_ = 3;
    int data_size;

    while(sensor_running_)
    {
        struct timeb tb;
        ftime(&tb);
        int t1 = tb.millitm;
        setbuf(stdin,NULL); //设置缓冲区
        writeCommand(REQUEST_MODE);
        data_size = readSensor(m_rec_data);
        int index = 0;

        if(data_size >= 12)
        {
            while(m_rec_data[index] != command_head_)
                index++;
            unsigned int temp_data = 0;
            if(m_rec_data[index +10] == 0x0d && m_rec_data[index +11] == 0x0a)
            {
            if ((m_rec_data[index +1] & 0x80) > 0)
            {
                temp_data = m_rec_data[index +1];
                temp_data = temp_data|0xFFFFFF00;
                temp_data = temp_data << 4;
                temp_data = temp_data | (uint)(m_rec_data[index +2] >> 4);
            }
            else
            {
                temp_data = m_rec_data[index +1];
                temp_data = temp_data << 4;
                temp_data = temp_data | (uint)(m_rec_data[index +2] >> 4);
            }
            ftResponse.FTData[0] = (int)temp_data * (float)0.009765625 * GRAVITY_ACC_;
            temp_data = 0;
            if ((m_rec_data[index +2] & 0x08) > 0)
            {
                temp_data = m_rec_data[index +2];
                temp_data = temp_data|0xFFFFFFF0;
                temp_data = temp_data << 8;
                temp_data = temp_data | m_rec_data[index +3];
            }
            else
            {
                temp_data = m_rec_data[index + 2];
                temp_data = temp_data & 0x0F;
                temp_data = temp_data << 8;
                temp_data = temp_data | m_rec_data[index +3];
            }
            ftResponse.FTData[1] = (int)temp_data * (float)0.009765625 * GRAVITY_ACC_;
            temp_data = 0;
            if ((m_rec_data[index +4] & 0x80) > 0)
            {
                temp_data = m_rec_data[index +4];
                temp_data = temp_data|0xFFFFFF00;
                temp_data = temp_data << 4;
                temp_data = temp_data | (uint)(m_rec_data[index +5] >> 4);
            }
            else
            {
                temp_data = m_rec_data[index +4];
                temp_data = temp_data << 4;
                temp_data = temp_data | (uint)(m_rec_data[index +5] >> 4);
            }
            ftResponse.FTData[2] = (int)temp_data * (float)0.009765625 * GRAVITY_ACC_;
            temp_data = 0;
            if ((m_rec_data[index +5] & 0x08) > 0)
            {
                temp_data = m_rec_data[index +5];
                temp_data = temp_data|0xFFFFFFF0;
                temp_data = temp_data << 8;
                temp_data = temp_data | m_rec_data[index +6];
            }
            else
            {
                temp_data = m_rec_data[index +5];
                temp_data = temp_data & 0x0F;
                temp_data = temp_data << 8;
                temp_data = temp_data | m_rec_data[index +6];
            }
            ftResponse.FTData[3] = (int)temp_data * (float)0.000390625 * GRAVITY_ACC_;
            temp_data = 0;
            if ((m_rec_data[index +7] & 0x80) > 0)
            {
                temp_data = m_rec_data[index +7];
                temp_data = temp_data|0xFFFFFF00;
                temp_data = temp_data << 4;
                temp_data = temp_data | (uint)(m_rec_data[index +8] >> 4);
            }
            else
            {
                temp_data = m_rec_data[index +7];
                temp_data = temp_data << 4;
                temp_data = temp_data | (uint)(m_rec_data[index +8] >> 4);
            }
            ftResponse.FTData[4] = (int)temp_data * (float)0.000390625 * GRAVITY_ACC_;
            temp_data = 0;
            if ((m_rec_data[index +8] & 0x08) > 0)
            {
                temp_data = m_rec_data[index +8];
                temp_data = temp_data|0xFFFFFFF0;
                temp_data = temp_data << 8;
                temp_data = temp_data | m_rec_data[index +9];
            }
            else
            {
                temp_data = m_rec_data[index +8];
                temp_data = temp_data & 0x0F;
                temp_data = temp_data << 8;
                temp_data = temp_data | m_rec_data[index +9];
            }
            ftResponse.FTData[5] = (int)temp_data * (float)0.000390625 * GRAVITY_ACC_;
            }
        }

//        std::cout<<"FTData:"<<ftResponse.FTData[5]<<","<<ftResponse.FTData[5]<<","<<ftResponse.FTData[5]<<std::endl;
        ftime(&tb);
        int t2 = tb.millitm;
        int sleepT = t2 - t1;
        if(sleepT < 0) sleepT += 1000;
        double ts = (control_period_-sleepT);
        if(ts < 0)
        {
            ts = 0;
//            std::cout<<"FTData: err"<<std::endl;
        }
        usleep(ts*1000);
    }

}

#else
int KunWeiSensor::kwr_ft_zero()
{
    assert(dev);
    char writedata[10]= {0};
    writedata[0] = 0x47;
    writedata[1] = 0xAA;
    writedata[2] = 0x0D;
    writedata[3] = 0x0A;
    int len = 0, total_len = 0;
    while ( total_len < 4 )
    {
        len = 0;
        len = write(dev->fd, &writedata[total_len], 4 - total_len);
        if (len > 0)
        {
            total_len += len;
        }
        else if(len < 0)
        {
            return -1;
        }
    }
    return 0;
}


int KunWeiSensor::kwr_read_continous_request()
{
    assert(dev);
    char writedata[10]= {0};
    writedata[0] = 0x48;
    writedata[1] = 0xAA;
    writedata[2] = 0x0D;
    writedata[3] = 0x0A;
    int len = 0, total_len = 0;
    while ( total_len < 4 )
    {
        len = 0;
        len = write(dev->fd, &writedata[total_len], 4 - total_len);
        if (len > 0)
        {
            total_len += len;
        }
        else if(len < 0)
        {
            return -1;
        }
    }
    dev->mode = CONTINUOUS_MODE;
    return 0;
}

void data2force(char* data, float* Force)
{
    char DataTemp = 0;
    if ((data[1] & 0x80) > 0)
    {
        DataTemp = data[1];
        DataTemp = DataTemp|0xFFFFFF00;
        DataTemp = DataTemp << 4;
        DataTemp = DataTemp | (uint)(data[2] >> 4);
    }
    else
    {
        DataTemp = data[1];
        DataTemp = DataTemp << 4;
        DataTemp = DataTemp | (uint)(data[2] >> 4);
    }
    Force[0] = (int)DataTemp * (float)0.009765625;
    DataTemp = 0;
    if ((data[2] & 0x08) > 0)
    {
        DataTemp = data[2];
        DataTemp = DataTemp|0xFFFFFFF0;
        DataTemp = DataTemp << 8;
        DataTemp = DataTemp | data[3];
    }
    else
    {
        DataTemp = data[2];
        DataTemp = DataTemp & 0x0F;
        DataTemp = DataTemp << 8;
        DataTemp = DataTemp | data[3];
    }
    Force[1] = (int)DataTemp * (float)0.009765625;
    DataTemp = 0;
    if ((data[4] & 0x80) > 0)
    {
        DataTemp = data[4];
        DataTemp = DataTemp|0xFFFFFF00;
        DataTemp = DataTemp << 4;
        DataTemp = DataTemp | (uint)(data[5] >> 4);
    }
    else
    {
        DataTemp = data[4];
        DataTemp = DataTemp << 4;
        DataTemp = DataTemp | (uint)(data[5] >> 4);
    }
    Force[2] = (int)DataTemp * (float)0.009765625;
    DataTemp = 0;
    if ((data[5] & 0x08) > 0)
    {
        DataTemp = data[5];
        DataTemp = DataTemp|0xFFFFFFF0;
        DataTemp = DataTemp << 8;
        DataTemp = DataTemp | data[6];
    }
    else
    {
        DataTemp = data[5];
        DataTemp = DataTemp & 0x0F;
        DataTemp = DataTemp << 8;
        DataTemp = DataTemp | data[6];
    }
    Force[3] = (int)DataTemp * (float)0.000390625;
    DataTemp = 0;
    if ((data[7] & 0x80) > 0)
    {
        DataTemp = data[7];
        DataTemp = DataTemp|0xFFFFFF00;
        DataTemp = DataTemp << 4;
        DataTemp = DataTemp | (uint)(data[8] >> 4);
    }
    else
    {
        DataTemp = data[7];
        DataTemp = DataTemp << 4;
        DataTemp = DataTemp | (uint)(data[8] >> 4);
    }
    Force[4] = (int)DataTemp * (float)0.000390625;
    DataTemp = 0;
    if ((data[8] & 0x08) > 0)
    {
        DataTemp = data[8];
        DataTemp = DataTemp|0xFFFFFFF0;
        DataTemp = DataTemp << 8;
        DataTemp = DataTemp | data[9];
    }
    else
    {
        DataTemp = data[8];
        DataTemp = DataTemp & 0x0F;
        DataTemp = DataTemp << 8;
        DataTemp = DataTemp | data[9];
    }
    Force[5] = (int)DataTemp * (float)0.000390625;
}

#endif
