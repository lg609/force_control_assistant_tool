#ifndef KUNWEI_SERIAL_H
#define KUNWEI_SERIAL_H

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

class SerialPort
{
public:
    SerialPort(){}
    //!
    ~SerialPort(){}

    //!
    int openPort(const char* device_name);
    //!
    int setPara(int serialfd, int baudLevel = 8, int databits = 8, int stopbits = 1, int parity = 0);
    //!
    int writeData(int fd, const char *data, int datalength);
    //!
    int readData(int fd,unsigned char *data,int datalength);
    //!
    void closePort(int fd);
    //!
    int getBaudRate(int level);
};
#endif
