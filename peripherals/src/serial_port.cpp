#include "serial_port.h"
#include <stdio.h>

int SerialPort::openPort(const char* device_name)
{
    struct termios termios_old;
    int fd = open( device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        fprintf(stderr, "Open serial port %s failed!\n", device_name);
        return -1;
    }
    tcgetattr(fd , &termios_old);
    return fd;
}

int SerialPort::setPara(int serialfd, int baud, int databits, int stopbits, int parity )
{
    struct termios termios_new;
    bzero(&termios_new, sizeof(termios_new));
    cfmakeraw(&termios_new);
    termios_new.c_cflag = getBaudRate(baud);
    termios_new.c_cflag |= CLOCAL | CREAD;

    termios_new.c_cflag &= ~CSIZE;
    switch (databits)
    {
    case 0:
        termios_new.c_cflag |= CS5;
        break;
    case 1:
        termios_new.c_cflag |= CS6;
        break;
    case 2:
        termios_new.c_cflag |= CS7;
        break;
    case 3:
        termios_new.c_cflag |= CS8;
        break;
    default:
        termios_new.c_cflag |= CS8;
        break;
    }

    switch (parity)
    {
    case 0:
        termios_new.c_cflag &= ~PARENB;
        break;
    case 1:
        termios_new.c_cflag |= PARENB;
        termios_new.c_cflag &= ~PARODD;
        break;
    case 2:
        termios_new.c_cflag |= PARENB;
        termios_new.c_cflag |= ~PARODD;
        break;
    default:
        termios_new.c_cflag &= ~PARENB;
        break;
    }
    switch (stopbits)// set Stop Bit
    {
    case 1:
        termios_new.c_cflag &= ~CSTOPB;
        break;
    case 2:
        termios_new.c_cflag |= CSTOPB;
        break;
    default:
        termios_new.c_cflag &= ~CSTOPB;
        break;
    }
    tcflush(serialfd,TCIFLUSH);    // clear input buffer
    tcflush(serialfd,TCOFLUSH);    // clear output buffer
    termios_new.c_cc[VTIME] = 1;
    termios_new.c_cc[VMIN] = 1;
    tcflush (serialfd, TCIFLUSH);
    return tcsetattr(serialfd,TCSANOW,&termios_new);
}

int SerialPort::writeData(int fd, const char *data, int datalength)
{
    if(fd < 0)
        return -1;

    int len = 0, total_len = 0; //modify8.
    for (total_len = 0 ; total_len < datalength;)
    {
        len = 0;
        len = write(fd, &data[total_len], datalength - total_len);
        //        printf("WriteData fd = %d ,len =%d,data = %s\n",fd,len,data);
        if (len > 0)
            total_len += len;
        else if(len <= 0)
        {
            len = -1;
            break;
        }
    }
    return len;
}

int SerialPort::readData(int fd, unsigned char *data, int datalength)
{
    if(fd <0)
        return -1;

    int len = 0;
    memset(data, 0, datalength);

    int max_fd = 0;
    fd_set readset = {0};
    struct timeval tv = {0};

    FD_ZERO(&readset);
    FD_SET((unsigned int)fd, &readset);
    max_fd = fd + 1;
    tv.tv_sec = 0;
    tv.tv_usec = 1000;
    if (select(max_fd, &readset, NULL, NULL, &tv) < 0)
        printf("ReadData: select error\n");

    int nRet = FD_ISSET(fd, &readset);
    if (nRet)
        len = read(fd, data, datalength);

    return len;
}

void SerialPort::closePort(int fd)
{
    struct termios termios_old;
    if(fd > 0)
    {
        tcsetattr (fd, TCSADRAIN, &termios_old);
        close(fd);
    }
}

int SerialPort::getBaudRate(int level)
{
    switch(level)
    {
    case 0:
        return (B2400);
    case 1:
        return (B4800);
    case 2:
        return (B9600);
    case 3:
        return (B19200);
    case 4:
        return (B38400);
    case 5:
        return (B57600);
    case 6:
        return (B115200);
    case 7:
        return (B230400);
    case 8:
        return (B460800);
    default:
        return (B9600);
    }
}
