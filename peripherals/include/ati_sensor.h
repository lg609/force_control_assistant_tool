#ifndef ATISENSOR_H
#define ATISENSOR_H

#include "ftsensor.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <iostream>
#include <netdb.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#pragma pack(1)
#define ATI_PORT			49152               /* Port the Ethernet DAQ always uses */
#define ATI_COMMAND				2
#define NUM_SAMPLES             1
#define ATI_IP_ADDRESS

typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;
typedef int SOCKET_HANDLE;

typedef struct ATIResponseStruct {
    uint32 rdt_sequence;
    uint32 ft_sequence;
    uint32 status;
    int32 FTData[6];
} ATIResponse;


class ATISensor: public FTSensor
{
public:
    ATISensor(std::string ipAddress = "192.168.1.1");

public:
    void mySleep(unsigned long ms);
    int connectSensor(SOCKET_HANDLE * handle, uint16 port);
    void closeSensor(SOCKET_HANDLE * handle);
    int readSensor(SOCKET_HANDLE * socket, ATIResponse &resp);
    void showResponse(ATIResponse * r, float ft[]);

    bool obtainFTSensorData(float m_ftData[SENSOR_DIMENSION]);
    bool initialFTSensor();
    bool uninitialFTSensor();
    void getFTSensorRange(double range[]);

private:
    SOCKET_HANDLE socketHandle;		/* Handle to UDP socket used to communicate with Ethernet DAQ. */
    ATIResponse ftResponse;
    static double s_range[SENSOR_DIMENSION];
    std::string ipAddress;

};
#endif // ATISENSOR_H
