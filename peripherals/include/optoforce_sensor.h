#ifndef FTSENSOR_H
#define FTSENSOR_H

#include "ftsensor.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#pragma pack(1)
#define PORT			49151       /* Port the Ethernet DAQ always uses */
#define READFT				0
#define READCALIBRATIONINFO 1
#define IPADDRESS "192.168.1.1"

typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;
typedef int SOCKET_HANDLE;

typedef struct FTResponseStruct {
    uint16 header;
    uint16 status;
    int16 ForceX;
    int16 ForceY;
    int16 ForceZ;
    int16 TorqueX;
    int16 TorqueY;
    int16 TorqueZ;
} FTResponse;

typedef struct CalibrationResponseStruct
{
    uint16 header;
    byte forceUnits;
    byte torqueUnits;
    uint32 countsPerForce;
    uint32 countsPerTorque;
    uint16 scaleFactors[6];
} CalibrationResponse;

typedef struct FTReadCommandStruct
{
    byte command;
    byte reserved[19];
}FTReadCommand;

typedef struct ReadCalibrationCommandStruct {
    byte command;
    byte reserved[19];
} ReadCalibrationCommand;

class OptoForceSensor: public FTSensor
{
public:
    OptoForceSensor();

public:
    int Connect(SOCKET_HANDLE * handle, const char * ipAddress, uint16 port);
    void Close(SOCKET_HANDLE * handle);
    void ShowCalibrationInfo(CalibrationResponse * r);
    int GetCalibrationInfo(SOCKET_HANDLE *socket);
    int16 swap_int16(int16 val);
    void SwapFTResponseBytes(FTResponse * r);
    int ReadFT(SOCKET_HANDLE * socket, FTResponse * r);
    void ShowResponse(FTResponse * r, float ft[]);

    bool obtainFTSensorData(float m_ftData[SENSOR_DIMENSION]);
    bool initialFTSensor();
    bool uninitialFTSensor();
    void getFTSensorRange(double range[]);

private:
    SOCKET_HANDLE socketHandle;		/* Handle to UDP socket used to communicate with Ethernet DAQ. */
    FTResponse ftResponse;
    static double s_range[SENSOR_DIMENSION];

};
#endif // FTSENSOR_H
