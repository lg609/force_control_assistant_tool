#include "../include/OptoForceSensor.h"

CalibrationResponse calibrationResponse;  // global variable to hold calibration info datas

double OptoForceSensor::s_range[SENSOR_DIMENSION] = {300,300,800,10,10,15};

 OptoForceSensor::OptoForceSensor()
 {

 }

 void OptoForceSensor::getFTSensorRange(double range[])
 {
     memcpy(range, s_range, sizeof(double)*SENSOR_DIMENSION);
 }

int OptoForceSensor::Connect(SOCKET_HANDLE * handle, const char * ipAddress, uint16 port)
{
	struct sockaddr_in addr;
	struct hostent *he;
	int err;

	*handle = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    if (*handle == -1)
    {
		fprintf(stderr, "Socket could not be opened.\n");
		return -2;
	}
	he = gethostbyname(ipAddress);
	memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	fprintf(stdout, "Connecting to EtherDAQ\r\n");
	fflush(stdout);
	err = connect(*handle, (struct sockaddr *)&addr, sizeof(addr));
	if (err < 0) {
		return -3;
	}
	return 0;
}

void OptoForceSensor::Close(SOCKET_HANDLE * handle)
{
	close(*handle);
}

void OptoForceSensor::ShowCalibrationInfo(CalibrationResponse * r)
{
	int i;
	if (r == NULL) {
		return;
	}
	fprintf(stdout, "Header: 0x%x\r\nForcfe Units: %u\r\nTorque Units: %u\r\nCounts Per Force: %u\r\nCounts Per Torque: %u\r\n",
		r->header, r->forceUnits, r->torqueUnits, r->countsPerForce, r->countsPerTorque);


	for (i = 0; i < 6; ++i) {
		fprintf(stdout, "scaleFactors[%d]: %u\r\n", i, r->scaleFactors[i]);
	}
	fflush(stdout);
}

int OptoForceSensor::GetCalibrationInfo(SOCKET_HANDLE *socket)
{
	int i;
	int sendSuccess;
	int readSuccess;
	ReadCalibrationCommand readCommand = { 0 };
	readCommand.command = READCALIBRATIONINFO;
	sendSuccess = send(*socket, (const char *)&readCommand, sizeof(ReadCalibrationCommand), 0);
	if (sendSuccess < 0) {
		return sendSuccess;
	}

	readSuccess = recv(*socket, (char *)&calibrationResponse, sizeof(CalibrationResponse), 0);
	if (readSuccess < 0) {
		return readSuccess;
	}
	calibrationResponse.header = htons(calibrationResponse.header);
	calibrationResponse.countsPerForce = ntohl(calibrationResponse.countsPerForce);
	calibrationResponse.countsPerTorque = ntohl(calibrationResponse.countsPerTorque);
	for (i = 0; i < 6; ++i) {
		calibrationResponse.scaleFactors[i] = htons(calibrationResponse.scaleFactors[i]);
	}
	if (calibrationResponse.header != 0x1234) {
		return -1;
	}
	return 0;
}


int16 OptoForceSensor::swap_int16(int16 val)
{
	return (val << 8) | ((val >> 8) & 0xFF);
}

void OptoForceSensor::SwapFTResponseBytes(FTResponse * r)
{
	r->header = htons(r->header);
	r->status = htons(r->status);
	r->ForceX = swap_int16(r->ForceX);
	r->ForceY = swap_int16(r->ForceY);
	r->ForceZ = swap_int16(r->ForceZ);
	r->TorqueX = swap_int16(r->TorqueX);
	r->TorqueY = swap_int16(r->TorqueY);
	r->TorqueZ = swap_int16(r->TorqueZ);
}


int OptoForceSensor::ReadFT(SOCKET_HANDLE * socket, FTResponse * r)
{
	FTReadCommand readCommand = { 0 };
	int readSuccess;
	int sendSuccess;
	readCommand.command = READFT;
    try
    {
        sendSuccess = send(*socket, (char *)&readCommand, sizeof(FTReadCommand), 0);
    }
    catch(...)
    {
        return 3;

    }
	if (sendSuccess < 0) {
		return sendSuccess;
	}
	readSuccess = recv(*socket, (char *)r, sizeof(FTResponse), 0);
	if (readSuccess != sizeof(FTResponse)) {
		return 1;
	}
	SwapFTResponseBytes(r);
	if (r->header != 0x1234) {
		return 2;
	}
	return 0;
}


void OptoForceSensor::ShowResponse(FTResponse * r, float ft[])
{

	double Fx = (double)r->ForceX / (double)calibrationResponse.countsPerForce * (double)calibrationResponse.scaleFactors[0];
	double Fy = (double)r->ForceY / (double)calibrationResponse.countsPerForce * (double)calibrationResponse.scaleFactors[1];
	double Fz = (double)r->ForceZ / (double)calibrationResponse.countsPerForce * (double)calibrationResponse.scaleFactors[2];
	double Tx = (double)r->TorqueX / (double)calibrationResponse.countsPerTorque * (double)calibrationResponse.scaleFactors[3];
	double Ty = (double)r->TorqueY / (double)calibrationResponse.countsPerTorque * (double)calibrationResponse.scaleFactors[4];
	double Tz = (double)r->TorqueZ / (double)calibrationResponse.countsPerTorque * (double)calibrationResponse.scaleFactors[5];
    ft[0] = Fx; ft[1] = Fy; ft[2] = Fz; ft[3] = Tx; ft[4] = Ty; ft[5] = Tz;
    if (calibrationResponse.forceUnits == 2 && calibrationResponse.torqueUnits == 3)
    {
//		fprintf(stdout, "Status: %u Fx:%.4f N Fy:%.4f N Fz: %.4f N Tx: %.4f Nm Ty: %.4f Nm Tz: %.4f Nm\r\n", r->status, Fx, Fy, Fz, Tx, Ty, Tz);
	}
    else
    {
//		fprintf(stdout, "Status: %u Fx:%.4f Fy:%.4f Fz: %.4f Tx: %.4f Ty: %.4f Tz: %.4f\r\n", r->status, Fx, Fy, Fz, Tx, Ty, Tz);
	}
	fflush(stdout);
}


bool OptoForceSensor::initialFTSensor()
{
    bool ret = false;

    if (Connect(&socketHandle, IPADDRESS, PORT) != 0)
    {
		fprintf(stderr, "Could not connect to device...\r\n");
        return ret;
	}
    else
    {
		fprintf(stdout, "Connected to Ethernet DAQ\r\n");
		fflush(stdout);
	}

    if (GetCalibrationInfo(&socketHandle) != 0)
    {
		fprintf(stderr, "Could not read calibration info...\r\n");
        return ret;
	}
	ShowCalibrationInfo(&calibrationResponse);
	
    ret = true;
    return ret;
}

bool OptoForceSensor::obtainFTSensorData(float m_ftData[SENSOR_DIMENSION])
{
    int readSuccess;
    try
    {
        readSuccess = ReadFT(&socketHandle, &ftResponse);
    }
    catch(...)
    {

    }

    if (readSuccess == 0)
    {
        ShowResponse(&ftResponse, m_ftData);
    }
    else
    {
        fprintf(stderr, "Could not read F/T data, error code: %d\r\n", readSuccess);
    }

    return readSuccess==0;
}

bool OptoForceSensor::uninitialFTSensor()
{
    close(socketHandle);
    return true;
}

